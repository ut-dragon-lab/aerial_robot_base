#!/usr/bin/env python3
"""Run a MuJoCo model and bridge its aerial-robot sensors/actuators to ROS 2."""

import hashlib
import math
import os
import random
import re
import signal
import shlex
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, MagneticField


_MJCF_GENERATOR_VERSION = "urdf_to_mjcf_v2"
_MUJOCO_VIEWER_FONT_SCALES = (50, 100, 150, 200, 250, 300)


def _xml_name(tag):
    return tag.rsplit("}", 1)[-1]


def _children(elem, name):
    return [child for child in elem if _xml_name(child.tag) == name]


def _first_child(elem, name):
    for child in elem:
        if _xml_name(child.tag) == name:
            return child
    return None


def _descendants(elem, name):
    return [child for child in elem.iter() if _xml_name(child.tag) == name]


def _parse_vec(text, default):
    if text is None:
        return list(default)
    values = [float(item) for item in text.split()]
    if len(values) < len(default):
        values.extend(default[len(values) :])
    return values[: len(default)]


def _format_float(value):
    if abs(value) < 1.0e-12:
        value = 0.0
    return f"{value:.9g}"


def _format_vec(values):
    return " ".join(_format_float(float(value)) for value in values)


def _safe_name(name):
    safe = re.sub(r"[^A-Za-z0-9_]+", "_", name)
    safe = safe.strip("_")
    return safe or "mesh"


class UrdfToMjcfGenerator:
    """Small URDF-to-MJCF converter for aerial-robot simulation models."""

    def __init__(self, logger):
        self.logger = logger
        self.output_path = None
        self.assets_dir = None
        self.asset = None
        self.links = {}
        self.parent_joints = {}
        self.mesh_assets = {}
        self.rotor_sites = []
        self.m_f_rate = 0.0

    def generate(self, urdf_xml, output_path):
        self.output_path = Path(output_path)
        self.assets_dir = self.output_path.parent / "assets"
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.assets_dir.mkdir(parents=True, exist_ok=True)
        self.mesh_assets = {}
        self.rotor_sites = []

        urdf_root = ET.fromstring(urdf_xml)
        robot_name = _safe_name(urdf_root.get("name", "robot"))
        self.links = {link.get("name"): link for link in _children(urdf_root, "link") if link.get("name")}
        self.parent_joints = {}
        child_links = set()
        for joint in _children(urdf_root, "joint"):
            parent = _first_child(joint, "parent")
            child = _first_child(joint, "child")
            if parent is None or child is None:
                continue
            parent_name = parent.get("link")
            child_name = child.get("link")
            if not parent_name or not child_name:
                continue
            self.parent_joints.setdefault(parent_name, []).append(joint)
            child_links.add(child_name)

        root_links = [name for name in self.links if name not in child_links]
        if not root_links:
            raise RuntimeError("URDF does not contain a root link")
        dynamic_root = self._select_dynamic_root(root_links[0])
        self.m_f_rate = self._read_float_element(urdf_root, "m_f_rate", 0.0)

        mjcf_root = ET.Element("mujoco", {"model": robot_name})
        ET.SubElement(
            mjcf_root,
            "compiler",
            {"angle": "radian", "inertiafromgeom": "false", "meshdir": "assets"},
        )
        ET.SubElement(
            mjcf_root,
            "option",
            {"timestep": "0.001", "integrator": "RK4", "gravity": "0 0 -9.80665"},
        )

        self.asset = ET.SubElement(mjcf_root, "asset")
        ET.SubElement(
            self.asset,
            "texture",
            {
                "name": "checker",
                "type": "2d",
                "builtin": "checker",
                "width": "512",
                "height": "512",
                "rgb1": "0.02 0.04 0.07",
                "rgb2": "0.10 0.20 0.32",
            },
        )
        ET.SubElement(
            self.asset,
            "material",
            {"name": "checker", "texture": "checker", "texrepeat": "10 10", "reflectance": "0.25"},
        )

        default = ET.SubElement(mjcf_root, "default")
        ET.SubElement(default, "geom", {"condim": "3", "friction": "0.8 0.1 0.1"})
        ET.SubElement(default, "site", {"size": "0.004", "rgba": "0.1 0.8 0.1 1"})
        visual_default = ET.SubElement(default, "default", {"class": "visual"})
        ET.SubElement(visual_default, "geom", {"contype": "0", "conaffinity": "0", "group": "1"})
        collision_default = ET.SubElement(default, "default", {"class": "collision"})
        ET.SubElement(collision_default, "geom", {"rgba": "0 0 0 0", "group": "3"})

        visual = ET.SubElement(mjcf_root, "visual")
        ET.SubElement(visual, "global", {"offwidth": "1280", "offheight": "720"})

        worldbody = ET.SubElement(mjcf_root, "worldbody")
        ET.SubElement(worldbody, "light", {"name": "sun", "pos": "0 0 3", "dir": "0 0 -1"})
        ET.SubElement(
            worldbody,
            "geom",
            {"name": "ground", "type": "plane", "size": "6 6 0.02", "material": "checker"},
        )

        root_body = ET.SubElement(worldbody, "body", {"name": dynamic_root, "pos": "0 0 0"})
        ET.SubElement(root_body, "freejoint", {"name": "root"})
        self._add_link(root_body, dynamic_root)

        actuator = ET.SubElement(mjcf_root, "actuator")
        for rotor_name, max_force, yaw_gear in self.rotor_sites:
            ET.SubElement(
                actuator,
                "motor",
                {
                    "name": rotor_name,
                    "site": rotor_name,
                    "gear": _format_vec((0.0, 0.0, 1.0, 0.0, 0.0, yaw_gear)),
                    "ctrlrange": f"0 {_format_float(max_force)}",
                    "ctrllimited": "true",
                },
            )

        sensor = ET.SubElement(mjcf_root, "sensor")
        ET.SubElement(sensor, "accelerometer", {"name": "acc", "site": "fc"})
        ET.SubElement(sensor, "gyro", {"name": "gyro", "site": "fc"})
        ET.SubElement(sensor, "magnetometer", {"name": "mag", "site": "fc"})
        ET.SubElement(sensor, "framelinvel", {"name": "fc_vel", "objtype": "site", "objname": "fc"})
        ET.SubElement(sensor, "frameangvel", {"name": "fc_angvel", "objtype": "site", "objname": "fc"})

        if not self._has_site(root_body, "fc"):
            ET.SubElement(root_body, "site", {"name": "fc", "pos": "0 0 0"})

        ET.indent(mjcf_root, space="  ")
        ET.ElementTree(mjcf_root).write(self.output_path, encoding="utf-8", xml_declaration=True)
        return self.output_path

    def _select_dynamic_root(self, root_link_name):
        root_link = self.links[root_link_name]
        children = self.parent_joints.get(root_link_name, [])
        if not self._link_has_content(root_link) and len(children) == 1:
            joint = children[0]
            if joint.get("type", "fixed") == "fixed":
                child = _first_child(joint, "child")
                if child is not None and child.get("link") in self.links:
                    return child.get("link")
        return root_link_name

    def _link_has_content(self, link):
        if _children(link, "visual") or _children(link, "collision"):
            return True
        inertial = _first_child(link, "inertial")
        if inertial is None:
            return False
        mass = _first_child(inertial, "mass")
        return mass is not None and float(mass.get("value", "0")) > 1.0e-9

    def _add_link(self, body, link_name):
        link = self.links[link_name]
        self._add_inertial(body, link)
        self._add_geometries(body, link, "visual")
        self._add_geometries(body, link, "collision")

        if link_name == "fc" and not self._has_site(body, "fc"):
            ET.SubElement(body, "site", {"name": "fc", "pos": "0 0 0"})

        for joint in self.parent_joints.get(link_name, []):
            child = _first_child(joint, "child")
            if child is None or child.get("link") not in self.links:
                continue
            child_name = child.get("link")
            origin_xyz, origin_rpy = self._origin(joint)
            attrs = {"name": child_name, "pos": _format_vec(origin_xyz)}
            if any(abs(value) > 1.0e-12 for value in origin_rpy):
                attrs["euler"] = _format_vec(origin_rpy)
            child_body = ET.SubElement(body, "body", attrs)

            joint_name = joint.get("name", "")
            if self._is_rotor_joint(joint):
                ET.SubElement(child_body, "site", {"name": joint_name, "pos": "0 0 0"})
                self.rotor_sites.append(self._rotor_actuator_info(joint))

            self._add_link(child_body, child_name)

    def _add_inertial(self, body, link):
        inertial = _first_child(link, "inertial")
        if inertial is None:
            return

        mass_elem = _first_child(inertial, "mass")
        if mass_elem is None:
            return
        mass = float(mass_elem.get("value", "0"))
        if mass <= 1.0e-9:
            return

        inertia_elem = _first_child(inertial, "inertia")
        if inertia_elem is None:
            return
        origin_xyz, origin_rpy = self._origin(inertial)
        attrs = {
            "mass": _format_float(mass),
            "pos": _format_vec(origin_xyz),
            "fullinertia": _format_vec(
                (
                    float(inertia_elem.get("ixx", "0")),
                    float(inertia_elem.get("iyy", "0")),
                    float(inertia_elem.get("izz", "0")),
                    float(inertia_elem.get("ixy", "0")),
                    float(inertia_elem.get("ixz", "0")),
                    float(inertia_elem.get("iyz", "0")),
                )
            ),
        }
        if any(abs(value) > 1.0e-12 for value in origin_rpy):
            attrs["euler"] = _format_vec(origin_rpy)
        ET.SubElement(body, "inertial", attrs)

    def _add_geometries(self, body, link, kind):
        for index, geom_parent in enumerate(_children(link, kind)):
            geometry = _first_child(geom_parent, "geometry")
            if geometry is None:
                continue
            geom_attrs = {
                "name": f"{link.get('name')}_{kind}_{index}",
                "class": kind,
            }
            origin_xyz, origin_rpy = self._origin(geom_parent)
            if any(abs(value) > 1.0e-12 for value in origin_xyz):
                geom_attrs["pos"] = _format_vec(origin_xyz)
            if any(abs(value) > 1.0e-12 for value in origin_rpy):
                geom_attrs["euler"] = _format_vec(origin_rpy)

            if not self._set_geometry_attrs(geom_attrs, geometry):
                continue
            if kind == "visual":
                geom_attrs.setdefault("rgba", self._visual_rgba(link.get("name", "")))
            ET.SubElement(body, "geom", geom_attrs)

    def _set_geometry_attrs(self, geom_attrs, geometry):
        mesh = _first_child(geometry, "mesh")
        if mesh is not None:
            filename = mesh.get("filename")
            if not filename:
                return False
            scale = _parse_vec(mesh.get("scale"), (1.0, 1.0, 1.0))
            mesh_name = self._register_mesh(filename, scale)
            if not mesh_name:
                return False
            geom_attrs["type"] = "mesh"
            geom_attrs["mesh"] = mesh_name
            return True

        box = _first_child(geometry, "box")
        if box is not None:
            size = _parse_vec(box.get("size"), (0.0, 0.0, 0.0))
            geom_attrs["type"] = "box"
            geom_attrs["size"] = _format_vec([value * 0.5 for value in size])
            return True

        cylinder = _first_child(geometry, "cylinder")
        if cylinder is not None:
            radius = float(cylinder.get("radius", "0"))
            length = float(cylinder.get("length", "0"))
            geom_attrs["type"] = "cylinder"
            geom_attrs["size"] = _format_vec((radius, length * 0.5))
            return True

        sphere = _first_child(geometry, "sphere")
        if sphere is not None:
            geom_attrs["type"] = "sphere"
            geom_attrs["size"] = _format_float(float(sphere.get("radius", "0")))
            return True

        return False

    def _register_mesh(self, uri, scale):
        source_path = self._resolve_mesh_path(uri)
        key = (str(source_path), tuple(scale))
        if key in self.mesh_assets:
            return self.mesh_assets[key]

        digest = hashlib.sha1(f"{source_path}:{scale}".encode()).hexdigest()[:10]
        mesh_name = f"{_safe_name(source_path.stem)}_{digest}"
        suffix = source_path.suffix.lower()

        if suffix == ".dae":
            dest_name = f"{mesh_name}.obj"
            self._convert_collada_to_obj(source_path, self.assets_dir / dest_name)
        elif suffix in (".obj", ".stl"):
            dest_name = f"{mesh_name}{suffix}"
            shutil.copy2(source_path, self.assets_dir / dest_name)
        else:
            self.logger.warn(f"Unsupported mesh format for MuJoCo conversion: {source_path}")
            return None

        attrs = {"name": mesh_name, "file": dest_name}
        if any(abs(value - 1.0) > 1.0e-12 for value in scale):
            attrs["scale"] = _format_vec(scale)
        ET.SubElement(self.asset, "mesh", attrs)
        self.mesh_assets[key] = mesh_name
        return mesh_name

    def _resolve_mesh_path(self, uri):
        if uri.startswith("package://"):
            package_path = uri[len("package://") :]
            package_name, relative_path = package_path.split("/", 1)
            from ament_index_python.packages import get_package_share_directory

            path = Path(get_package_share_directory(package_name)) / relative_path
        elif uri.startswith("file://"):
            path = Path(uri[len("file://") :])
        else:
            path = Path(uri)
            if not path.is_absolute() and self.output_path is not None:
                path = (Path.cwd() / path).resolve()

        if not path.is_file():
            raise FileNotFoundError(f"Mesh file does not exist: {uri} -> {path}")
        return path

    def _convert_collada_to_obj(self, source_path, dest_path):
        root = ET.parse(source_path).getroot()
        unit = 1.0
        unit_elems = _descendants(root, "unit")
        if unit_elems:
            unit = float(unit_elems[0].get("meter", "1.0"))

        geometry_meshes = {}
        for geometry in _descendants(root, "geometry"):
            mesh = _first_child(geometry, "mesh")
            geometry_id = geometry.get("id")
            if mesh is not None and geometry_id:
                geometry_meshes[geometry_id] = mesh

        instances = self._collada_geometry_instances(root)
        vertices = []
        faces = []
        if instances:
            for geometry_id, transform in instances:
                mesh = geometry_meshes.get(geometry_id)
                if mesh is not None:
                    self._append_collada_mesh(mesh, unit, transform, vertices, faces)
        else:
            for mesh in geometry_meshes.values():
                self._append_collada_mesh(mesh, unit, self._identity_matrix(), vertices, faces)

        if not vertices or not faces:
            raise RuntimeError(f"Collada mesh contains no supported triangles/polylist: {source_path}")

        with open(dest_path, "w", encoding="utf-8") as obj:
            obj.write(f"# Generated from {source_path.name}\n")
            for vertex in vertices:
                obj.write(f"v {_format_vec(vertex)}\n")
            for face in faces:
                obj.write(f"f {face[0]} {face[1]} {face[2]}\n")

    def _collada_geometry_instances(self, root):
        instances = []
        for visual_scene in _descendants(root, "visual_scene"):
            self._collect_collada_instances(visual_scene, self._identity_matrix(), instances)
        return instances

    def _collect_collada_instances(self, elem, parent_transform, instances):
        for child in elem:
            if _xml_name(child.tag) != "node":
                continue
            transform = parent_transform
            matrix_elem = _first_child(child, "matrix")
            if matrix_elem is not None and matrix_elem.text:
                transform = self._matrix_multiply(parent_transform, self._matrix_from_text(matrix_elem.text))
            for instance in _children(child, "instance_geometry"):
                geometry_id = instance.get("url", "").lstrip("#")
                if geometry_id:
                    instances.append((geometry_id, transform))
            self._collect_collada_instances(child, transform, instances)

    def _append_collada_mesh(self, mesh, unit, transform, vertices, faces):
        sources = {}
        for source in _children(mesh, "source"):
            source_id = source.get("id")
            float_array = _first_child(source, "float_array")
            if not source_id or float_array is None or not float_array.text:
                continue
            accessor = _descendants(source, "accessor")
            stride = int(accessor[0].get("stride", "3")) if accessor else 3
            sources[source_id] = ([float(value) for value in float_array.text.split()], stride)

        vertex_sources = {}
        for vertices_elem in _children(mesh, "vertices"):
            vertices_id = vertices_elem.get("id")
            if not vertices_id:
                continue
            for input_elem in _children(vertices_elem, "input"):
                if input_elem.get("semantic") == "POSITION":
                    vertex_sources[vertices_id] = input_elem.get("source", "").lstrip("#")

        for primitive in mesh:
            primitive_type = _xml_name(primitive.tag)
            if primitive_type == "triangles":
                self._append_collada_triangles(primitive, sources, vertex_sources, unit, transform, vertices, faces)
            elif primitive_type == "polylist":
                self._append_collada_polylist(primitive, sources, vertex_sources, unit, transform, vertices, faces)
            elif primitive_type == "polygons":
                self._append_collada_polygons(primitive, sources, vertex_sources, unit, transform, vertices, faces)

    def _collada_position_input(self, primitive, vertex_sources):
        inputs = _children(primitive, "input")
        for input_elem in inputs:
            semantic = input_elem.get("semantic")
            if semantic not in ("VERTEX", "POSITION"):
                continue
            source_id = input_elem.get("source", "").lstrip("#")
            if semantic == "VERTEX":
                source_id = vertex_sources.get(source_id)
            if source_id:
                tuple_stride = max(int(item.get("offset", "0")) for item in inputs) + 1
                offset = int(input_elem.get("offset", "0"))
                return source_id, offset, tuple_stride
        return None, 0, 0

    def _append_collada_triangles(self, primitive, sources, vertex_sources, unit, transform, vertices, faces):
        source_id, offset, tuple_stride = self._collada_position_input(primitive, vertex_sources)
        if source_id not in sources or tuple_stride <= 0:
            return
        p_elem = _first_child(primitive, "p")
        if p_elem is None or not p_elem.text:
            return
        indices = [int(value) for value in p_elem.text.split()]
        for cursor in range(0, len(indices), tuple_stride * 3):
            if cursor + tuple_stride * 3 > len(indices):
                break
            face = []
            for corner in range(3):
                vertex_index = indices[cursor + corner * tuple_stride + offset]
                face.append(self._append_obj_vertex(sources[source_id], vertex_index, unit, transform, vertices))
            faces.append(tuple(face))

    def _append_collada_polylist(self, primitive, sources, vertex_sources, unit, transform, vertices, faces):
        source_id, offset, tuple_stride = self._collada_position_input(primitive, vertex_sources)
        if source_id not in sources or tuple_stride <= 0:
            return
        p_elem = _first_child(primitive, "p")
        vcount_elem = _first_child(primitive, "vcount")
        if p_elem is None or vcount_elem is None or not p_elem.text or not vcount_elem.text:
            return
        indices = [int(value) for value in p_elem.text.split()]
        vertex_counts = [int(value) for value in vcount_elem.text.split()]
        cursor = 0
        for vertex_count in vertex_counts:
            polygon = []
            for corner in range(vertex_count):
                vertex_index = indices[cursor + corner * tuple_stride + offset]
                polygon.append(self._append_obj_vertex(sources[source_id], vertex_index, unit, transform, vertices))
            cursor += vertex_count * tuple_stride
            self._triangulate_polygon(polygon, faces)

    def _append_collada_polygons(self, primitive, sources, vertex_sources, unit, transform, vertices, faces):
        source_id, offset, tuple_stride = self._collada_position_input(primitive, vertex_sources)
        if source_id not in sources or tuple_stride <= 0:
            return
        for p_elem in _children(primitive, "p"):
            if not p_elem.text:
                continue
            indices = [int(value) for value in p_elem.text.split()]
            polygon = []
            for cursor in range(0, len(indices), tuple_stride):
                vertex_index = indices[cursor + offset]
                polygon.append(self._append_obj_vertex(sources[source_id], vertex_index, unit, transform, vertices))
            self._triangulate_polygon(polygon, faces)

    def _append_obj_vertex(self, source, vertex_index, unit, transform, vertices):
        values, stride = source
        base = vertex_index * stride
        if base + 2 >= len(values):
            raise RuntimeError(f"Collada vertex index out of range: {vertex_index}")
        point = self._transform_point(transform, (values[base], values[base + 1], values[base + 2]))
        vertices.append((point[0] * unit, point[1] * unit, point[2] * unit))
        return len(vertices)

    def _identity_matrix(self):
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _matrix_from_text(self, text):
        values = [float(value) for value in text.split()]
        if len(values) != 16:
            raise RuntimeError("Collada transform matrix must contain 16 values")
        return [values[index : index + 4] for index in range(0, 16, 4)]

    def _matrix_multiply(self, lhs, rhs):
        return [
            [sum(lhs[row][k] * rhs[k][col] for k in range(4)) for col in range(4)]
            for row in range(4)
        ]

    def _transform_point(self, matrix, point):
        x, y, z = point
        return (
            matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3],
            matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3],
            matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3],
        )

    def _triangulate_polygon(self, polygon, faces):
        if len(polygon) < 3:
            return
        for index in range(1, len(polygon) - 1):
            faces.append((polygon[0], polygon[index], polygon[index + 1]))

    def _rotor_actuator_info(self, joint):
        joint_name = joint.get("name", "")
        axis_elem = _first_child(joint, "axis")
        axis = _parse_vec(axis_elem.get("xyz") if axis_elem is not None else None, (0.0, 0.0, 1.0))
        limit = _first_child(joint, "limit")
        max_force = float(limit.get("upper", "8.0")) if limit is not None else 8.0
        yaw_gear = axis[2] * self.m_f_rate
        return joint_name, max_force, yaw_gear

    def _is_rotor_joint(self, joint):
        return joint.get("name", "").startswith("rotor")

    def _origin(self, elem):
        origin = _first_child(elem, "origin")
        if origin is None:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        return _parse_vec(origin.get("xyz"), (0.0, 0.0, 0.0)), _parse_vec(origin.get("rpy"), (0.0, 0.0, 0.0))

    def _visual_rgba(self, link_name):
        lowered = link_name.lower()
        if "battery" in lowered:
            return "0.04 0.04 0.04 1"
        if "thrust" in lowered or "rotor" in lowered or "prop" in lowered:
            return "0.05 0.05 0.05 0.75"
        return "0.55 0.60 0.62 1"

    def _read_float_element(self, root, name, default):
        elem = _first_child(root, name)
        if elem is None:
            return default
        return float(elem.get("value", str(default)))

    def _has_site(self, body, site_name):
        return any(elem.get("name") == site_name for elem in _descendants(body, "site"))


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")

        self.declare_parameter("model_path", "")
        self.declare_parameter("urdf_xacro_path", "")
        self.declare_parameter("xacro_options", "")
        self.declare_parameter("generated_model_dir", "/tmp/aerial_robot_mujoco")
        self.declare_parameter("headless", True)
        self.declare_parameter("viewer_font_scale", 100)
        self.declare_parameter("step_rate", 1000.0)
        self.declare_parameter("render_rate", 60.0)
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_z", 0.5)
        self.declare_parameter("rotor_joints", ["rotor1", "rotor2", "rotor3", "rotor4"])

        self.declare_parameter("ground_truth_pub_rate", 0.01)
        self.declare_parameter("ground_truth_pos_noise", 0.0)
        self.declare_parameter("ground_truth_vel_noise", 0.0)
        self.declare_parameter("ground_truth_rot_noise", 0.0)
        self.declare_parameter("ground_truth_angular_noise", 0.0)
        self.declare_parameter("mocap_pub_rate", 0.01)
        self.declare_parameter("mocap_pos_noise", 0.001)
        self.declare_parameter("mocap_rot_noise", 0.001)

        self.model_path_arg = str(self.get_parameter("model_path").value).strip()
        self.urdf_xacro_path = str(self.get_parameter("urdf_xacro_path").value).strip()
        self.xacro_options = str(self.get_parameter("xacro_options").value).strip()
        self.generated_model_dir = Path(os.path.expanduser(str(self.get_parameter("generated_model_dir").value)))
        self.model_path = None
        self.headless = bool(self.get_parameter("headless").value)
        self.viewer_font_scale = int(self.get_parameter("viewer_font_scale").value)
        self.step_rate = max(1.0, float(self.get_parameter("step_rate").value))
        self.render_period = 1.0 / max(1.0, float(self.get_parameter("render_rate").value))
        self.rotor_joints = list(self.get_parameter("rotor_joints").value)

        self.ground_truth_period = max(0.0, float(self.get_parameter("ground_truth_pub_rate").value))
        self.ground_truth_pos_noise = float(self.get_parameter("ground_truth_pos_noise").value)
        self.ground_truth_vel_noise = float(self.get_parameter("ground_truth_vel_noise").value)
        self.ground_truth_rot_noise = float(self.get_parameter("ground_truth_rot_noise").value)
        self.ground_truth_angular_noise = float(self.get_parameter("ground_truth_angular_noise").value)
        self.mocap_period = max(0.0, float(self.get_parameter("mocap_pub_rate").value))
        self.mocap_pos_noise = float(self.get_parameter("mocap_pos_noise").value)
        self.mocap_rot_noise = float(self.get_parameter("mocap_rot_noise").value)

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.imu_pub = self.create_publisher(Imu, "mujoco/imu", 10)
        self.mag_pub = self.create_publisher(MagneticField, "mujoco/mag", 10)
        self.ground_truth_pub = self.create_publisher(Odometry, "ground_truth", 10)
        self.mocap_pub = self.create_publisher(PoseStamped, "mocap/pose", 10)
        self.create_subscription(JointState, "mujoco/rotor_forces", self._rotor_force_callback, 10)

        self.rotor_force = {name: 0.0 for name in self.rotor_joints}
        self.last_ground_truth_pub_time = -1.0
        self.last_mocap_pub_time = -1.0

    def _rotor_force_callback(self, msg):
        if msg.name and msg.effort:
            for name, effort in zip(msg.name, msg.effort):
                if name in self.rotor_force:
                    self.rotor_force[name] = max(0.0, float(effort))
            return

        for name, effort in zip(self.rotor_joints, msg.effort):
            self.rotor_force[name] = max(0.0, float(effort))

    def run(self):
        try:
            import mujoco
            import numpy as np
        except ImportError as exc:
            self.get_logger().fatal(
                "Python package 'mujoco' is required for Mujoco simulation. "
                "Install the optional rosdep key 'python3-mujoco' from "
                "src/aerial_robot_base/dependencies/rosdep/mujoco.yaml."
            )
            raise SystemExit(1) from exc

        self.model_path = self._resolve_model_path()
        if not self.model_path.is_file():
            self.get_logger().fatal(f"MuJoCo model file does not exist: {self.model_path}")
            raise SystemExit(1)

        model = mujoco.MjModel.from_xml_path(str(self.model_path))
        data = mujoco.MjData(model)
        self._set_initial_pose(mujoco, model, data)

        actuator_ids = self._collect_actuators(mujoco, model)
        sensor_ids = self._collect_sensors(mujoco, model)
        fc_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "fc")
        main_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "main_body")
        if fc_site_id < 0 or main_body_id < 0:
            self.get_logger().fatal("MuJoCo model must define site 'fc' and body 'main_body'")
            raise SystemExit(1)

        viewer = None
        previous_sigint_handler = None
        if not self.headless:
            try:
                import mujoco.viewer

                viewer = mujoco.viewer.launch_passive(model, data)
                self._configure_viewer_font_scale(viewer, model, data)
                previous_sigint_handler = signal.getsignal(signal.SIGINT)
                signal.signal(signal.SIGINT, self._exit_gui_process)
            except Exception as exc:  # noqa: BLE001 - report viewer setup failures without hiding headless use.
                self.get_logger().error(f"Failed to start MuJoCo viewer: {exc}")
                raise

        mujoco.mj_forward(model, data)
        self.get_logger().info(
            f"[mujoco] started {self.model_path} with {len(actuator_ids)} rotor actuators at {self.step_rate:.1f} Hz"
        )

        step_period = 1.0 / self.step_rate
        last_render = time.monotonic()

        try:
            while rclpy.ok() and (viewer is None or viewer.is_running()):
                loop_start = time.monotonic()
                try:
                    rclpy.spin_once(self, timeout_sec=0.0)
                except (KeyboardInterrupt, RuntimeError):
                    if not rclpy.ok():
                        break
                    raise

                self._apply_rotor_forces(data, actuator_ids)
                mujoco.mj_step(model, data)
                self._publish_clock(data.time)
                self._publish_sensors(mujoco, np, model, data, sensor_ids, fc_site_id, main_body_id)

                if viewer is not None and time.monotonic() - last_render >= self.render_period:
                    viewer.sync()
                    last_render = time.monotonic()

                elapsed = time.monotonic() - loop_start
                if elapsed < step_period:
                    time.sleep(step_period - elapsed)
        finally:
            if previous_sigint_handler is not None:
                signal.signal(signal.SIGINT, previous_sigint_handler)
            if viewer is not None:
                try:
                    viewer.close()
                except KeyboardInterrupt:
                    pass

    def _configure_viewer_font_scale(self, viewer, model, data):
        if self.viewer_font_scale <= 0:
            return
        if self.viewer_font_scale not in _MUJOCO_VIEWER_FONT_SCALES:
            self.get_logger().warn(
                "viewer_font_scale must be one of "
                f"{_MUJOCO_VIEWER_FONT_SCALES}; keeping MuJoCo default"
            )
            return

        sim = viewer._get_sim()  # MuJoCo does not expose viewer font scale in its public Python API.
        if sim is None:
            self.get_logger().warn("Could not access MuJoCo viewer state to set font scale")
            return

        font_index = _MUJOCO_VIEWER_FONT_SCALES.index(self.viewer_font_scale)
        if not self._write_viewer_font_index(sim, font_index):
            self.get_logger().warn("Could not set MuJoCo viewer font scale; keeping MuJoCo default")
            return

        # Reloading the same model lets MuJoCo recreate mjrContext with the updated font index.
        sim.load(model, data, str(self.model_path))
        self.get_logger().info(f"[mujoco] viewer font scale set to {self.viewer_font_scale}%")

    def _write_viewer_font_index(self, sim, font_index):
        import ctypes

        readable_ranges = self._readable_memory_ranges()
        if not readable_ranges:
            return False

        pointer_size = ctypes.sizeof(ctypes.c_void_p)
        object_size = sys.getsizeof(sim)
        pointer_count = object_size // pointer_size
        if pointer_count < 3:
            return False

        sentinel_ui0 = 0x13572468
        sentinel_ui1 = 0x24681357
        original_ui0 = int(sim.ui0_enable)
        original_ui1 = int(sim.ui1_enable)

        try:
            sim.ui0_enable = sentinel_ui0
            sim.ui1_enable = sentinel_ui1
            pyobject_ptrs = (ctypes.c_void_p * pointer_count).from_address(id(sim))

            for pointer_index in range(2, pointer_count):
                wrapper_ptr = pyobject_ptrs[pointer_index]
                if not self._is_readable_address(wrapper_ptr, pointer_size, readable_ranges):
                    continue

                simulate_ptr = ctypes.c_void_p.from_address(wrapper_ptr).value
                scan_bytes = 16 * 1024
                if not self._is_readable_address(simulate_ptr, scan_bytes, readable_ranges):
                    continue

                values = (ctypes.c_int * (scan_bytes // ctypes.sizeof(ctypes.c_int))).from_address(simulate_ptr)
                for offset_index in range(len(values) - 1):
                    if values[offset_index] != sentinel_ui0 or values[offset_index + 1] != sentinel_ui1:
                        continue

                    font_offset_index = offset_index - 1
                    if font_offset_index < 0 or values[font_offset_index] not in range(len(_MUJOCO_VIEWER_FONT_SCALES)):
                        continue
                    values[font_offset_index] = font_index
                    return True
        finally:
            sim.ui0_enable = original_ui0
            sim.ui1_enable = original_ui1

        return False

    def _readable_memory_ranges(self):
        ranges = []
        try:
            with open("/proc/self/maps", "r", encoding="utf-8") as maps_file:
                for line in maps_file:
                    columns = line.split()
                    if len(columns) < 2 or "r" not in columns[1]:
                        continue
                    start, end = columns[0].split("-", 1)
                    ranges.append((int(start, 16), int(end, 16)))
        except OSError:
            return []
        return ranges

    def _is_readable_address(self, address, size, readable_ranges):
        if not address or address < 0x10000:
            return False
        end_address = address + size
        return any(start <= address and end_address <= end for start, end in readable_ranges)

    def _set_initial_pose(self, mujoco, model, data):
        root_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        if root_joint_id < 0:
            return

        qpos_addr = model.jnt_qposadr[root_joint_id]
        data.qpos[qpos_addr + 0] = float(self.get_parameter("spawn_x").value)
        data.qpos[qpos_addr + 1] = float(self.get_parameter("spawn_y").value)
        data.qpos[qpos_addr + 2] = float(self.get_parameter("spawn_z").value)
        mujoco.mj_forward(model, data)

    def _resolve_model_path(self):
        if self.model_path_arg:
            candidate = Path(os.path.expanduser(self.model_path_arg))
            if candidate.is_file():
                return candidate
            self.get_logger().warn(
                f"MuJoCo model file is missing, generating one from URDF/Xacro instead: {candidate}"
            )

        if not self.urdf_xacro_path:
            self.get_logger().fatal(
                "MuJoCo model path was empty or missing and no urdf_xacro_path parameter was provided"
            )
            raise SystemExit(1)

        xacro_path = Path(os.path.expanduser(self.urdf_xacro_path))
        if not xacro_path.is_file():
            self.get_logger().fatal(f"URDF/Xacro file does not exist: {xacro_path}")
            raise SystemExit(1)

        xacro_executable = shutil.which("xacro")
        if xacro_executable is None:
            self.get_logger().fatal("xacro executable was not found in PATH")
            raise SystemExit(1)

        command = [xacro_executable, str(xacro_path)] + shlex.split(self.xacro_options)
        try:
            completed = subprocess.run(command, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as exc:
            self.get_logger().fatal(f"xacro failed for {xacro_path}: {exc.stderr.strip()}")
            raise SystemExit(1) from exc

        urdf_xml = completed.stdout
        robot_name = "robot"
        try:
            robot_name = _safe_name(ET.fromstring(urdf_xml).get("name", "robot"))
        except ET.ParseError:
            pass
        digest = hashlib.sha1(
            f"{_MJCF_GENERATOR_VERSION}\n{xacro_path}\n{self.xacro_options}\n{urdf_xml}".encode()
        ).hexdigest()[:12]
        output_path = self.generated_model_dir / robot_name / digest / "robot.xml"
        generator = UrdfToMjcfGenerator(self.get_logger())
        try:
            generated_path = generator.generate(urdf_xml, output_path)
        except Exception as exc:  # noqa: BLE001 - report generator failures as launch-time errors.
            self.get_logger().fatal(f"Failed to generate MuJoCo model from {xacro_path}: {exc}")
            raise SystemExit(1) from exc

        self.get_logger().info(f"[mujoco] generated model from {xacro_path}: {generated_path}")
        return generated_path

    def _collect_actuators(self, mujoco, model):
        actuator_ids = {}
        for rotor_name in self.rotor_joints:
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, rotor_name)
            if actuator_id < 0:
                self.get_logger().warn(f"MuJoCo actuator '{rotor_name}' was not found")
                continue
            actuator_ids[rotor_name] = actuator_id
        return actuator_ids

    def _collect_sensors(self, mujoco, model):
        sensor_ids = {}
        for name in ("acc", "gyro", "mag", "fc_vel", "fc_angvel"):
            sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sensor_id >= 0:
                sensor_ids[name] = sensor_id
            else:
                self.get_logger().warn(f"MuJoCo sensor '{name}' was not found")
        return sensor_ids

    def _sensor_vec(self, model, data, sensor_ids, name, default):
        sensor_id = sensor_ids.get(name)
        if sensor_id is None:
            return default
        addr = model.sensor_adr[sensor_id]
        dim = model.sensor_dim[sensor_id]
        values = data.sensordata[addr : addr + dim]
        if dim < len(default):
            return tuple(default)
        return tuple(float(values[i]) for i in range(len(default)))

    def _apply_rotor_forces(self, data, actuator_ids):
        for rotor_name, actuator_id in actuator_ids.items():
            data.ctrl[actuator_id] = self.rotor_force.get(rotor_name, 0.0)

    def _publish_clock(self, sim_time):
        msg = Clock()
        msg.clock.sec = int(sim_time)
        msg.clock.nanosec = int((sim_time - msg.clock.sec) * 1.0e9)
        self.clock_pub.publish(msg)

    def _publish_sensors(self, mujoco, np, model, data, sensor_ids, fc_site_id, main_body_id):
        stamp = self.get_clock().now().to_msg()
        acc = self._sensor_vec(model, data, sensor_ids, "acc", (0.0, 0.0, 9.80665))
        gyro = self._sensor_vec(model, data, sensor_ids, "gyro", (0.0, 0.0, 0.0))
        mag = self._sensor_vec(model, data, sensor_ids, "mag", (0.0, 0.0, 0.0))
        linear_vel = self._sensor_vec(model, data, sensor_ids, "fc_vel", (0.0, 0.0, 0.0))
        angular_vel = self._sensor_vec(model, data, sensor_ids, "fc_angvel", gyro)

        fc_quat = np.zeros(4)
        mujoco.mju_mat2Quat(fc_quat, data.site_xmat[fc_site_id])

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = "fc"
        imu_msg.orientation.w = float(fc_quat[0])
        imu_msg.orientation.x = float(fc_quat[1])
        imu_msg.orientation.y = float(fc_quat[2])
        imu_msg.orientation.z = float(fc_quat[3])
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]
        self.imu_pub.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = "magnet"
        mag_msg.magnetic_field.x = mag[0]
        mag_msg.magnetic_field.y = mag[1]
        mag_msg.magnetic_field.z = mag[2]
        self.mag_pub.publish(mag_msg)

        if self.ground_truth_period == 0.0 or data.time - self.last_ground_truth_pub_time >= self.ground_truth_period:
            self._publish_ground_truth(stamp, data, fc_site_id, fc_quat, linear_vel, angular_vel)

        if self.mocap_period == 0.0 or data.time - self.last_mocap_pub_time >= self.mocap_period:
            self._publish_mocap(stamp, data, fc_site_id, fc_quat)

        _ = main_body_id

    def _publish_ground_truth(self, stamp, data, fc_site_id, fc_quat, linear_vel, angular_vel):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.child_frame_id = "fc"

        pos = data.site_xpos[fc_site_id]
        msg.pose.pose.position.x = float(pos[0] + self._noise(self.ground_truth_pos_noise))
        msg.pose.pose.position.y = float(pos[1] + self._noise(self.ground_truth_pos_noise))
        msg.pose.pose.position.z = float(pos[2] + self._noise(self.ground_truth_pos_noise))

        noisy_quat = self._apply_quat_noise(fc_quat, self.ground_truth_rot_noise)
        self._set_ros_quat(msg.pose.pose.orientation, noisy_quat)

        msg.twist.twist.linear.x = linear_vel[0] + self._noise(self.ground_truth_vel_noise)
        msg.twist.twist.linear.y = linear_vel[1] + self._noise(self.ground_truth_vel_noise)
        msg.twist.twist.linear.z = linear_vel[2] + self._noise(self.ground_truth_vel_noise)
        msg.twist.twist.angular.x = angular_vel[0] + self._noise(self.ground_truth_angular_noise)
        msg.twist.twist.angular.y = angular_vel[1] + self._noise(self.ground_truth_angular_noise)
        msg.twist.twist.angular.z = angular_vel[2] + self._noise(self.ground_truth_angular_noise)

        self.ground_truth_pub.publish(msg)
        self.last_ground_truth_pub_time = float(data.time)

    def _publish_mocap(self, stamp, data, fc_site_id, fc_quat):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"

        pos = data.site_xpos[fc_site_id]
        msg.pose.position.x = float(pos[0] + self._noise(self.mocap_pos_noise))
        msg.pose.position.y = float(pos[1] + self._noise(self.mocap_pos_noise))
        msg.pose.position.z = float(pos[2] + self._noise(self.mocap_pos_noise))
        self._set_ros_quat(msg.pose.orientation, self._apply_quat_noise(fc_quat, self.mocap_rot_noise))

        self.mocap_pub.publish(msg)
        self.last_mocap_pub_time = float(data.time)

    def _noise(self, sigma):
        if sigma <= 0.0:
            return 0.0
        return random.gauss(0.0, sigma)

    def _apply_quat_noise(self, quat_wxyz, sigma):
        if sigma <= 0.0:
            return quat_wxyz
        dq = self._rpy_to_quat(self._noise(sigma), self._noise(sigma), self._noise(sigma))
        return self._quat_multiply(quat_wxyz, dq)

    @staticmethod
    def _rpy_to_quat(roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )

    @staticmethod
    def _quat_multiply(lhs, rhs):
        lw, lx, ly, lz = lhs
        rw, rx, ry, rz = rhs
        return (
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        )

    @staticmethod
    def _set_ros_quat(dst, quat_wxyz):
        dst.w = float(quat_wxyz[0])
        dst.x = float(quat_wxyz[1])
        dst.y = float(quat_wxyz[2])
        dst.z = float(quat_wxyz[3])

    @staticmethod
    def _exit_gui_process(signum, frame):
        _ = signum
        _ = frame
        os._exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridge()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main(sys.argv)
