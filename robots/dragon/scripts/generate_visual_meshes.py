#!/usr/bin/env python3
"""Offline visual-only LOD generation; runtime does not need these Python packages.

Tested with trimesh 5.1.0, fast-simplification 0.2.0 and numpy 1.26.4.
Original CAD meshes remain in mesh/. Collision shapes and inertias are untouched.
"""
from pathlib import Path
import json
import numpy as np
import trimesh
import fast_simplification

root=Path(__file__).resolve().parents[1]/'urdf/v1_5/mesh'
report={}
for path in sorted(root.rglob('*.stl')):
    if 'visual' in path.relative_to(root).parts:
        continue
    original=trimesh.load_mesh(path,process=True)
    original_faces=len(original.faces)
    target=min(original_faces,2500)
    mesh=original
    while target<original_faces:
        v,f=fast_simplification.simplify(original.vertices,original.faces,
                                       target_count=target,agg=4.0)
        candidate=trimesh.Trimesh(v,f,process=False)
        bounds_error=float(np.max(np.abs(candidate.bounds-original.bounds)))
        area_error=abs(candidate.area/original.area-1)
        if bounds_error<=0.0005 and area_error<=0.015:
            mesh=candidate
            break
        target=min(original_faces,target*2)
    destination=root/'visual'/path.relative_to(root)
    destination.parent.mkdir(parents=True,exist_ok=True)
    mesh.export(destination)
    report[str(path.relative_to(root))]={
        'original_triangles':original_faces,'visual_triangles':len(mesh.faces),
        'max_bounds_error_m':float(np.max(np.abs(mesh.bounds-original.bounds))),
        'surface_area_relative_error':float(abs(mesh.area/original.area-1)),
    }
(root/'visual/metrics.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
