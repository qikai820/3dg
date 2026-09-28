#!/usr/bin/env python3
"""Bake the pinned PX4 X500 visual meshes into the offline 3DG resource.

Authoring only: pip install numpy pycollada fast-simplification.
Usage: prepare-quadrotor-model.py /path/to/download-cache
The application and normal build do not need Python or network access.
"""
import hashlib
import json
from pathlib import Path
import struct
import sys
import urllib.request
import xml.etree.ElementTree as ET

import collada
import numpy as np
import fast_simplification

REVISION = "5577035667afb4b63fe1f966fb1a58bbb05d905b"
BASE = f"https://raw.githubusercontent.com/PX4/PX4-gazebo-models/{REVISION}/"
CACHE = Path(sys.argv[1])
OUTPUT = Path(__file__).resolve().parents[1] / "qCC/mission3dg/assets/x500"
CACHE.mkdir(parents=True, exist_ok=True)
OUTPUT.mkdir(parents=True, exist_ok=True)
sources = {}


def source(path):
    local = CACHE / Path(path).name
    if not local.exists():
        urllib.request.urlretrieve(BASE + path, local)
    sources[path] = hashlib.sha256(local.read_bytes()).hexdigest()
    return local


def pose(text):
    x, y, z, roll, pitch, yaw = map(float, (text or "0 0 0 0 0 0").split())
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    matrix = np.eye(4)
    matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                       [-sp, cp*sr, cp*cr]]
    matrix[:3, 3] = [x, y, z]
    return matrix


vertices, colors, faces = [], [], []
vertex_count = 0
original_faces = 0
parts = {}
sdf = ET.parse(source("models/x500_base/model.sdf")).getroot()
for link in sdf.findall("./model/link"):
    for visual in link.findall("visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is None:  # Skip the planar brand decals, not structural geometry.
            continue
        filename = Path(mesh.findtext("uri")).name
        if filename not in parts:
            path = source("models/x500_base/meshes/" + filename)
            chunks = []
            if path.suffix == ".dae":
                model = collada.Collada(str(path))
                assert model.assetInfo.upaxis == "Z_UP"
                for geometry in model.scene.objects("geometry"):
                    for primitive in geometry.primitives():
                        if not isinstance(primitive, collada.triangleset.BoundTriangleSet):
                            continue
                        diffuse = primitive.material.effect.diffuse
                        # Replace the carbon texture with a dark, readable solid material.
                        color = diffuse[:3] if isinstance(diffuse, tuple) else (.12, .15, .18)
                        chunks.append((primitive.vertex, primitive.vertex_index, color))
            else:
                raw = path.read_bytes()
                count = struct.unpack_from("<I", raw, 80)[0]
                records = np.frombuffer(raw, dtype=np.dtype([
                    ("normal", "<f4", (3,)), ("points", "<f4", (3, 3)), ("attr", "<u2")]),
                    count=count, offset=84)
                points = records["points"].reshape(-1, 3)
                chunks.append((points, np.arange(len(points)).reshape(-1, 3), (.18, .20, .23)))
            parts[filename] = []
            for points, indices, color in chunks:
                before = len(indices)
                # Weld duplicated STL/COLLADA vertices before decimating.
                points, remap = np.unique(points, axis=0, return_inverse=True)
                indices = remap[indices]
                target = max(80, int(before * .10))
                points, indices = fast_simplification.simplify(points, indices, target_count=target)
                parts[filename].append((points, indices, color, before))
        transform = pose(link.findtext("pose")) @ pose(visual.findtext("pose"))
        scale = np.array(list(map(float, mesh.findtext("scale", "1 1 1").split())))
        for points, indices, color, before in parts[filename]:
            xyz = (points * scale) @ transform[:3, :3].T + transform[:3, 3]
            rgb = np.clip(np.array(color) * 255, 24, 255).astype(np.uint8)
            vertices.extend(xyz)
            colors.extend([rgb] * len(xyz))
            faces.extend(indices + vertex_count)
            vertex_count += len(xyz)
            original_faces += before

xyz = np.asarray(vertices)
triangles = np.asarray(faces)
assert np.isfinite(xyz).all() and triangles.min() >= 0 and triangles.max() < len(xyz)
assert (xyz.max(0) - xyz.min(0) < 1.0).all(), "Unexpected source units"
destination = OUTPUT / "quadrotor.mesh"
with destination.open("wb") as stream:
    stream.write(b"3DGQ0001")
    stream.write(struct.pack("<II", len(vertices), len(faces)))
    for point, color in zip(vertices, colors):
        stream.write(struct.pack("<fffBBB", *point, *color))
    stream.write(triangles.astype("<u4").tobytes())
(OUTPUT / "LICENSE").write_bytes(source("LICENSE").read_bytes())
manifest = dict(repository="https://github.com/PX4/PX4-gazebo-models", revision=REVISION,
                sources_sha256=sources, original_triangles=original_faces,
                vertices=len(vertices), triangles=len(faces),
                bounds_m=[xyz.min(0).tolist(), xyz.max(0).tolist()],
                output_sha256=hashlib.sha256(destination.read_bytes()).hexdigest())
(OUTPUT / "provenance.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps(manifest, indent=2))
