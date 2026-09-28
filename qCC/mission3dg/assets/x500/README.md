# PX4 X500 display model

Source: https://github.com/PX4/PX4-gazebo-models/tree/5577035667afb4b63fe1f966fb1a58bbb05d905b/models/x500_base

BSD-3-Clause; see LICENSE. The pinned revision and source SHA-256 hashes are in
provenance.json. Geometry includes the frame, landing gear, motor bases/bells
and four propellers assembled using the source SDF link/visual transforms.
The source model's world spawn height is deliberately excluded: the origin is
base_link, so incoming vehicle position remains the model's origin.

Coordinates: metres, +X forward, +Y left, +Z up. This keeps the previous marker's
+X nose convention; telemetry must supply the body-to-map orientation in this
convention. No implicit ENU/NED conversion is performed here.

Adaptations: each material group targets 10% of its original triangles (the
simplifier retains more where needed: 251,556 to 59,976 triangles overall).
Carbon-fibre texture is replaced by a dark solid material; planar
brand decals and non-triangle geometry are omitted. Source material colours
are retained with a small brightness floor. Propellers are static because the
protocol does not provide rotor RPM or armed state.

Rebuild only when changing the asset:

```
python3 -m venv /tmp/quadrotor-authoring
/tmp/quadrotor-authoring/bin/pip install numpy==2.5.1 pycollada==0.9.3 fast-simplification==0.2.0
/tmp/quadrotor-authoring/bin/python scripts/prepare-quadrotor-model.py /tmp/quadrotor-sources
```

Normal builds use the checked-in resource without Python, mesh import plugins,
texture paths, or network access. The binary is little-endian: 8-byte magic
`3DGQ0001`, uint32 vertex/triangle counts, vertex records (float32 XYZ and uint8
RGB), then uint32 triangle indices. The resource loader validates its shape.
