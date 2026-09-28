#pragma once

class ccMesh;

// Loads the bundled PX4 X500, in metres, +X forward, +Y left, +Z up.
// Caller owns the mesh. Returns nullptr if the resource is invalid/unavailable.
ccMesh *createQuadrotorModel();
