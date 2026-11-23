# Rendering the camera reconstruction realistically

How to make the coloured reconstruction (`colored_map`, `/reconstruction/cloud`) in the 3D view look like a surface
instead of a cloud of dots. These are the same tricks Gaussian-splat viewers use, adapted to surfels from depth
(DAP or DA3, scaled by the lidar), with no extra neural network.

Ordered by value for effort. Do them top-down.

## 0. Prerequisite: send surfels, not points

Publish `sensor_msgs/PointCloud2` with fields `x y z rgb normal_x normal_y normal_z radius` (float32, 32 B per surfel).

- **Normal:** cross product of neighbouring pixels in the depth panorama.
- **Radius:** `distance × (π / H_pano) / |cos(incidence)|`, clamped (e.g. 2–30 cm).
- **Fuse into a voxel hash** (5–10 cm) with a weighted running mean of position, normal and colour. This bounds the
  size, and repeated frames don't ghost on top of each other.
- **Drop:** sky (DAP `pred_mask`, DA3 `sky`), the drone's body, and anything near `/malt/moving`.
- **Size:** 100k surfels ≈ 3.2 MB. That's fine at the GUI's 3 s throttle over CBOR. Later, publish only changed voxels.

In `app.js`, `decodeCloud` reads `normal_*` and `radius` when present. Draw an instanced quad per surfel
(`InstancedBufferGeometry` and a `ShaderMaterial`), oriented by its normal and sized by its radius. Keep plain
`THREE.Points` as the fallback when a cloud has no normals.

## 1. Remove flying pixels (biggest win, no rendering cost)

Depth networks smear points between foreground and background at object edges ("flying pixels"). Nothing in the
renderer hides them well, so remove them before publishing:

- Drop pixels where |Δ log depth| to a neighbour is above `edge_thresh` (as `da3_fusion.py` already does for anchors).
- Drop grazing-angle samples (`|cos(incidence)| < 0.15`).
- Drop isolated voxels (seen in only one frame, or fewer than N neighbours).

## 2. Eye-Dome Lighting (EDL)

A screen-space pass over the depth buffer that darkens pixels whose neighbours are closer. Shapes and edges stand out
strongly. It needs no normals and costs one full-screen pass. Potree and CloudCompare use it by default.

- Render the splats to a render target with a depth texture.
- Full-screen shader: `shade = exp(-strength × Σ max(0, log z_neighbour − log z_centre))` over 4–8 neighbours at 1–2 px.

## 3. Get colour space right

Camera RGB is sRGB. In three.js, mark vertex colours as sRGB, or convert them to linear when decoding, and set the
renderer's output colour space to sRGB. Otherwise colours look washed out or too dark. Small change, very visible.

## 4. Gaussian splats with soft blending (the "splat look")

Each surfel becomes an oriented ellipse with a Gaussian alpha falloff instead of a hard disc. Classic EWA surface
splatting needs no sorting:

1. **Depth pass:** draw splats opaque, pushed back by a small ε (about 1–2 % of depth).
2. **Accumulation pass:** additive blending of `(w · rgb, w)` with `w = exp(−r² / 2σ²)`, depth-tested against pass 1
   without writing depth.
3. **Normalise:** a full-screen pass computes `rgb / w`.

The result is smooth, hole-free surfaces, with overlapping splats blended instead of z-fighting.
(Zwicker et al. 2001; Botsch et al. 2005, "High-quality surface splatting on today's GPUs".)

## 5. Lighting and depth cues

Surfel colours already contain the scene's lighting (baked in), so don't fully relight. Add cues instead:

- **SSAO** (screen-space ambient occlusion): contact shadows in corners and under objects.
- **Light normal shading:** `colour × (0.75 + 0.25 · max(0, n·l))` with a hemisphere light gives shape without
  fighting the baked shading.
- True relighting would need the baked lighting removed from the colours first. Not worth it for an operations GUI.

## 6. Colour consistency across frames

- **Exposure compensation per frame:** scale each frame's colours so its overlap with the map matches in median.
- **Weight by lens angle:** weight colour samples by `cos(angle from the lens axis)`, so fisheye edges (vignetting,
  blur) lose to centre views.
- **Robust fusion:** a per-voxel median, or a running mean with outlier rejection, handles moving shadows and people.

## 7. View-dependent colour (optional)

3D Gaussian Splatting stores spherical harmonics (SH) per Gaussian, so colour changes with viewing direction
(sheen, specular highlights). For surfels, fit degree-1 SH (4 coefficients per channel) per voxel from its
observations. Only worth it for shiny materials. Mostly overkill here.

## 8. Scale and anti-aliasing

- **Minimum splat size:** clamp to at least 1–1.5 px on screen, so far splats don't sparkle (the idea behind
  Mip-Splatting's filter).
- **Level of detail:** an octree, as in Potree, once the map passes a few million surfels.
- **Hole filling:** a screen-space pull-push pass for any remaining gaps.

## 9. Background

Draw the sky from the latest panorama as an environment sphere behind everything. Don't turn sky pixels into points.

## Suggested order

Surfels with normals (0) → flying-pixel cleanup (1) → EDL (2) → sRGB (3) → Gaussian blending (4) → SSAO (5).
That gives most of the Gaussian-splat look while depth stays DAP + lidar, with no splat network to train.
