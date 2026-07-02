# Cloth

A real-time cloth simulation written in C#/.NET using raw Win32, WGL, and OpenGL (core profile) — no NuGet packages, no engine, no OpenGL wrapper, no external assets.

The program simulates a sheet of cloth as a grid of point masses connected by distance constraints. It is integrated with Verlet integration and relaxed with an **XPBD** (Extended Position-Based Dynamics) solver, so stiffness is a material property rather than a side effect of the iteration count — which is demonstrated live: **material presets** (cotton, silk, canvas, rubber) are switched at runtime by swapping three compliance numbers, with no constraint rebuild and no re-tuning. The sheet collides with itself, with a sphere, and with the ground, can tear, is pushed by a divergence-free **curl-noise wind** with optional **aerodynamic (normal-pressure) coupling** for realistic flag flutter, and is rebuilt into a lit, two-sided, UV-textured mesh every frame. A key press saves a **PNG screenshot** through a dependency-free encoder.

It ships with two simulation backends, switchable at runtime:

- a **CPU path** (`64 x 64`) with the full feature set — self-collision, tearing, sphere draping, interactive grabbing;
- a **GPU compute path** (`320 x 320 = 102,400` particles, ~611k constraints) that runs integration, a 12-color graph-colored XPBD solve (structural + shear + bend), aerodynamic wind, and floor collision entirely in compute shaders and renders straight from the GPU buffer.

## Screenshot
![Screenshot](screenshot.jpg)

## Features

### Simulation
- Verlet integration with damping.
- **XPBD solver** with per-constraint compliance (inverse stiffness): structural (rigid), shear, and bend constraints each get their own stiffness, independent of how many solver iterations run.
- Structural, shear, and bend distance constraints on a `64 x 64` grid.
- **Particle self-collision** via a uniform spatial hash, so folds don't pass through the sheet.
- **Sphere collision and draping** with tangential friction, so the cloth grips and folds over the obstacle instead of sliding off.
- **Ground-plane collision** with tangential friction: a released sheet falls, lands, and piles on the floor instead of dropping forever. A soft projected contact shadow grounds it visually.
- **Tearing** (off by default): when enabled, constraints snap past a stretch threshold and the render mesh rebuilds to open real holes (torn quads are removed, not stretched). The threshold is high enough that ordinary settling and wind never tear — only a hard mouse yank does.
- **Runtime material presets** (`M`): cotton, silk, canvas, and rubber. Each preset is just three XPBD compliance values (stretch / shear / bend); because XPBD makes stiffness independent of the iteration count, switching is instant — no constraint rebuild, no re-tuning. Rubber has non-zero stretch compliance and is visibly elastic; silk collapses into fine folds; canvas holds its shape.
- **Curl-noise wind**: the wind field is the curl of a noise vector potential, so it is divergence-free and reads as real, swirling air, with a steady breeze and slow gusts on top. Dependency-free 3D value noise.
- **Aerodynamic wind coupling** (`A`, on by default): instead of pushing particles with the field directly, the field is treated as an air velocity and each particle feels a quadratic normal-pressure force from the *relative* flow, `a = C (n·v_rel)|n·v_rel| n`. Edge-on cloth feels nothing, so a flag snaps and flutters like real fabric instead of being pushed uniformly — and since `v_rel` includes the particle's own velocity, the same term acts as air drag: a released sheet falls slower and tumbles even with the wind off.

### Interaction
- Mouse picking and dragging of cloth particles.
- Flag mode (pin one edge) and per-corner pin/unpin toggles.
- Live wind strength control, wind on/off, and aerodynamic-coupling toggle.
- Material preset cycling (cotton / silk / canvas / rubber).
- Drop-onto-sphere, drop-onto-floor, and reset.
- Orbit camera and zoom.
- **PNG screenshots** (`P`): the back buffer is read with `glReadPixels` and written by a ~150-line dependency-free PNG encoder (stored-deflate zlib stream, hand-rolled CRC-32 and Adler-32) — see `Png.cs`.
- **Title-bar HUD**: live FPS, active path, particle/constraint counts, material, and wind state, refreshed twice a second (the zero-dependency alternative to a font renderer).
- **VSync toggle** (`V`) for uncapped frame-rate measurements.

### Rendering
- Dynamic per-vertex normals recomputed from particle positions each frame.
- Two-sided shading with separate front/back colors.
- Procedural plain-weave fabric texture (with mipmapping) and UVs.
- Procedural background gradient.

### GPU compute path
- `320 x 320 = 102,400` particles, ~611k distance constraints.
- Positions, previous positions, constraints, Lagrange multipliers, and the render mesh all live in SSBOs; nothing is read back to the CPU.
- **Graph coloring**: structural + shear + bend constraints are partitioned into 12 conflict-free colors (no two constraints in a color share a particle), so each color is a single race-free compute dispatch — no atomics. The bend colors come from the same analytic pattern: `(i>>1)&1` flips every two columns, which is exactly the conflict distance of an `(i, i+2)` constraint.
- **Material presets work here too, for free**: the constraint buffer stores only endpoints and rest length; the compliance for each constraint *type* is a per-dispatch uniform, so switching materials uploads nothing.
- **Aerodynamic wind on the GPU**: the predict pass reads each particle's surface normal from the render vertex buffer written by the *previous* frame's build pass — race-free and one frame stale, which is invisible at 240 substeps per second.
- **Ground-plane collision** with tangential friction, mirroring the CPU path.
- Curl-noise wind and XPBD ported to GLSL compute. The draw call pulls vertices directly from the same buffer the build pass writes.

## Controls

| Input | Action |
| --- | --- |
| Left mouse + drag | Grab / drag the nearest particle |
| Right mouse + drag | Orbit the camera |
| Mouse wheel | Zoom in / out |
| `Space` | Drop a flat sheet onto the sphere (drape) |
| `C` | Drop a flat sheet onto the floor |
| `R` | Reset to the hanging curtain |
| `T` | Toggle tearing (off by default; yank hard to tear once on) |
| `F` | Flag mode (pin the left edge) |
| `1` `2` `3` `4` | Toggle pin on each corner (TL / TR / BL / BR) |
| `W` | Toggle wind |
| `A` | Toggle aerodynamic (normal-pressure) wind coupling |
| `M` | Cycle material preset: cotton → silk → canvas → rubber |
| `Up` / `Down` | Increase / decrease wind strength |
| `G` | Switch between the CPU and GPU (102k-particle) paths |
| `P` | Save a PNG screenshot next to the executable |
| `V` | Toggle vsync |
| Close window | Exit |

## Requirements

- Windows.
- .NET SDK.
- A GPU and driver supporting **OpenGL 4.3 or newer** (the GPU path uses compute shaders; the context is created as 4.3 core).
- Visual Studio, Rider, or the `dotnet` CLI.

The project is Windows-specific because it uses direct Win32, GDI, and WGL calls through P/Invoke.

## Build and run

From the project directory:

```bash
dotnet run --project Cloth.csproj -c Release
```

Or open the solution in a recent IDE and run the `Cloth` project in Release mode.

## Project structure

```text
Cloth.csproj   .NET project file
Cloth.slnx     Solution file
Program.cs     CPU simulation, materials, aero wind, Win32 windowing, OpenGL loading, rendering, shaders, input
GL.cs          OpenGL constant + function-pointer bindings (incl. compute / SSBO)
Win.cs         Win32 structs and P/Invoke declarations
GpuCloth.cs    GPU compute path: SSBOs, 12-color XPBD compute shaders, floor pass, GPU mesh build
Png.cs         Dependency-free PNG encoder for screenshots (stored-deflate zlib, CRC-32, Adler-32)
```

## How it works

### CPU path

The cloth is a square grid of particles; each stores its current and previous position, so velocity is implicit (Verlet). Every substep:

1. Apply gravity and the wind to all unpinned particles and integrate. With aerodynamic coupling on (default), per-particle surface normals are estimated from grid neighbors and the wind enters as a quadratic normal-pressure force from the relative flow; with it off, the field is applied directly as an acceleration (the cheaper legacy mode).
2. **XPBD solve.** Each constraint accumulates a Lagrange multiplier over the substep:
   `dLambda = (-C - alphaTilde * lambda) / (w_a + w_b + alphaTilde)`, with `alphaTilde = compliance / dt^2`. Compliance `0` collapses to ordinary rigid PBD; larger compliance yields a softer, iteration-count-independent material. Each constraint stores only its *type* (structural / shear / bend); the compliance is looked up from the active material preset at solve time, which is why cycling materials with `M` is free.
3. **Tear** any constraint stretched past `Rest * TearFactor`; if it was a structural edge, mark it so the affected quads drop out of the index buffer on the next rebuild, opening a real hole.
4. **Self-collide** (spatial hash), **sphere-collide**, and **floor-collide** (project out + tangential friction).
5. Force pinned particles back to their fixed positions.

The grid is then turned into a triangle mesh with per-vertex normals recomputed from neighbors, uploaded to a dynamic buffer, and drawn two-sided with a small GLSL lighting shader. The index buffer is only rebuilt and re-uploaded on frames where the topology changed (tearing).

### GPU path

The same XPBD, but on the GPU. Particles and constraints are uploaded once to SSBOs. Per frame, for each substep: a `predict` compute pass integrates and evaluates the curl-noise wind (and, when enabled, the aerodynamic pressure term — normals come from the previous frame's render mesh) in GLSL; a `clear` pass resets the multipliers; then for each iteration, the 12 constraint colors are dispatched in turn (a shader-storage barrier between colors), each dispatch carrying its material compliance as a uniform; a `floor` pass projects particles out of the ground plane with tangential friction. A final `build` pass writes positions + recomputed normals into a buffer that doubles as the render vertex buffer, and the draw call pulls from it directly. Self-collision, tearing, and the sphere are intentionally omitted on this path — they require atomic spatial hashing / dynamic topology and would be a separate effort.

## Important parameters

CPU path (top of `Program.cs`):

```csharp
const int N = 64;                  // grid resolution
const int SubSteps = 2;
const int Iters = 12;              // XPBD relaxation passes
const float Dt = 1f / 120f;
const float TearFactor = 4.0f;     // snap a constraint past Rest * this (tearing off by default)
const float Friction = 0.25f;      // sphere grip (0 = ice, 1 = sticky)
const float AeroCoeff = 0.14f;     // aerodynamic pressure coefficient
```

Material stiffness lives in the `Materials` table (stretch / shear / bend compliance per preset) — edit a preset or add your own; the same table drives both the CPU and the GPU path. Wind (`BaseBreeze`, `CurlStrength`, `NoiseFreq`, `ScrollSpeed`, `WindDir`) is also at the top of `Program.cs`. GPU-path settings (`GN`, `SubSteps`, `Iters`) are at the top of `GpuCloth.cs`.

Tuning directions: raise `Iters` for better convergence (no longer for stiffness — that is what compliance controls now); raise a preset's `Bend` compliance for a softer, silkier sheet; raise `CurlStrength` for more violent fluttering; raise `AeroCoeff` for harder flag snaps; lower `TearFactor` for fabric that tears more easily.

## Implementation notes

The project deliberately avoids helper frameworks. It manually registers a Win32 window class, creates the window, selects a pixel format, creates a WGL/OpenGL 4.3 core context, loads OpenGL (including compute and SSBO) entry points via `wglGetProcAddress`, compiles GLSL at runtime, manages VAO/VBO/EBO/SSBO objects, and processes Win32 input messages. It is a compact reference for talking directly to Win32 and modern OpenGL from C# without OpenTK, Silk.NET, SDL, GLFW, or an engine.

## Current limitations

- Windows-only.
- The CPU path is single-threaded.
- The GPU path omits self-collision, tearing, and sphere collision (they need atomic spatial hashing / dynamic topology).
- Mouse grabbing works on the CPU path only — GPU-path positions never leave VRAM.
- Error handling is intentionally minimal and demo-oriented.

## Possible improvements

- Port self-collision (GPU spatial hash) and tearing (compaction) to the GPU path.
- GPU-side mouse picking (a small compute reduction over the position buffer).
- Anisotropic materials (different warp/weft stiffness — trivially expressible in the preset table).
- On-screen text overlay (bitmap font) instead of the title-bar HUD.

## License

MIT

## Support

If you found this project interesting or useful, you can support my work:

[![GitHub Sponsors](https://img.shields.io/github/sponsors/makarov-mm?style=flat&logo=github)](https://github.com/sponsors/makarov-mm)
