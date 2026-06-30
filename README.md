# Cloth

A real-time cloth simulation written in C#/.NET using raw Win32, WGL, and OpenGL (core profile) — no NuGet packages, no engine, no OpenGL wrapper, no external assets.

The program simulates a sheet of cloth as a grid of point masses connected by distance constraints. It is integrated with Verlet integration and relaxed with an **XPBD** (Extended Position-Based Dynamics) solver, so stiffness is a material property rather than a side effect of the iteration count. The sheet collides with itself, with a sphere, and with the ground, can tear, is pushed by a divergence-free **curl-noise wind**, and is rebuilt into a lit, two-sided, UV-textured mesh every frame.

It ships with two simulation backends, switchable at runtime:

- a **CPU path** (`64 x 64`) with the full feature set — self-collision, tearing, sphere draping, interactive grabbing;
- a **GPU compute path** (`320 x 320 = 102,400` particles) that runs integration and a graph-colored XPBD solve entirely in compute shaders and renders straight from the GPU buffer.

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
- **Curl-noise wind**: the wind field is the curl of a noise vector potential, so it is divergence-free and reads as real, swirling air, with a steady breeze and slow gusts on top. Dependency-free 3D value noise.

### Interaction
- Mouse picking and dragging of cloth particles.
- Flag mode (pin one edge) and per-corner pin/unpin toggles.
- Live wind strength control and wind on/off.
- Drop-onto-sphere, drop-onto-floor, and reset.
- Orbit camera and zoom.

### Rendering
- Dynamic per-vertex normals recomputed from particle positions each frame.
- Two-sided shading with separate front/back colors.
- Procedural plain-weave fabric texture (with mipmapping) and UVs.
- Procedural background gradient.

### GPU compute path
- `320 x 320 = 102,400` particles, ~407k distance constraints.
- Positions, previous positions, constraints, Lagrange multipliers, and the render mesh all live in SSBOs; nothing is read back to the CPU.
- **Graph coloring**: structural + shear constraints are partitioned into 8 conflict-free colors (no two constraints in a color share a particle), so each color is a single race-free compute dispatch — no atomics.
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
| `Up` / `Down` | Increase / decrease wind strength |
| `G` | Switch between the CPU and GPU (102k-particle) paths |
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
Program.cs     CPU simulation, Win32 windowing, OpenGL loading, rendering, shaders, input
GL.cs          OpenGL constant + function-pointer bindings (incl. compute / SSBO)
Win.cs         Win32 structs and P/Invoke declarations
GpuCloth.cs    GPU compute path: SSBOs, graph-colored XPBD compute shaders, GPU mesh build
```

## How it works

### CPU path

The cloth is a square grid of particles; each stores its current and previous position, so velocity is implicit (Verlet). Every substep:

1. Apply gravity and the curl-noise wind to all unpinned particles and integrate.
2. **XPBD solve.** Each constraint accumulates a Lagrange multiplier over the substep:
   `dLambda = (-C - alphaTilde * lambda) / (w_a + w_b + alphaTilde)`, with `alphaTilde = compliance / dt^2`. Compliance `0` collapses to ordinary rigid PBD; larger compliance yields a softer, iteration-count-independent material. Structural is rigid, bend is soft (so folds form readily).
3. **Tear** any constraint stretched past `Rest * TearFactor`; if it was a structural edge, mark it so the affected quads drop out of the index buffer on the next rebuild, opening a real hole.
4. **Self-collide** (spatial hash), **sphere-collide**, and **floor-collide** (project out + tangential friction).
5. Force pinned particles back to their fixed positions.

The grid is then turned into a triangle mesh with per-vertex normals recomputed from neighbors, uploaded to a dynamic buffer, and drawn two-sided with a small GLSL lighting shader. The index buffer is only rebuilt and re-uploaded on frames where the topology changed (tearing).

### GPU path

The same XPBD, but on the GPU. Particles and constraints are uploaded once to SSBOs. Per frame, for each substep: a `predict` compute pass integrates and evaluates the curl-noise wind in GLSL; a `clear` pass resets the multipliers; then for each iteration, the 8 constraint colors are dispatched in turn (a shader-storage barrier between colors). A final `build` pass writes positions + recomputed normals into a buffer that doubles as the render vertex buffer, and the draw call pulls from it directly. Self-collision, tearing, and the sphere are intentionally omitted on this path — they require atomic spatial hashing / dynamic topology and would be a separate effort.

## Important parameters

CPU path (top of `Program.cs`):

```csharp
const int N = 64;                  // grid resolution
const int SubSteps = 2;
const int Iters = 10;              // XPBD relaxation passes
const float Dt = 1f / 120f;
const float TearFactor = 4.0f;     // snap a constraint past Rest * this (tearing off by default)
const float StructCompliance = 0f; // rigid stretch
const float ShearCompliance = 2e-5f;
const float BendCompliance = 6e-4f; // soft bending -> folds
const float Friction = 0.25f;      // sphere grip (0 = ice, 1 = sticky)
```

Wind (`BaseBreeze`, `CurlStrength`, `NoiseFreq`, `ScrollSpeed`, `WindDir`) is also at the top of `Program.cs`. GPU-path settings (`GN`, `SubSteps`, `Iters`, compliance) are at the top of `GpuCloth.cs`.

Tuning directions: raise `Iters` for better convergence (no longer for stiffness — that is what compliance controls now); raise `BendCompliance` for a softer, silkier sheet; raise `CurlStrength` for more violent fluttering; lower `TearFactor` for fabric that tears more easily.

## Implementation notes

The project deliberately avoids helper frameworks. It manually registers a Win32 window class, creates the window, selects a pixel format, creates a WGL/OpenGL 4.3 core context, loads OpenGL (including compute and SSBO) entry points via `wglGetProcAddress`, compiles GLSL at runtime, manages VAO/VBO/EBO/SSBO objects, and processes Win32 input messages. It is a compact reference for talking directly to Win32 and modern OpenGL from C# without OpenTK, Silk.NET, SDL, GLFW, or an engine.

## Current limitations

- Windows-only.
- The CPU path is single-threaded.
- The GPU path omits self-collision, tearing, and sphere collision.
- Error handling is intentionally minimal and demo-oriented.

## Possible improvements

- Port self-collision (GPU spatial hash) and tearing (compaction) to the GPU path.
- Aerodynamic (relative-velocity) wind coupling for sharper flag flutter.
- Bend constraints on the GPU path (same analytic coloring pattern).
- FPS / frame-time overlay and screenshot/video capture.
- UI controls for live parameter tweaking.

## License

MIT

## Support

If you found this project interesting or useful, you can support my work:

[![GitHub Sponsors](https://img.shields.io/github/sponsors/makarov-mm?style=flat&logo=github)](https://github.com/sponsors/makarov-mm)
