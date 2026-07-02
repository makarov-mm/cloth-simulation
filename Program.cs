// Cloth — an XPBD cloth simulation on raw Win32 + WGL + OpenGL 4.3 core.
// Zero external dependencies (System.Numerics is part of the BCL).
//
//   A grid of point masses is integrated with Verlet integration and held
//   together by distance constraints (structural, shear and bend), solved with
//   several Position-Based-Dynamics relaxation passes per frame. Particle
//   self-collision (via a uniform spatial hash) keeps folds from passing through
//   each other. The sheet hangs from its top corners, is pushed by wind, and is
//   rebuilt every frame into a lit, two-sided, UV-textured triangle mesh with a
//   procedurally generated woven fabric texture.
//
//   Left mouse drags the cloth, right mouse orbits the camera, wheel zooms.
//
//   dotnet run -c Release
//
// Author: Mykhailo Makarov (m.m.makarov@gmail.com), no-library style (P/Invoke only).

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.InteropServices;

namespace Cloth;

internal static class Program
{
    const int N = 64; // grid resolution (N x N particles)
    const float W = 2.0f; // cloth width / height (world units)
    const float Gravity = 9.8f;
    const float Damp = 0.99f; // Verlet velocity damping
    const int SubSteps = 2;
    const int Iters = 12; // constraint relaxation passes
    const float Dt = 1f / 120f;

    static readonly float Spacing = W / (N - 1);
    static readonly float ColRadius = 0.38f * Spacing; // self-collision particle radius (< rest spacing)

    static readonly Vector3 SphereC = new(0f, -0.15f, 0f);
    const float SphereR = 0.4f; // sized so the 2x2 sheet has a real skirt to drape
                                // (at the old 0.6 the sheet barely capped the sphere)
    const float Friction = 0.25f; // tangential velocity bled off on contact (0 = ice, 1 = sticky)
    const float ColEps = 0.01f;   // keep the sheet a hair off the surface to avoid z-fighting
    const float TearFactor = 4.0f; // a constraint snaps once stretched past Rest * TearFactor.
                                   // High enough that ordinary settling/wind never tears — only a
                                   // hard yank with the mouse does. (Tearing is also off by default.)

    // Ground plane. The sheet collides with it and drapes/piles instead of
    // falling forever; the default hanging curtain clears it with room to spare.
    const float FloorY = -1.6f;
    const float FloorEps = 0.01f;     // rest a hair above the plane (no z-fighting)
    const float FloorFriction = 0.3f; // horizontal slide bled off on contact

    // XPBD compliance (inverse stiffness, world-unit^2 / force). 0 = perfectly
    // rigid. Higher = softer. Because XPBD divides compliance by dt^2, stiffness
    // is a *material property*, independent of how many solver iterations run —
    // which is what makes runtime material presets possible: switching material
    // just swaps three compliance numbers, no constraint rebuild, no re-tuning
    // of the iteration count. Cycle with M.
    internal struct Material
    {
        public string Name;
        public float Struct;  // stretch compliance (0 = inextensible)
        public float Shear;   // in-plane shear compliance
        public float Bend;    // bending compliance (higher = silkier folds)
    }

    internal static readonly Material[] Materials =
    [
        new() { Name = "Cotton", Struct = 0f,    Shear = 2e-5f, Bend = 6e-4f }, // the original tuning
        new() { Name = "Silk",   Struct = 0f,    Shear = 6e-5f, Bend = 6e-3f }, // drapes into fine folds
        new() { Name = "Canvas", Struct = 0f,    Shear = 4e-6f, Bend = 8e-6f }, // stiff, holds its shape
        new() { Name = "Rubber", Struct = 4e-4f, Shear = 2e-4f, Bend = 2e-3f }, // visibly stretchy
    ];
    static int _matIdx;
    static Material Mat => Materials[_matIdx];

    // Aerodynamic wind coupling (toggle with A, on by default). Instead of
    // pushing every particle with the wind field directly, treat the field as an
    // air *velocity* and apply a normal-pressure force per particle:
    //     a = Aero * (n . v_rel) * |n . v_rel| * n,   v_rel = v_wind - v_particle
    // The force vanishes when the sheet turns edge-on to the flow and reverses
    // with the facing side, which is what produces the sharp, snapping flutter
    // of a real flag. Because v_rel includes the particle's own velocity, the
    // same term acts as air drag: a released sheet falls slower and tumbles even
    // with the wind switched off.
    const float AeroCoeff = 0.14f;    // pressure coefficient (per unit mass)
    const float AeroMaxRel = 30f;     // clamp |v_rel| so a mouse yank can't explode it

    // Curl-noise wind. The flow is the curl of a noise vector potential, so it is
    // divergence-free (swirls, no sources/sinks). WindDir is the steady breeze;
    // the curl term layers organic, time-evolving eddies on top.
    static readonly Vector3 WindDir = Vector3.Normalize(new Vector3(1f, 0.05f, 0.35f));
    const float NoiseFreq = 1.1f;     // spatial frequency of the eddies
    const float ScrollSpeed = 0.6f;   // how fast the field advects downwind
    const float BaseBreeze = 5.0f;    // steady push along WindDir (acceleration)
    const float CurlStrength = 7.0f;  // strength of the swirling turbulence
    const float CurlEps = 0.12f;      // finite-difference step for the curl

    static Vector3[] Pos, Prev, PinPos;
    static bool[] Pinned;

    // Constraints carry a *type*; the compliance is looked up from the active
    // material at solve time, so switching materials is free.
    const byte TStruct = 0, TShear = 1, TBend = 2;
    struct Con { public int A, B; public float Rest; public byte Type; }
    static readonly List<Con> Cons = new();
    static float[] Lambda; // per-constraint Lagrange multiplier, reset every substep

    static Vector3[] AeroN; // per-particle surface normals for the aero force

    static float[] MeshV; // pos(3) + normal(3) + uv(2) per particle
    static uint[] MeshI; // triangle indices
    static int MeshICount;
    static uint _clothEbo; // element buffer, re-uploaded when the cloth tears

    // Which grid edges are still intact. A quad is only drawn while its four
    // structural edges survive, so removing edges opens real holes in the mesh.
    static bool[] _hAlive; // horizontal edge (i,j)-(i+1,j),  i in [0,N-2], j in [0,N-1]
    static bool[] _vAlive; // vertical   edge (i,j)-(i,j+1),  i in [0,N-1], j in [0,N-2]
    static bool _topoDirty = true; // index buffer needs a rebuild/upload
    static bool _tearing = false;  // OFF by default (toggle with T). Wind/settling never tears;
                                   // when enabled, only a hard mouse yank snaps edges.
    static uint _fabricTex;
    static float[] SphV;
    static uint[] SphI;
    static int SphICount;

    static int _w = 1080, _h = 1080;
    static bool _running = true;
    static Win.WndProc _wndProcRef;

    static float _yaw = 0.5f, _pitch = 0.15f, _dist = 4.0f;
    static bool _grab, _orbit;
    static bool _drape; // false = hanging curtain (default), true = sheet dropped onto the sphere
    static bool _windOn = true;
    static float _windScale = 1f; // live wind multiplier (Up/Down arrows)
    static bool _gpu; // G: switch to the GPU high-resolution path
    static bool _aero = true;  // A: aerodynamic (normal-pressure) wind coupling
    static bool _vsync = true; // V: vsync toggle (for uncapped frame-rate tests)
    static bool _shot;         // P: capture a PNG screenshot after the next present
    static string _shotName;   // last saved screenshot (shown in the title HUD)
    static int _lastX, _lastY, _mx, _my;
    static int _grabIndex = -1;
    static bool _grabWasPinned;
    static Vector3 _grabOrigPin;
    static readonly Vector3 Target = new(0f, -0.1f, 0f);

    [STAThread]
    static void Main()
    {
        InitCloth();

        IntPtr hInstance = Win.GetModuleHandleW(IntPtr.Zero);
        const string cls = "ClothGLWindow";

        _wndProcRef = WindowProc;
        var wc = new Win.WNDCLASSEX
        {
            cbSize = (uint)Marshal.SizeOf<Win.WNDCLASSEX>(),
            style = Win.CS_OWNDC | Win.CS_HREDRAW | Win.CS_VREDRAW,
            lpfnWndProc = Marshal.GetFunctionPointerForDelegate(_wndProcRef),
            hInstance = hInstance,
            hCursor = Win.LoadCursorW(IntPtr.Zero, (IntPtr)Win.IDC_ARROW),
            lpszClassName = cls,
        };
        if (Win.RegisterClassExW(ref wc) == 0)
            throw new Exception("RegisterClassEx failed: " + Marshal.GetLastWin32Error());

        IntPtr hwnd = Win.CreateWindowExW(
            0, cls, "Cloth — XPBD — C#/OpenGL",
            Win.WS_OVERLAPPEDWINDOW | Win.WS_VISIBLE,
            Win.CW_USEDEFAULT, Win.CW_USEDEFAULT, _w, _h,
            IntPtr.Zero, IntPtr.Zero, hInstance, IntPtr.Zero);
        if (hwnd == IntPtr.Zero)
            throw new Exception("CreateWindowEx failed: " + Marshal.GetLastWin32Error());

        IntPtr hdc = Win.GetDC(hwnd);
        IntPtr ctx = CreateGLContext(hdc);
        GL.Load();
        if (GL.wglSwapIntervalEXT != null) GL.wglSwapIntervalEXT(1);

        if (Win.GetClientRect(hwnd, out var rc)) { _w = rc.right - rc.left; _h = rc.bottom - rc.top; }

        uint gradProg = BuildProgram(QuadVS, GradFS);
        uint litProg = BuildProgram(LitVS, LitFS);
        uint sphProg = BuildProgram(SphereVS, SphereFS);
        uint floorProg = BuildProgram(FloorVS, FloorFS);
        uint shadowProg = BuildProgram(ShadowVS, ShadowFS);

        int gScale = GL.glGetUniformLocation(gradProg, Ascii("uScale"));
        int uMVP = GL.glGetUniformLocation(litProg, Ascii("uMVP"));
        int uCam = GL.glGetUniformLocation(litProg, Ascii("uCam"));
        int uLight = GL.glGetUniformLocation(litProg, Ascii("uLight"));
        int uFront = GL.glGetUniformLocation(litProg, Ascii("uFront"));
        int uBack = GL.glGetUniformLocation(litProg, Ascii("uBack"));

        int sMVP = GL.glGetUniformLocation(sphProg, Ascii("uMVP"));
        int sCam = GL.glGetUniformLocation(sphProg, Ascii("uCam"));
        int sLight = GL.glGetUniformLocation(sphProg, Ascii("uLight"));
        int sColor = GL.glGetUniformLocation(sphProg, Ascii("uColor"));

        int fMVP = GL.glGetUniformLocation(floorProg, Ascii("uMVP"));
        int fCam = GL.glGetUniformLocation(floorProg, Ascii("uCam"));
        int fLight = GL.glGetUniformLocation(floorProg, Ascii("uLight"));

        int shMVP = GL.glGetUniformLocation(shadowProg, Ascii("uMVP"));
        int shColor = GL.glGetUniformLocation(shadowProg, Ascii("uColor"));
        int shAlpha = GL.glGetUniformLocation(shadowProg, Ascii("uAlpha"));

        // direction the light travels (downward), used to project the shadow
        Vector3 shadowLightDir = Vector3.Normalize(new Vector3(0.4f, 0.85f, 0.5f)) * -1f;

        uint quadVao = MakeQuadVao();
        MakeMeshVao(out uint clothVao, out uint clothVbo);
        BuildSphereMesh(48, 96);
        MakeStaticVao(SphV, SphI, out uint sphVao);

        // ground plane: one big quad (pos3 + normal3), normal pointing up
        const float fext = 8f;
        float[] floorV =
        [
            -fext, FloorY, -fext, 0f, 1f, 0f,
             fext, FloorY, -fext, 0f, 1f, 0f,
             fext, FloorY,  fext, 0f, 1f, 0f,
            -fext, FloorY,  fext, 0f, 1f, 0f,
        ];
        uint[] floorI = [0, 1, 2, 0, 2, 3];
        MakeStaticVao(floorV, floorI, out uint floorVao);

        _fabricTex = MakeFabricTexture(512);
        int uTex = GL.glGetUniformLocation(litProg, Ascii("uTex"));

        GpuCloth.Init(); // build GPU buffers + compute programs (102k-particle path)

        // Ground plane, drawn in both paths.
        void DrawFloor(float[] mvp, Vector3 eye)
        {
            GL.glUseProgram(floorProg);
            GL.glUniformMatrix4fv(fMVP, 1, 0, mvp);
            GL.glUniform3f(fCam, eye.X, eye.Y, eye.Z);
            GL.glUniform3f(fLight, 0.4f, 0.85f, 0.5f);
            GL.glBindVertexArray(floorVao);
            GL.glDrawElements(GL.GL_TRIANGLES, 6, GL.GL_UNSIGNED_INT, IntPtr.Zero);
        }

        // Soft contact shadow: the CPU cloth flattened onto the floor (a hair above
        // it to avoid z-fighting), drawn blended with depth writes off.
        void DrawShadow(float[] mvp)
        {
            if (MeshICount == 0) return;
            float[] shMvp = Mul(mvp, ShadowMatrix(FloorY + 0.004f, shadowLightDir));
            GL.glEnable(GL.GL_BLEND);
            GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA);
            GL.glDepthMask(0);
            GL.glUseProgram(shadowProg);
            GL.glUniformMatrix4fv(shMVP, 1, 0, shMvp);
            GL.glUniform3f(shColor, 0.0f, 0.0f, 0.0f);
            GL.glUniform1f(shAlpha, 0.33f);
            GL.glBindVertexArray(clothVao);
            GL.glDrawElements(GL.GL_TRIANGLES, MeshICount, GL.GL_UNSIGNED_INT, IntPtr.Zero);
            GL.glDepthMask(1);
            GL.glDisable(GL.GL_BLEND);
        }

        var clock = Stopwatch.StartNew();

        // Title-bar HUD: FPS + current mode, refreshed twice a second. A text
        // overlay would need a font path; the title bar is the zero-dependency HUD.
        int frames = 0;
        double hudLast = 0;
        bool vsyncApplied = _vsync;

        while (_running)
        {
            while (Win.PeekMessageW(out var msg, IntPtr.Zero, 0, 0, Win.PM_REMOVE))
            {
                if (msg.message == Win.WM_QUIT) { _running = false; break; }
                Win.TranslateMessage(ref msg);
                Win.DispatchMessageW(ref msg);
            }
            if (!_running) break;

            if (vsyncApplied != _vsync && GL.wglSwapIntervalEXT != null)
            {
                GL.wglSwapIntervalEXT(_vsync ? 1 : 0);
                vsyncApplied = _vsync;
            }

            float t = (float)clock.Elapsed.TotalSeconds;

            frames++;
            double now = clock.Elapsed.TotalSeconds;
            if (now - hudLast >= 0.5)
            {
                double fps = frames / (now - hudLast);
                frames = 0; hudLast = now;
                string path = _gpu
                    ? $"GPU 320\u00b2 ({GpuCloth.ParticleCount:n0} particles, {GpuCloth.ConstraintCount:n0} constraints)"
                    : $"CPU 64\u00b2 ({N * N:n0} particles, {Cons.Count:n0} constraints)";
                string shot = _shotName != null ? $" \u00b7 saved {_shotName}" : "";
                Win.SetWindowTextW(hwnd,
                    $"Cloth \u2014 XPBD \u2014 {path} \u00b7 {Mat.Name} \u00b7 wind {(_windOn ? $"{_windScale * 100f:0}%" : "off")}" +
                    $" \u00b7 aero {(_aero ? "on" : "off")} \u00b7 {fps:0} FPS{(_vsync ? " (vsync)" : "")}{shot}");
            }

            // camera
            var eye = Target + new Vector3(
                _dist * MathF.Cos(_pitch) * MathF.Sin(_yaw),
                _dist * MathF.Sin(_pitch),
                _dist * MathF.Cos(_pitch) * MathF.Cos(_yaw));
            float aspect = (float)_w / _h;
            float[] mvp = Mul(Perspective(0.85f, aspect, 0.05f, 60f), LookAt(eye, Target, Vector3.UnitY));

            if (!_gpu) HandleGrab(mvp); // GPU-path positions live in VRAM; grabbing is CPU-only

            if (!_gpu)
            {
                for (int s = 0; s < SubSteps; s++) Step(t);
                BuildClothMesh();
                if (_topoDirty) { ReuploadClothIndices(clothVao); _topoDirty = false; }
                UploadCloth(clothVbo);
            }

            GL.glViewport(0, 0, _w, _h);
            GL.glClearColor(0f, 0f, 0f, 1f);
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);

            GL.glDisable(GL.GL_DEPTH_TEST);
            GL.glUseProgram(gradProg);
            GL.glUniform2f(gScale, 1f, 1f);
            GL.glBindVertexArray(quadVao);
            GL.glDrawArrays(GL.GL_TRIANGLE_STRIP, 0, 4);

            if (_gpu)
            {
                GpuCloth.Step(t, _windScale, _windOn ? 1 : 0, _aero ? 1 : 0,
                              Mat.Struct, Mat.Shear, Mat.Bend);
                GL.glEnable(GL.GL_DEPTH_TEST);
                DrawFloor(mvp, eye);
                GL.glUseProgram(litProg);
                GL.glUniformMatrix4fv(uMVP, 1, 0, mvp);
                GL.glUniform3f(uCam, eye.X, eye.Y, eye.Z);
                GL.glUniform3f(uLight, 0.4f, 0.85f, 0.5f);
                GL.glActiveTexture(GL.GL_TEXTURE0);
                GL.glBindTexture(GL.GL_TEXTURE_2D, _fabricTex);
                GL.glUniform1i(uTex, 0);
                GL.glUniform3f(uFront, 0.85f, 0.12f, 0.22f);
                GL.glUniform3f(uBack, 0.20f, 0.10f, 0.35f);
                GpuCloth.Draw();
                CaptureIfRequested();
                Win.SwapBuffers(hdc);
                continue;
            }

            GL.glEnable(GL.GL_DEPTH_TEST);
            DrawFloor(mvp, eye);
            DrawShadow(mvp);
            GL.glUseProgram(litProg);
            GL.glUniformMatrix4fv(uMVP, 1, 0, mvp);
            GL.glUniform3f(uCam, eye.X, eye.Y, eye.Z);
            GL.glUniform3f(uLight, 0.4f, 0.85f, 0.5f);
            GL.glActiveTexture(GL.GL_TEXTURE0);
            GL.glBindTexture(GL.GL_TEXTURE_2D, _fabricTex);
            GL.glUniform1i(uTex, 0);

            // cloth (two-sided)
            GL.glUniform3f(uFront, 0.85f, 0.12f, 0.22f);
            GL.glUniform3f(uBack, 0.20f, 0.10f, 0.35f);
            GL.glBindVertexArray(clothVao);
            GL.glDrawElements(GL.GL_TRIANGLES, MeshICount, GL.GL_UNSIGNED_INT, IntPtr.Zero);

            // sphere (only when the cloth is draped over it)
            if (_drape)
            {
                GL.glUseProgram(sphProg);
                GL.glUniformMatrix4fv(sMVP, 1, 0, mvp);
                GL.glUniform3f(sCam, eye.X, eye.Y, eye.Z);
                GL.glUniform3f(sLight, 0.4f, 0.85f, 0.5f);
                GL.glUniform3f(sColor, 0.16f, 0.17f, 0.20f);
                GL.glBindVertexArray(sphVao);
                GL.glDrawElements(GL.GL_TRIANGLES, SphICount, GL.GL_UNSIGNED_INT, IntPtr.Zero);
            }

            CaptureIfRequested();
            Win.SwapBuffers(hdc);
        }

        Win.wglMakeCurrent(IntPtr.Zero, IntPtr.Zero);
        Win.wglDeleteContext(ctx);
        Win.ReleaseDC(hwnd, hdc);
    }

    static IntPtr WindowProc(IntPtr hWnd, uint msg, IntPtr wParam, IntPtr lParam)
    {
        switch (msg)
        {
            case Win.WM_SIZE:
                int lp = (int)(long)lParam;
                _w = Math.Max(1, lp & 0xFFFF);
                _h = Math.Max(1, (lp >> 16) & 0xFFFF);
                return IntPtr.Zero;

            case Win.WM_LBUTTONDOWN:
                _grab = true; _mx = Short(lParam, 0); _my = Short(lParam, 16);
                return IntPtr.Zero;

            case Win.WM_LBUTTONUP:
                _grab = false;
                return IntPtr.Zero;

            case Win.WM_RBUTTONDOWN:
                _orbit = true; _lastX = Short(lParam, 0); _lastY = Short(lParam, 16);
                return IntPtr.Zero;

            case Win.WM_RBUTTONUP:
                _orbit = false;
                return IntPtr.Zero;

            case Win.WM_MOUSEMOVE:
                {
                    int x = Short(lParam, 0), y = Short(lParam, 16);
                    _mx = x; _my = y;
                    if (_orbit)
                    {
                        _yaw -= (x - _lastX) * 0.006f;
                        _pitch += (y - _lastY) * 0.006f;
                        float lim = 1.45f;
                        if (_pitch > lim) _pitch = lim;
                        if (_pitch < -lim) _pitch = -lim;
                        _lastX = x; _lastY = y;
                    }
                }
                return IntPtr.Zero;

            case Win.WM_MOUSEWHEEL:
                int delta = (short)((((long)wParam) >> 16) & 0xFFFF);
                _dist *= 1f - delta / 1200f;
                if (_dist < 1.5f) _dist = 1.5f;
                if (_dist > 14f) _dist = 14f;
                return IntPtr.Zero;

            case Win.WM_KEYDOWN:
                {
                    int vk = (int)(long)wParam;
                    if (vk == 0x20) DropOntoSphere();        // SPACE: drop sheet onto sphere
                    else if (vk == 0x43) DropOntoFloor();      // C: drop flat sheet onto the floor
                    else if (vk == 0x52) { InitCloth(); _drape = false; } // R: reset to curtain
                    else if (vk == 0x54) _tearing = !_tearing; // T: toggle tearing
                    else if (vk == 0x46) FlagMode();           // F: pin left edge (flag)
                    else if (vk == 0x57) _windOn = !_windOn;   // W: toggle wind
                    else if (vk == 0x47) _gpu = !_gpu;          // G: CPU <-> GPU path
                    else if (vk == 0x4D) _matIdx = (_matIdx + 1) % Materials.Length; // M: cycle material
                    else if (vk == 0x41) _aero = !_aero;        // A: aerodynamic wind coupling
                    else if (vk == 0x50) _shot = true;          // P: save a PNG screenshot
                    else if (vk == 0x56) _vsync = !_vsync;      // V: vsync on/off
                    else if (vk == 0x31) ToggleCornerPin(0, 0);          // 1: top-left
                    else if (vk == 0x32) ToggleCornerPin(N - 1, 0);      // 2: top-right
                    else if (vk == 0x33) ToggleCornerPin(0, N - 1);      // 3: bottom-left
                    else if (vk == 0x34) ToggleCornerPin(N - 1, N - 1);  // 4: bottom-right
                    else if (vk == 0x26) _windScale = MathF.Min(_windScale * 1.25f, 20f); // Up: stronger
                    else if (vk == 0x28) _windScale = MathF.Max(_windScale / 1.25f, 0f);  // Down: weaker
                }
                return IntPtr.Zero;

            case Win.WM_DESTROY:
                Win.PostQuitMessage(0);
                return IntPtr.Zero;
        }

        return Win.DefWindowProcW(hWnd, msg, wParam, lParam);
    }

    private static int Short(IntPtr lParam, int shift) => (short)((((long)lParam) >> shift) & 0xFFFF);

    private static int Idx(int i, int j) => j * N + i;

    private static void InitCloth()
    {
        Cons.Clear();
        int n = N * N;
        Pos = new Vector3[n]; Prev = new Vector3[n]; PinPos = new Vector3[n]; Pinned = new bool[n];

        float top = 1.0f, frontZ = 0.7f;
        for (int j = 0; j < N; j++)
            for (int i = 0; i < N; i++)
            {
                var p = new Vector3((i / (float)(N - 1) - 0.5f) * W, top - j / (float)(N - 1) * W, frontZ);
                int k = Idx(i, j);
                p += Jitter(i, j);              // break the perfect mirror symmetry
                Pos[k] = p; Prev[k] = p;
            }

        // pin the two top corners
        Pin(0, 0); Pin(N - 1, 0);

        _hAlive = new bool[(N - 1) * N];
        _vAlive = new bool[N * (N - 1)];
        for (int e = 0; e < _hAlive.Length; e++) _hAlive[e] = true;
        for (int e = 0; e < _vAlive.Length; e++) _vAlive[e] = true;
        _topoDirty = true;

        void Add(int ai, int aj, int bi, int bj, byte type)
        {
            // Both endpoints must be in range. Checking only b let the shear '/'
            // case Add(i+1, j, i, j+1) slip through at i == N-1, where a = (N, j)
            // wraps via Idx to (0, j+1) — a near-rigid constraint spanning the
            // whole width that left the sheet stiff along X and draping only along Z.
            if (ai < 0 || ai >= N || aj < 0 || aj >= N) return;
            if (bi < 0 || bi >= N || bj < 0 || bj >= N) return;
            int a = Idx(ai, aj), b = Idx(bi, bj);
            Cons.Add(new Con { A = a, B = b, Rest = Vector3.Distance(Pos[a], Pos[b]), Type = type });
        }

        for (int j = 0; j < N; j++)
            for (int i = 0; i < N; i++)
            {
                Add(i, j, i + 1, j, TStruct);     // structural
                Add(i, j, i, j + 1, TStruct);
                Add(i, j, i + 1, j + 1, TShear);  // shear
                Add(i + 1, j, i, j + 1, TShear);
                Add(i, j, i + 2, j, TBend);       // bend
                Add(i, j, i, j + 2, TBend);
            }

        Lambda = new float[Cons.Count];
        AeroN = new Vector3[n];
    }

    private static void Pin(int i, int j)
    {
        int k = Idx(i, j);
        Pinned[k] = true; PinPos[k] = Pos[k];
    }

    // Tiny per-particle position offset that breaks the otherwise perfect mirror
    // symmetry of the centered scenes. Without it the sheet — centered square,
    // centered sphere, deterministic solver — collapses into an unnatural mirror
    // fold (exactly about the x = 0 plane) instead of buckling to one side like
    // real fabric. ~0.3 mm: invisible, but enough to seed a natural drape.
    // Hashed (deterministic), so a given scene folds the same way every run.
    private static Vector3 Jitter(int i, int j) => new(
        Hash3(i + 1, j + 7, 101) * 3e-4f,
        Hash3(i + 3, j + 11, 211) * 3e-4f,
        Hash3(i + 5, j + 13, 307) * 3e-4f);

    private static void Step(float t)
    {
        var grav = new Vector3(0f, -Gravity, 0f);

        if (_aero) ComputeAeroNormals();

        // Verlet integration with a divergence-free curl-noise wind field.
        // Two coupling modes:
        //   direct (_aero off): the field is applied as an acceleration, the
        //     original behavior — cheap, but the force ignores how the sheet
        //     faces the flow;
        //   aerodynamic (_aero on): the field is an air velocity, and the sheet
        //     feels a quadratic normal-pressure force from the *relative* flow.
        //     Edge-on cloth feels nothing, so a flag snaps and flutters instead
        //     of being pushed uniformly; the -v term doubles as air drag.
        for (int k = 0; k < Pos.Length; k++)
        {
            if (Pinned[k]) continue;
            Vector3 acc = grav;

            if (_aero)
            {
                Vector3 vrel = WindField(Pos[k], t) - (Pos[k] - Prev[k]) / Dt;
                float vr2 = vrel.LengthSquared();
                if (vr2 > AeroMaxRel * AeroMaxRel) vrel *= AeroMaxRel / MathF.Sqrt(vr2);
                float f = Vector3.Dot(AeroN[k], vrel);
                acc += AeroN[k] * (AeroCoeff * f * MathF.Abs(f));
            }
            else
            {
                acc += WindField(Pos[k], t);
            }

            Vector3 cur = Pos[k];
            Pos[k] = cur + (cur - Prev[k]) * Damp + acc * (Dt * Dt);
            Prev[k] = cur;
        }

        // XPBD constraint solve. The Lagrange multiplier of each constraint is
        // reset once per substep and accumulated across the relaxation passes.
        //   dlambda = (-C - alphaTilde*lambda) / (w_a + w_b + alphaTilde)
        // with alphaTilde = compliance / dt^2. Compliance 0 collapses this to
        // ordinary (infinitely stiff) PBD; larger compliance yields a softer,
        // iteration-count-independent material response.
        float dt2 = Dt * Dt;
        Array.Clear(Lambda, 0, Cons.Count);
        var mat = Mat; // constraint stiffness comes from the active material preset

        for (int it = 0; it < Iters; it++)
        {
            for (int c = 0; c < Cons.Count; c++)
            {
                var con = Cons[c];
                int a = con.A, b = con.B;
                float wa = Pinned[a] ? 0f : 1f;
                float wb = Pinned[b] ? 0f : 1f;
                float wsum = wa + wb;
                if (wsum == 0f) continue;

                Vector3 d = Pos[a] - Pos[b];
                float len = d.Length();
                if (len < 1e-6f) continue;
                Vector3 n = d / len;
                float C = len - con.Rest;

                float compliance = con.Type == TStruct ? mat.Struct
                                 : con.Type == TShear ? mat.Shear : mat.Bend;
                float alphaTilde = compliance / dt2;
                float dLambda = (-C - alphaTilde * Lambda[c]) / (wsum + alphaTilde);
                Lambda[c] += dLambda;

                Vector3 corr = dLambda * n;
                Pos[a] += wa * corr;
                Pos[b] -= wb * corr;
            }
        }

        TearPass();

        SelfCollide();
        if (_drape) SphereCollide();
        FloorCollide();

        // pin enforcement
        for (int k = 0; k < Pos.Length; k++)
        {
            if (Pinned[k])
            {
                Pos[k] = PinPos[k]; Prev[k] = PinPos[k];
            }
        }
    }

    // Particle-vs-particle self-collision via a uniform spatial hash. Particles
    // closer than 2*ColRadius are pushed apart, so folds of the sheet no longer
    // pass through each other. Cell size = collision diameter, so each particle
    // only needs to test its own cell and the 26 neighbours.
    private static void SelfCollide()
    {
        float minD = 2f * ColRadius;
        float cell = minD;
        var grid = new Dictionary<long, List<int>>(Pos.Length);

        static long Key(int x, int y, int z) =>
            (long)(uint)(x + 1024) | ((long)(uint)(y + 1024) << 21) | ((long)(uint)(z + 1024) << 42);

        for (int k = 0; k < Pos.Length; k++)
        {
            int gx = (int)MathF.Floor(Pos[k].X / cell);
            int gy = (int)MathF.Floor(Pos[k].Y / cell);
            int gz = (int)MathF.Floor(Pos[k].Z / cell);
            long key = Key(gx, gy, gz);
            if (!grid.TryGetValue(key, out var list)) { list = new List<int>(); grid[key] = list; }
            list.Add(k);
        }

        for (int k = 0; k < Pos.Length; k++)
        {
            int gx = (int)MathF.Floor(Pos[k].X / cell);
            int gy = (int)MathF.Floor(Pos[k].Y / cell);
            int gz = (int)MathF.Floor(Pos[k].Z / cell);

            for (int dz = -1; dz <= 1; dz++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        if (!grid.TryGetValue(Key(gx + dx, gy + dy, gz + dz), out var list)) continue;
                        foreach (int m in list)
                        {
                            if (m <= k) continue;
                            Vector3 d = Pos[m] - Pos[k];
                            float dist = d.Length();
                            if (dist >= minD || dist < 1e-6f) continue;
                            float wa = Pinned[k] ? 0f : 1f;
                            float wb = Pinned[m] ? 0f : 1f;
                            float ws = wa + wb;
                            if (ws == 0f) continue;
                            Vector3 corr = d * ((minD - dist) / dist);
                            Pos[k] -= corr * (wa / ws);
                            Pos[m] += corr * (wb / ws);
                        }
                    }
        }
    }

    // Snap any constraint stretched past Rest * TearFactor. Removal is O(1) via
    // swap-with-last (constraint order is irrelevant to PBD). When a *structural*
    // edge snaps we also mark it dead so the mesh opens a hole there next rebuild.
    private static void TearPass()
    {
        if (!_tearing) return;
        for (int c = Cons.Count - 1; c >= 0; c--)
        {
            var con = Cons[c];
            float maxLen = con.Rest * TearFactor;
            if (Vector3.DistanceSquared(Pos[con.A], Pos[con.B]) <= maxLen * maxLen) continue;

            FlagEdgeTorn(con.A, con.B);
            Cons[c] = Cons[Cons.Count - 1];
            Cons.RemoveAt(Cons.Count - 1);
        }
    }

    private static int HEdge(int i, int j) => i + j * (N - 1);
    private static int VEdge(int i, int j) => i + j * N;

    private static void FlagEdgeTorn(int a, int b)
    {
        int ai = a % N, aj = a / N, bi = b % N, bj = b / N;
        if (aj == bj && Math.Abs(ai - bi) == 1)       // horizontal structural edge
        {
            _hAlive[HEdge(Math.Min(ai, bi), aj)] = false;
            _topoDirty = true;
        }
        else if (ai == bi && Math.Abs(aj - bj) == 1)  // vertical structural edge
        {
            _vAlive[VEdge(ai, Math.Min(aj, bj))] = false;
            _topoDirty = true;
        }
        // shear / bend constraints just vanish — they bridge no quad on their own
    }

    private static bool QuadAlive(int i, int j) =>
        _hAlive[HEdge(i, j)] && _hAlive[HEdge(i, j + 1)] &&
        _vAlive[VEdge(i, j)] && _vAlive[VEdge(i + 1, j)];

    // Push every particle out of the sphere along the surface normal, then bleed
    // off part of the tangential velocity so the cloth grips and drapes instead
    // of sliding straight off. Verlet "velocity" is encoded as Pos - Prev, so we
    // apply friction by nudging Prev toward Pos along the tangent.
    private static void SphereCollide()
    {
        float r = SphereR + ColEps;
        for (int k = 0; k < Pos.Length; k++)
        {
            if (Pinned[k]) continue;
            Vector3 d = Pos[k] - SphereC;
            float dist = d.Length();
            if (dist >= r || dist < 1e-6f) continue;

            Vector3 n = d / dist;
            Pos[k] = SphereC + n * r;             // project to the surface

            Vector3 v = Pos[k] - Prev[k];          // implied velocity
            Vector3 vt = v - Vector3.Dot(v, n) * n; // tangential part
            Prev[k] += vt * Friction;               // keep (1-Friction) of the slide
        }
    }

    // Clamp every particle to sit on (or above) the ground plane y = FloorY, then
    // bleed off part of the horizontal slide so the cloth settles and piles
    // instead of skating away. Same Verlet-friction trick as the sphere: nudge
    // Prev toward Pos along the tangent. Runs in every mode, so a sheet that is
    // released (unpinned corners, dropped, or torn loose) lands here.
    private static void FloorCollide()
    {
        float fy = FloorY + FloorEps;
        for (int k = 0; k < Pos.Length; k++)
        {
            if (Pinned[k]) continue;
            if (Pos[k].Y >= fy) continue;

            Vector3 p = Pos[k];
            p.Y = fy;
            Pos[k] = p;                              // project up onto the floor

            Vector3 v = Pos[k] - Prev[k];           // implied velocity
            Vector3 vt = new(v.X, 0f, v.Z);         // horizontal (tangential) part
            Prev[k] += vt * FloorFriction;          // keep (1-FloorFriction) of the slide
        }
    }

    // Per-particle surface normals for the aerodynamic force, from central
    // differences over grid neighbors — the same estimate the render mesh uses,
    // but computed per substep so the force always sees the current shape.
    private static void ComputeAeroNormals()
    {
        for (int j = 0; j < N; j++)
            for (int i = 0; i < N; i++)
            {
                int il = Math.Max(i - 1, 0), ir = Math.Min(i + 1, N - 1);
                int jl = Math.Max(j - 1, 0), jr = Math.Min(j + 1, N - 1);
                Vector3 dx = Pos[Idx(ir, j)] - Pos[Idx(il, j)];
                Vector3 dy = Pos[Idx(i, jr)] - Pos[Idx(i, jl)];
                Vector3 nrm = Vector3.Cross(dy, dx);
                float l = nrm.Length();
                AeroN[Idx(i, j)] = l > 1e-6f ? nrm / l : Vector3.UnitZ;
            }
    }

    // P was pressed: read the back buffer (the frame just rendered, before the
    // swap) and save it as a PNG next to the executable. Fully dependency-free —
    // see Png.cs.
    private static void CaptureIfRequested()
    {
        if (!_shot) return;
        _shot = false;

        var px = new byte[_w * _h * 3];
        GL.glPixelStorei(GL.GL_PACK_ALIGNMENT, 1); // rows are tightly packed
        GCHandle h = GCHandle.Alloc(px, GCHandleType.Pinned);
        try { GL.glReadPixels(0, 0, _w, _h, GL.GL_RGB, GL.GL_UNSIGNED_BYTE, h.AddrOfPinnedObject()); }
        finally { h.Free(); }

        string name = $"screenshot_{DateTime.Now:yyyyMMdd_HHmmss}.png";
        try
        {
            Png.WriteRgbBottomUp(System.IO.Path.Combine(AppContext.BaseDirectory, name), px, _w, _h);
            _shotName = name; // surfaced in the title-bar HUD
        }
        catch (System.IO.IOException) { /* demo-grade: a failed save is not fatal */ }
    }

    // ---- Curl-noise wind ---------------------------------------------------

    // Wind acceleration at a world point: a steady breeze plus divergence-free
    // turbulence (the curl of a noise vector potential), modulated by a slow gust.
    private static Vector3 WindField(Vector3 p, float t)
    {
        if (!_windOn) return Vector3.Zero;
        float gust = 0.55f + 0.45f * MathF.Sin(t * 0.7f) * MathF.Sin(t * 0.23f + 1.3f);
        Vector3 s = p * NoiseFreq + WindDir * (t * ScrollSpeed);
        Vector3 swirl = Curl(s);
        return (WindDir * BaseBreeze + swirl * CurlStrength) * (gust * _windScale);
    }

    // Curl of the vector potential, by central differences. ∇×ψ is always
    // divergence-free, which is what makes the flow read as real moving air.
    private static Vector3 Curl(Vector3 p)
    {
        Vector3 dx = new(CurlEps, 0f, 0f), dy = new(0f, CurlEps, 0f), dz = new(0f, 0f, CurlEps);
        Vector3 pxp = Pot(p + dx), pxm = Pot(p - dx);
        Vector3 pyp = Pot(p + dy), pym = Pot(p - dy);
        Vector3 pzp = Pot(p + dz), pzm = Pot(p - dz);
        float inv = 1f / (2f * CurlEps);
        return new Vector3(
            ((pyp.Z - pym.Z) - (pzp.Y - pzm.Y)) * inv,
            ((pzp.X - pzm.X) - (pxp.Z - pxm.Z)) * inv,
            ((pxp.Y - pxm.Y) - (pyp.X - pym.X)) * inv);
    }

    // Three decorrelated noise channels form the vector potential ψ.
    static readonly Vector3 PsiOff2 = new(31.416f, 17.073f, 47.853f);
    static readonly Vector3 PsiOff3 = new(-19.34f, 83.155f, -5.926f);
    private static Vector3 Pot(Vector3 s) =>
        new(ValueNoise(s), ValueNoise(s + PsiOff2), ValueNoise(s + PsiOff3));

    // Smooth 3D value noise: hash the 8 lattice corners, trilinearly blend with a
    // quintic fade. Cheap, dependency-free, smooth enough for a potential field.
    private static float ValueNoise(Vector3 p)
    {
        int xi = (int)MathF.Floor(p.X), yi = (int)MathF.Floor(p.Y), zi = (int)MathF.Floor(p.Z);
        float xf = p.X - xi, yf = p.Y - yi, zf = p.Z - zi;
        float u = Fade(xf), v = Fade(yf), w = Fade(zf);

        float c000 = Hash3(xi, yi, zi),         c100 = Hash3(xi + 1, yi, zi);
        float c010 = Hash3(xi, yi + 1, zi),     c110 = Hash3(xi + 1, yi + 1, zi);
        float c001 = Hash3(xi, yi, zi + 1),     c101 = Hash3(xi + 1, yi, zi + 1);
        float c011 = Hash3(xi, yi + 1, zi + 1), c111 = Hash3(xi + 1, yi + 1, zi + 1);

        float x00 = Lerp(c000, c100, u), x10 = Lerp(c010, c110, u);
        float x01 = Lerp(c001, c101, u), x11 = Lerp(c011, c111, u);
        return Lerp(Lerp(x00, x10, v), Lerp(x01, x11, v), w);
    }

    private static float Fade(float t) => t * t * t * (t * (t * 6f - 15f) + 10f);
    private static float Lerp(float a, float b, float t) => a + (b - a) * t;

    private static float Hash3(int x, int y, int z)
    {
        unchecked
        {
            int h = x * 374761393 + y * 668265263 + z * 1442695040;
            h = (h ^ (h >> 13)) * 1274126177;
            h ^= h >> 16;
            return (h & 0xFFFF) / 32767.5f - 1f; // [-1, 1]
        }
    }

    // ---- Pinning interactions ---------------------------------------------

    // Pin a free corner to its current position, or release it if already pinned.
    private static void ToggleCornerPin(int i, int j)
    {
        int k = Idx(i, j);
        if (Pinned[k]) { Pinned[k] = false; }
        else { Pinned[k] = true; PinPos[k] = Pos[k]; }
    }

    // Pin the whole left edge and free everything else, so the sheet flies as a
    // flag from a vertical pole and flutters in the wind.
    private static void FlagMode()
    {
        for (int j = 0; j < N; j++)
            for (int i = 0; i < N; i++)
            {
                int k = Idx(i, j);
                if (i == 0) { Pinned[k] = true; PinPos[k] = Pos[k]; }
                else Pinned[k] = false;
            }
        _drape = false;
    }

    private static void HandleGrab(float[] mvp)
    {
        if (!_grab)
        {
            if (_grabIndex >= 0)
            {
                Pinned[_grabIndex] = _grabWasPinned;
                if (_grabWasPinned) PinPos[_grabIndex] = _grabOrigPin;
                _grabIndex = -1;
            }
            return;
        }

        if (_grabIndex < 0)
        {
            // pick the particle whose screen projection is closest to the cursor
            float best = 45f * 45f; int bi = -1;
            for (int k = 0; k < Pos.Length; k++)
            {
                Vector4 c = MulVec(mvp, Pos[k]);
                if (c.W <= 0f) continue;
                float px = (c.X / c.W * 0.5f + 0.5f) * _w;
                float py = (1f - (c.Y / c.W * 0.5f + 0.5f)) * _h;
                float dx = px - _mx, dy = py - _my;
                float d = dx * dx + dy * dy;
                if (d < best) { best = d; bi = k; }
            }
            if (bi < 0) return;
            _grabIndex = bi; _grabWasPinned = Pinned[bi]; _grabOrigPin = PinPos[bi];
        }

        // move the grabbed particle along the cursor ray, keeping its depth
        Vector4 clip = MulVec(mvp, Pos[_grabIndex]);
        if (clip.W <= 0f) return;
        float ndcZ = clip.Z / clip.W;
        float mx = _mx / (float)_w * 2f - 1f;
        float my = 1f - _my / (float)_h * 2f;
        float[] inv = Invert(mvp);
        Vector4 w = MulVec4(inv, mx, my, ndcZ, 1f);
        var target = new Vector3(w.X / w.W, w.Y / w.W, w.Z / w.W);
        Pos[_grabIndex] = target;
        PinPos[_grabIndex] = target;
        Pinned[_grabIndex] = true;
    }

    private static Vector4 MulVec(float[] m, Vector3 p) => MulVec4(m, p.X, p.Y, p.Z, 1f);

    private static Vector4 MulVec4(float[] m, float x, float y, float z, float w) => new Vector4(
        m[0] * x + m[4] * y + m[8] * z + m[12] * w,
        m[1] * x + m[5] * y + m[9] * z + m[13] * w,
        m[2] * x + m[6] * y + m[10] * z + m[14] * w,
        m[3] * x + m[7] * y + m[11] * z + m[15] * w);

    private static float[] Invert(float[] m)
    {
        var inv = new float[16];
        inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
        inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
        inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
        inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
        inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
        inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
        inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
        inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
        inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
        inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
        inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
        inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
        inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
        inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
        inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
        inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

        float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
        if (MathF.Abs(det) < 1e-12f) return inv;
        det = 1f / det;
        for (int i = 0; i < 16; i++) inv[i] *= det;
        return inv;
    }

    private static void BuildClothMesh()
    {
        if (MeshV == null) MeshV = new float[N * N * 8];
        if (MeshI == null || _topoDirty) RebuildIndices();

        for (int j = 0; j < N; j++)
        {
            for (int i = 0; i < N; i++)
            {
                int il = Math.Max(i - 1, 0), ir = Math.Min(i + 1, N - 1);
                int jl = Math.Max(j - 1, 0), jr = Math.Min(j + 1, N - 1);
                Vector3 dx = Pos[Idx(ir, j)] - Pos[Idx(il, j)];
                Vector3 dy = Pos[Idx(i, jr)] - Pos[Idx(i, jl)];
                Vector3 nrm = Vector3.Cross(dy, dx);
                float l = nrm.Length();
                nrm = l > 1e-6f ? nrm / l : Vector3.UnitZ;

                int o = Idx(i, j) * 8;
                Vector3 p = Pos[Idx(i, j)];
                MeshV[o] = p.X; MeshV[o + 1] = p.Y; MeshV[o + 2] = p.Z;
                MeshV[o + 3] = nrm.X; MeshV[o + 4] = nrm.Y; MeshV[o + 5] = nrm.Z;
                MeshV[o + 6] = i / (float)(N - 1);
                MeshV[o + 7] = j / (float)(N - 1);
            }
        }
    }

    // Emit two triangles per surviving quad. Torn quads are skipped, leaving holes.
    private static void RebuildIndices()
    {
        var idx = new List<uint>((N - 1) * (N - 1) * 6);
        for (int j = 0; j < N - 1; j++)
            for (int i = 0; i < N - 1; i++)
            {
                if (!QuadAlive(i, j)) continue;
                uint a = (uint)Idx(i, j), b = (uint)Idx(i + 1, j);
                uint c = (uint)Idx(i, j + 1), d = (uint)Idx(i + 1, j + 1);
                idx.Add(a); idx.Add(b); idx.Add(c);
                idx.Add(b); idx.Add(d); idx.Add(c);
            }
        MeshI = idx.ToArray();
        MeshICount = MeshI.Length;
    }

    private static void ReuploadClothIndices(uint clothVao)
    {
        GL.glBindVertexArray(clothVao);
        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, _clothEbo);
        UploadU(GL.GL_ELEMENT_ARRAY_BUFFER, MeshI);
    }

    // UV sphere as position(3) + normal(3), matching SetPosNormalAttribs.
    private static void BuildSphereMesh(int lat, int lon)
    {
        var v = new List<float>((lat + 1) * (lon + 1) * 6);
        var idx = new List<uint>(lat * lon * 6);

        for (int y = 0; y <= lat; y++)
        {
            float theta = y / (float)lat * MathF.PI;          // 0..pi
            float st = MathF.Sin(theta), ct = MathF.Cos(theta);
            for (int x = 0; x <= lon; x++)
            {
                float phi = x / (float)lon * 2f * MathF.PI;    // 0..2pi
                float nx = st * MathF.Cos(phi);
                float ny = ct;
                float nz = st * MathF.Sin(phi);
                v.Add(SphereC.X + SphereR * nx); v.Add(SphereC.Y + SphereR * ny); v.Add(SphereC.Z + SphereR * nz);
                v.Add(nx); v.Add(ny); v.Add(nz);
            }
        }

        int row = lon + 1;
        for (int y = 0; y < lat; y++)
            for (int x = 0; x < lon; x++)
            {
                uint a = (uint)(y * row + x), b = (uint)(y * row + x + 1);
                uint c = (uint)((y + 1) * row + x), d = (uint)((y + 1) * row + x + 1);
                idx.Add(a); idx.Add(c); idx.Add(b);
                idx.Add(b); idx.Add(c); idx.Add(d);
            }

        SphV = v.ToArray();
        SphI = idx.ToArray();
        SphICount = SphI.Length;
    }

    // Reposition the sheet as a flat horizontal plane just above the sphere and
    // release it with zero velocity, so it falls and drapes. Press SPACE.
    private static void DropOntoSphere()
    {
        InitCloth();               // fresh, intact sheet every drop
        float y = SphereC.Y + SphereR + 0.45f;
        for (int j = 0; j < N; j++)
            for (int i = 0; i < N; i++)
            {
                int k = Idx(i, j);
                var p = new Vector3(
                    (i / (float)(N - 1) - 0.5f) * W,
                    y,
                    (j / (float)(N - 1) - 0.5f) * W);
                p += Jitter(i, j);             // seed a natural (non-mirror) drape
                Pos[k] = p; Prev[k] = p;   // zero initial velocity
                Pinned[k] = false;
            }
        _drape = true;
    }

    // Reposition the sheet as a flat horizontal plane up in the air (no sphere)
    // and release it with zero velocity, so it falls flat and piles on the floor.
    // Press C.
    private static void DropOntoFloor()
    {
        InitCloth();               // fresh, intact sheet every drop
        float y = 0.7f;            // start well above the floor
        for (int j = 0; j < N; j++)
            for (int i = 0; i < N; i++)
            {
                int k = Idx(i, j);
                var p = new Vector3(
                    (i / (float)(N - 1) - 0.5f) * W,
                    y,
                    (j / (float)(N - 1) - 0.5f) * W);
                p += Jitter(i, j);             // seed a natural (non-mirror) drape
                Pos[k] = p; Prev[k] = p;   // zero initial velocity
                Pinned[k] = false;         // nothing pinned: the whole sheet falls
            }
        _drape = false;            // no sphere in this scene
    }

    // Planar projection matrix: flatten geometry onto the plane y = h along the
    // light's travel direction l (so the shadow tracks the light). Built in the
    // same column-major float[16] layout the rest of the math here uses.
    private static float[] ShadowMatrix(float h, Vector3 l)
    {
        float a = 1f / l.Y;        // l.Y is negative (light points downward)
        var m = new float[16];
        m[0] = 1f;            m[4] = -l.X * a;    m[8]  = 0f; m[12] = l.X * a * h;
        m[1] = 0f;            m[5] = 0f;          m[9]  = 0f; m[13] = h;
        m[2] = 0f;            m[6] = -l.Z * a;    m[10] = 1f; m[14] = l.Z * a * h;
        m[3] = 0f;            m[7] = 0f;          m[11] = 0f; m[15] = 1f;
        return m;
    }

    private static float[] Perspective(float fovy, float aspect, float n, float f)
    {
        float t = 1f / MathF.Tan(fovy * 0.5f);
        var m = new float[16];
        m[0] = t / aspect; m[5] = t;
        m[10] = (f + n) / (n - f); m[11] = -1f;
        m[14] = (2f * f * n) / (n - f);
        return m;
    }

    private static float[] LookAt(Vector3 eye, Vector3 center, Vector3 up)
    {
        var f = Vector3.Normalize(center - eye);
        var s = Vector3.Normalize(Vector3.Cross(f, up));
        var u = Vector3.Cross(s, f);

        return
        [
            s.X, u.X, -f.X, 0f,
            s.Y, u.Y, -f.Y, 0f,
            s.Z, u.Z, -f.Z, 0f,
            -Vector3.Dot(s, eye), -Vector3.Dot(u, eye), Vector3.Dot(f, eye), 1f
        ];
    }

    private static float[] Mul(float[] a, float[] b)
    {
        var r = new float[16];

        for (int col = 0; col < 4; col++)
        {
            for (int row = 0; row < 4; row++)
            {
                float s = 0f;
                for (int k = 0; k < 4; k++) s += a[k * 4 + row] * b[col * 4 + k];
                r[col * 4 + row] = s;
            }
        }

        return r;
    }

    private static void MakeMeshVao(out uint vao, out uint vbo)
    {
        BuildClothMesh(); // ensures MeshV/MeshI exist
        vao = 0; vbo = 0;
        GL.glGenVertexArrays(1, ref vao);
        GL.glBindVertexArray(vao);
        GL.glGenBuffers(1, ref vbo);
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo);
        GL.glBufferData(GL.GL_ARRAY_BUFFER, MeshV.Length * sizeof(float), IntPtr.Zero, GL.GL_DYNAMIC_DRAW);
        _clothEbo = 0;
        GL.glGenBuffers(1, ref _clothEbo);
        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, _clothEbo);
        UploadU(GL.GL_ELEMENT_ARRAY_BUFFER, MeshI);
        SetClothAttribs();
        _topoDirty = false;
    }

    private static void SetClothAttribs()
    {
        int stride = 8 * sizeof(float);
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, 0, stride, 0);
        GL.glEnableVertexAttribArray(0);
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, 0, stride, 3 * sizeof(float));
        GL.glEnableVertexAttribArray(1);
        GL.glVertexAttribPointer(2, 2, GL.GL_FLOAT, 0, stride, 6 * sizeof(float));
        GL.glEnableVertexAttribArray(2);
    }

    private static void MakeStaticVao(float[] verts, uint[] indices, out uint vao)
    {
        vao = 0; uint vbo = 0, ebo = 0;
        GL.glGenVertexArrays(1, ref vao);
        GL.glBindVertexArray(vao);
        GL.glGenBuffers(1, ref vbo);
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo);
        UploadF(GL.GL_ARRAY_BUFFER, verts, GL.GL_STATIC_DRAW);
        GL.glGenBuffers(1, ref ebo);
        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, ebo);
        UploadU(GL.GL_ELEMENT_ARRAY_BUFFER, indices);
        SetPosNormalAttribs();
    }

    private static void SetPosNormalAttribs()
    {
        int stride = 6 * sizeof(float);
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, 0, stride, 0);
        GL.glEnableVertexAttribArray(0);
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, 0, stride, 3 * sizeof(float));
        GL.glEnableVertexAttribArray(1);
    }

    private static void UploadCloth(uint vbo)
    {
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo);
        GCHandle h = GCHandle.Alloc(MeshV, GCHandleType.Pinned);

        try
        {
            GL.glBufferData(GL.GL_ARRAY_BUFFER, MeshV.Length * sizeof(float), h.AddrOfPinnedObject(), GL.GL_DYNAMIC_DRAW);
        }
        finally
        {
            h.Free();
        }
    }

    private static void UploadF(uint target, float[] data, uint usage)
    {
        GCHandle h = GCHandle.Alloc(data, GCHandleType.Pinned);

        try
        {
            GL.glBufferData(target, data.Length * sizeof(float), h.AddrOfPinnedObject(), usage);
        }
        finally
        {
            h.Free();
        }
    }

    private static void UploadU(uint target, uint[] data)
    {
        GCHandle h = GCHandle.Alloc(data, GCHandleType.Pinned);

        try
        {
            GL.glBufferData(target, data.Length * sizeof(uint), h.AddrOfPinnedObject(), GL.GL_STATIC_DRAW);
        }
        finally
        {
            h.Free();
        }
    }

    private static uint MakeQuadVao()
    {
        float[] q = [-1f, -1f, 0f, 0f, 1f, -1f, 1f, 0f, -1f, 1f, 0f, 1f, 1f, 1f, 1f, 1f];
        uint vao = 0, vbo = 0;
        GL.glGenVertexArrays(1, ref vao);
        GL.glBindVertexArray(vao);
        GL.glGenBuffers(1, ref vbo);
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo);
        UploadF(GL.GL_ARRAY_BUFFER, q, GL.GL_STATIC_DRAW);
        int stride = 4 * sizeof(float);
        GL.glVertexAttribPointer(0, 2, GL.GL_FLOAT, 0, stride, 0);
        GL.glEnableVertexAttribArray(0);
        GL.glVertexAttribPointer(1, 2, GL.GL_FLOAT, 0, stride, 2 * sizeof(float));
        GL.glEnableVertexAttribArray(1);
        return vao;
    }

    // Procedural plain-weave fabric texture: alternating warp/weft threads, each
    // raised thread caught by a soft highlight, with a little per-thread noise.
    private static uint MakeFabricTexture(int s)
    {
        var px = new byte[s * s * 3];
        const int threads = 8; // threads per tile

        static float Hash(int x, int y)
        {
            int h = x * 374761393 + y * 668265263;
            h = (h ^ (h >> 13)) * 1274126177;
            return ((h ^ (h >> 16)) & 0xFFFF) / 65535f;
        }
        static byte B(float f) => (byte)Math.Clamp((int)(f * 255f), 0, 255);

        for (int y = 0; y < s; y++)
            for (int x = 0; x < s; x++)
            {
                float fx = x / (float)s * threads;
                float fy = y / (float)s * threads;
                int cx = (int)MathF.Floor(fx), cy = (int)MathF.Floor(fy);
                float tx = fx - cx, ty = fy - cy;
                bool over = ((cx + cy) & 1) == 0;                 // plain weave
                float shade = over ? MathF.Sin(ty * MathF.PI) : MathF.Sin(tx * MathF.PI);
                shade = 0.45f + 0.55f * shade;
                float n = Hash(cx, cy) * 0.08f - 0.04f;           // thread-to-thread variation
                int o = (y * s + x) * 3;
                px[o]     = B(shade * 0.96f + n);
                px[o + 1] = B(shade * 0.93f + n);
                px[o + 2] = B(shade * 0.90f + n);
            }

        uint tex = 0;
        GL.glGenTextures(1, ref tex);
        GL.glActiveTexture(GL.GL_TEXTURE0);
        GL.glBindTexture(GL.GL_TEXTURE_2D, tex);
        GCHandle h = GCHandle.Alloc(px, GCHandleType.Pinned);
        try
        {
            GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, (int)GL.GL_RGB8, s, s, 0,
                            GL.GL_RGB, GL.GL_UNSIGNED_BYTE, h.AddrOfPinnedObject());
        }
        finally { h.Free(); }
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, (int)GL.GL_REPEAT);
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, (int)GL.GL_REPEAT);
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, (int)GL.GL_LINEAR);
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, (int)GL.GL_LINEAR_MIPMAP_LINEAR);
        GL.glGenerateMipmap(GL.GL_TEXTURE_2D);
        return tex;
    }

    private const string LitVS = @"#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;
uniform mat4 uMVP;
out vec3 vN;
out vec3 vW;
out vec2 vUV;
void main(){
    vW = aPos;
    vN = aNormal;
    vUV = aUV;
    gl_Position = uMVP * vec4(aPos, 1.0);
}";

    private const string LitFS = @"#version 330 core
in vec3 vN;
in vec3 vW;
in vec2 vUV;
uniform vec3 uCam;
uniform vec3 uLight;
uniform vec3 uFront;
uniform vec3 uBack;
uniform sampler2D uTex;
out vec4 FragColor;
void main(){
    vec3 N = normalize(vN);
    vec3 V = normalize(uCam - vW);
    bool front = dot(N, V) >= 0.0;     // which geometric side faces the camera
    if (!front) N = -N;                // light the side we actually see
    vec3 L = normalize(uLight);
    float diff = max(dot(N, L), 0.0);
    vec3 weave = texture(uTex, vUV * 6.0).rgb;
    vec3 base = (front ? uFront : uBack) * weave * 1.25;
    vec3 col = base * (0.25 + 0.75 * diff);
    vec3 H = normalize(L + V);
    col += vec3(1.0) * pow(max(dot(N, H), 0.0), 40.0) * 0.25;
    col = col / (col + 0.85);
    FragColor = vec4(pow(col, vec3(0.85)), 1.0);
}";

    private const string SphereVS = @"#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 uMVP;
out vec3 vN;
out vec3 vW;
void main(){
    vW = aPos;
    vN = aNormal;
    gl_Position = uMVP * vec4(aPos, 1.0);
}";

    private const string SphereFS = @"#version 330 core
in vec3 vN;
in vec3 vW;
uniform vec3 uCam;
uniform vec3 uLight;
uniform vec3 uColor;
out vec4 FragColor;
void main(){
    vec3 N = normalize(vN);
    vec3 V = normalize(uCam - vW);
    vec3 L = normalize(uLight);
    float diff = max(dot(N, L), 0.0);
    vec3 col = uColor * (0.18 + 0.82 * diff);
    vec3 H = normalize(L + V);
    col += vec3(1.0) * pow(max(dot(N, H), 0.0), 24.0) * 0.18;
    col = col / (col + 0.85);
    FragColor = vec4(pow(col, vec3(0.85)), 1.0);
}";

    private const string QuadVS = @"#version 330 core
layout(location=0) in vec2 aPos;
layout(location=1) in vec2 aUV;
uniform vec2 uScale;
out vec2 vUV;
void main(){ vUV = aUV; gl_Position = vec4(aPos * uScale, 0.0, 1.0); }";

    private const string GradFS = @"#version 330 core
in vec2 vUV;
out vec4 FragColor;
void main(){
    float d = length(vUV - vec2(0.5, 0.55));
    vec3 c = mix(vec3(0.05, 0.055, 0.085), vec3(0.008, 0.008, 0.014), smoothstep(0.0, 0.85, d));
    FragColor = vec4(c, 1.0);
}";

    // Ground plane: pos(3) + normal(3). A soft checkerboard from world XZ, fading
    // out with distance so the plane reads as ground without looking like an
    // infinite hard grid.
    private const string FloorVS = @"#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 uMVP;
out vec3 vW;
out vec3 vN;
void main(){ vW = aPos; vN = aNormal; gl_Position = uMVP * vec4(aPos, 1.0); }";

    private const string FloorFS = @"#version 330 core
in vec3 vW;
in vec3 vN;
uniform vec3 uCam;
uniform vec3 uLight;
out vec4 FragColor;
void main(){
    vec2 g = vW.xz;
    float checker = mod(floor(g.x) + floor(g.y), 2.0);   // 1m tiles
    vec3 a = vec3(0.052, 0.055, 0.070);
    vec3 b = vec3(0.034, 0.036, 0.048);
    vec3 base = mix(a, b, checker);
    // thin grid lines on top of the tiles
    vec2 f = abs(fract(g) - 0.5);
    float line = smoothstep(0.49, 0.5, max(f.x, f.y));
    base = mix(base, vec3(0.09, 0.10, 0.13), line * 0.5);
    float diff = max(dot(normalize(vN), normalize(uLight)), 0.0);
    vec3 col = base * (0.55 + 0.45 * diff);
    float fade = smoothstep(11.0, 3.0, length(uCam.xz - g)); // dim toward the horizon
    col *= mix(0.25, 1.0, fade);
    FragColor = vec4(col, 1.0);
}";

    // Cloth flattened onto the floor for a soft contact shadow. Reads position
    // only; the projection is baked into uMVP on the CPU.
    private const string ShadowVS = @"#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
void main(){ gl_Position = uMVP * vec4(aPos, 1.0); }";

    private const string ShadowFS = @"#version 330 core
uniform vec3 uColor;
uniform float uAlpha;
out vec4 FragColor;
void main(){ FragColor = vec4(uColor, uAlpha); }";

    private static uint BuildProgram(string vsSrc, string fsSrc)
    {
        uint vs = Compile(GL.GL_VERTEX_SHADER, vsSrc);
        uint fs = Compile(GL.GL_FRAGMENT_SHADER, fsSrc);
        uint p = GL.glCreateProgram();
        GL.glAttachShader(p, vs);
        GL.glAttachShader(p, fs);
        GL.glLinkProgram(p);
        int ok = 0; GL.glGetProgramiv(p, GL.GL_LINK_STATUS, ref ok);

        if (ok == 0)
        {
            var log = new byte[2048]; int len = 0;
            GL.glGetProgramInfoLog(p, log.Length, ref len, log);
            throw new Exception("Link error: " + System.Text.Encoding.ASCII.GetString(log, 0, len));
        }

        GL.glDeleteShader(vs); GL.glDeleteShader(fs);
        return p;
    }

    private static uint Compile(uint type, string src)
    {
        uint sh = GL.glCreateShader(type);
        IntPtr str = Marshal.StringToHGlobalAnsi(src);

        try
        {
            GL.glShaderSource(sh, 1, [str], IntPtr.Zero);
        }
        finally
        {
            Marshal.FreeHGlobal(str);
        }

        GL.glCompileShader(sh);
        int ok = 0; GL.glGetShaderiv(sh, GL.GL_COMPILE_STATUS, ref ok);

        if (ok == 0)
        {
            var log = new byte[2048]; int len = 0;
            GL.glGetShaderInfoLog(sh, log.Length, ref len, log);
            throw new Exception("Compile error: " + System.Text.Encoding.ASCII.GetString(log, 0, len));
        }

        return sh;
    }

    private static byte[] Ascii(string s)
    {
        var b = new byte[s.Length + 1];
        System.Text.Encoding.ASCII.GetBytes(s, 0, s.Length, b, 0);
        return b;
    }

    private static IntPtr CreateGLContext(IntPtr hdc)
    {
        var pfd = new Win.PIXELFORMATDESCRIPTOR
        {
            nSize = (ushort)Marshal.SizeOf<Win.PIXELFORMATDESCRIPTOR>(),
            nVersion = 1,
            dwFlags = Win.PFD_DRAW_TO_WINDOW | Win.PFD_SUPPORT_OPENGL | Win.PFD_DOUBLEBUFFER,
            iPixelType = Win.PFD_TYPE_RGBA,
            cColorBits = 32,
            cDepthBits = 24,
            cStencilBits = 8,
            iLayerType = Win.PFD_MAIN_PLANE,
        };

        int fmt = Win.ChoosePixelFormat(hdc, ref pfd);

        if (fmt == 0) throw new Exception("ChoosePixelFormat failed");
        if (!Win.SetPixelFormat(hdc, fmt, ref pfd)) throw new Exception("SetPixelFormat failed");

        IntPtr tmp = Win.wglCreateContext(hdc);
        Win.wglMakeCurrent(hdc, tmp);
        IntPtr proc = Win.wglGetProcAddress("wglCreateContextAttribsARB");

        if (proc != IntPtr.Zero)
        {
            var create = Marshal.GetDelegateForFunctionPointer<GL.WglCreateContextAttribsARB>(proc);
            int[] attributes = [0x2091, 4, 0x2092, 3, 0x9126, 0x0001, 0]; // OpenGL 4.3 core (compute shaders)
            IntPtr core = create(hdc, IntPtr.Zero, attributes);

            if (core != IntPtr.Zero)
            {
                Win.wglMakeCurrent(hdc, core);
                Win.wglDeleteContext(tmp);
                return core;
            }
        }

        return tmp;
    }
}
