// Cloth — a Verlet cloth simulation on raw Win32 + WGL + OpenGL 3.3 core.
// Zero external dependencies (System.Numerics is part of the BCL).
//
//   A grid of point masses is integrated with Verlet integration and held
//   together by distance constraints (structural, shear and bend), solved with
//   several Position-Based-Dynamics relaxation passes per frame. The sheet hangs
//   from its top corners, is pushed by wind, and drapes over a sphere it collides
//   with. The result is rebuilt into a lit, two-sided triangle mesh every frame.
//
//   Drag with the left mouse button to orbit, mouse wheel to zoom.
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
    const int Iters = 10; // constraint relaxation passes
    const float Dt = 1f / 120f;

    static readonly Vector3 SphereC = new(0f, -0.15f, 0f);
    const float SphereR = 0.6f;

    static Vector3[] Pos, Prev, PinPos;
    static bool[] Pinned;

    struct Con { public int A, B; public float Rest; }
    static readonly List<Con> Cons = new();

    static float[] MeshV; // pos(3) + normal(3) per particle
    static uint[] MeshI; // triangle indices
    static int MeshICount;
    static float[] SphV;
    static uint[] SphI;
    static int SphICount;

    static int _w = 1080, _h = 1080;
    static bool _running = true;
    static Win.WndProc _wndProcRef;

    static float _yaw = 0.5f, _pitch = 0.15f, _dist = 4.0f;
    static bool _grab, _orbit;
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
            0, cls, "Cloth — Verlet + constraints — C#/OpenGL  (left = drag cloth, right = rotate, wheel = zoom)",
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

        int gScale = GL.glGetUniformLocation(gradProg, Ascii("uScale"));
        int uMVP = GL.glGetUniformLocation(litProg, Ascii("uMVP"));
        int uCam = GL.glGetUniformLocation(litProg, Ascii("uCam"));
        int uLight = GL.glGetUniformLocation(litProg, Ascii("uLight"));
        int uFront = GL.glGetUniformLocation(litProg, Ascii("uFront"));
        int uBack = GL.glGetUniformLocation(litProg, Ascii("uBack"));

        uint quadVao = MakeQuadVao();
        MakeMeshVao(out uint clothVao, out uint clothVbo);

        var clock = Stopwatch.StartNew();

        while (_running)
        {
            while (Win.PeekMessageW(out var msg, IntPtr.Zero, 0, 0, Win.PM_REMOVE))
            {
                if (msg.message == Win.WM_QUIT) { _running = false; break; }
                Win.TranslateMessage(ref msg);
                Win.DispatchMessageW(ref msg);
            }
            if (!_running) break;

            float t = (float)clock.Elapsed.TotalSeconds;

            // camera
            var eye = Target + new Vector3(
                _dist * MathF.Cos(_pitch) * MathF.Sin(_yaw),
                _dist * MathF.Sin(_pitch),
                _dist * MathF.Cos(_pitch) * MathF.Cos(_yaw));
            float aspect = (float)_w / _h;
            float[] mvp = Mul(Perspective(0.85f, aspect, 0.05f, 60f), LookAt(eye, Target, Vector3.UnitY));

            HandleGrab(mvp);

            for (int s = 0; s < SubSteps; s++) Step(t);
            BuildClothMesh();
            UploadCloth(clothVbo);

            GL.glViewport(0, 0, _w, _h);
            GL.glClearColor(0f, 0f, 0f, 1f);
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);

            GL.glDisable(GL.GL_DEPTH_TEST);
            GL.glUseProgram(gradProg);
            GL.glUniform2f(gScale, 1f, 1f);
            GL.glBindVertexArray(quadVao);
            GL.glDrawArrays(GL.GL_TRIANGLE_STRIP, 0, 4);

            GL.glEnable(GL.GL_DEPTH_TEST);
            GL.glUseProgram(litProg);
            GL.glUniformMatrix4fv(uMVP, 1, 0, mvp);
            GL.glUniform3f(uCam, eye.X, eye.Y, eye.Z);
            GL.glUniform3f(uLight, 0.4f, 0.85f, 0.5f);

            // cloth (two-sided)
            GL.glUniform3f(uFront, 0.85f, 0.12f, 0.22f);
            GL.glUniform3f(uBack, 0.20f, 0.10f, 0.35f);
            GL.glBindVertexArray(clothVao);
            GL.glDrawElements(GL.GL_TRIANGLES, MeshICount, GL.GL_UNSIGNED_INT, IntPtr.Zero);

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
        int n = N * N;
        Pos = new Vector3[n]; Prev = new Vector3[n]; PinPos = new Vector3[n]; Pinned = new bool[n];

        float top = 1.0f, frontZ = 0.7f, cell = W / (N - 1);
        for (int j = 0; j < N; j++)
            for (int i = 0; i < N; i++)
            {
                var p = new Vector3((i / (float)(N - 1) - 0.5f) * W, top - j / (float)(N - 1) * W, frontZ);
                int k = Idx(i, j);
                Pos[k] = p; Prev[k] = p;
            }

        // pin the two top corners
        Pin(0, 0); Pin(N - 1, 0);

        void Add(int ai, int aj, int bi, int bj)
        {
            if (bi < 0 || bi >= N || bj < 0 || bj >= N) return;
            int a = Idx(ai, aj), b = Idx(bi, bj);
            Cons.Add(new Con { A = a, B = b, Rest = Vector3.Distance(Pos[a], Pos[b]) });
        }

        for (int j = 0; j < N; j++)
            for (int i = 0; i < N; i++)
            {
                Add(i, j, i + 1, j);            // structural
                Add(i, j, i, j + 1);
                Add(i, j, i + 1, j + 1);        // shear
                Add(i + 1, j, i, j + 1);
                Add(i, j, i + 2, j);            // bend
                Add(i, j, i, j + 2);
            }
    }

    private static void Pin(int i, int j)
    {
        int k = Idx(i, j);
        Pinned[k] = true; PinPos[k] = Pos[k];
    }

    private static void Step(float t)
    {
        var grav = new Vector3(0f, -Gravity, 0f);
        var wind = new Vector3(0.5f * MathF.Sin(t * 0.9f),
                               0f,
                               0.8f * MathF.Sin(t * 0.6f));

        // Verlet integration
        for (int k = 0; k < Pos.Length; k++)
        {
            if (Pinned[k]) continue;
            // a touch of per-particle turbulence so the sheet ripples
            var turb = new Vector3(0f, 0f, 0.4f * MathF.Sin(t * 2.3f + Pos[k].X * 4f + Pos[k].Y * 3f));
            Vector3 acc = grav + wind + turb;
            Vector3 cur = Pos[k];
            Pos[k] = cur + (cur - Prev[k]) * Damp + acc * (Dt * Dt);
            Prev[k] = cur;
        }

        // constraint relaxation
        for (int it = 0; it < Iters; it++)
        {
            for (int c = 0; c < Cons.Count; c++)
            {
                var con = Cons[c];
                int a = con.A, b = con.B;
                Vector3 d = Pos[b] - Pos[a];
                float len = d.Length();
                if (len < 1e-6f) continue;
                float diff = (len - con.Rest) / len;
                float wa = Pinned[a] ? 0f : 1f;
                float wb = Pinned[b] ? 0f : 1f;
                float ws = wa + wb;
                if (ws == 0f) continue;
                Vector3 corr = d * diff;
                Pos[a] += corr * (wa / ws);
                Pos[b] -= corr * (wb / ws);
            }
        }

        // pin enforcement
        for (int k = 0; k < Pos.Length; k++)
        {
            if (Pinned[k])
            {
                Pos[k] = PinPos[k]; Prev[k] = PinPos[k];
            }
        }
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
        if (MeshV == null)
        {
            MeshV = new float[N * N * 6];
            var idx = new List<uint>();
            for (int j = 0; j < N - 1; j++)
                for (int i = 0; i < N - 1; i++)
                {
                    uint a = (uint)Idx(i, j), b = (uint)Idx(i + 1, j);
                    uint c = (uint)Idx(i, j + 1), d = (uint)Idx(i + 1, j + 1);
                    idx.Add(a); idx.Add(b); idx.Add(c);
                    idx.Add(b); idx.Add(d); idx.Add(c);
                }
            MeshI = idx.ToArray();
            MeshICount = MeshI.Length;
        }

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

                int o = Idx(i, j) * 6;
                Vector3 p = Pos[Idx(i, j)];
                MeshV[o] = p.X; MeshV[o + 1] = p.Y; MeshV[o + 2] = p.Z;
                MeshV[o + 3] = nrm.X; MeshV[o + 4] = nrm.Y; MeshV[o + 5] = nrm.Z;
            }
        }
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
        uint ebo = 0;
        GL.glGenBuffers(1, ref ebo);
        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, ebo);
        UploadU(GL.GL_ELEMENT_ARRAY_BUFFER, MeshI);
        SetPosNormalAttribs();
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

    private const string LitVS = @"#version 330 core
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

    private const string LitFS = @"#version 330 core
in vec3 vN;
in vec3 vW;
uniform vec3 uCam;
uniform vec3 uLight;
uniform vec3 uFront;
uniform vec3 uBack;
out vec4 FragColor;
void main(){
    vec3 N = normalize(vN);
    vec3 V = normalize(uCam - vW);
    bool front = dot(N, V) >= 0.0;     // which geometric side faces the camera
    if (!front) N = -N;                // light the side we actually see
    vec3 L = normalize(uLight);
    float diff = max(dot(N, L), 0.0);
    vec3 base = front ? uFront : uBack;
    vec3 col = base * (0.25 + 0.75 * diff);
    vec3 H = normalize(L + V);
    col += vec3(1.0) * pow(max(dot(N, H), 0.0), 40.0) * 0.25;
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
            int[] attributes = [0x2091, 3, 0x2092, 3, 0x9126, 0x0001, 0];
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
