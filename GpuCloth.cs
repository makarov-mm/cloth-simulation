// GpuCloth — a high-resolution XPBD cloth solved entirely on the GPU with
// compute shaders. Positions never leave VRAM: integration, the graph-colored
// constraint solve, and the render-mesh (positions + normals) all run in
// compute, and the draw call pulls vertices straight from the same buffer.
//
//   * 320 x 320 = 102,400 particles, ~610k distance constraints.
//   * Structural + shear + bend constraints, partitioned into 12 conflict-free
//     colors (no two constraints in a color share a particle), so each color is
//     one race-free dispatch — no atomics needed.
//   * Constraint *type* lives in the buffer; the compliance for each type is a
//     per-dispatch uniform, so switching material presets (M) costs nothing —
//     no buffer re-upload, exactly the XPBD promise.
//   * Curl-noise wind ported to GLSL, with the same optional aerodynamic
//     (normal-pressure) coupling as the CPU path; the surface normals come for
//     free from the previous frame's render-mesh build pass.
//   * Ground-plane collision with tangential friction, as on the CPU path.
//
// This mode deliberately omits self-collision, tearing and the sphere: those
// need atomic spatial hashing / dynamic topology and are their own GPU project.
// The full-featured demo stays on the CPU path (toggle with G).
//
// Requires an OpenGL 4.3 core context (see CreateGLContext in Program.cs).

using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;

namespace Cloth;

internal static class GpuCloth
{
    const int GN = 320;            // grid resolution per side (GN*GN particles)
    const float Width = 3.0f;      // world size of the sheet
    const int SubSteps = 4;
    const int Iters = 4;
    const float Dt = 1f / 240f;    // SubSteps * Dt = 1/60 s per frame (real time)
    const float Gravity = 9.8f;
    const float Damp = 0.99f;

    // Ground plane — same plane the CPU path uses, so both scenes share a floor.
    const float FloorY = -1.6f;
    const float FloorEps = 0.01f;
    const float FloorFriction = 0.3f;

    // Aerodynamic coupling, mirroring Program.cs (see the comment there).
    const float AeroCoeff = 0.14f;
    const float AeroMaxRel = 30f;

    const int Local = 256;  // compute local_size_x
    const int Colors = 12;  // 4 structural + 4 shear + 4 bend

    static int M;          // particle count
    static int ConCount;   // total constraints
    static int IdxCount;   // render index count

    public static int ParticleCount => M;        // for the title-bar HUD
    public static int ConstraintCount => ConCount;

    static uint _posBuf, _prevBuf, _lamBuf, _conBuf, _vtxBuf, _ebo, _vao;
    static uint _predict, _clear, _solve, _floor, _build;

    // colour ranges into _conBuf (offset,count)
    static readonly int[] _colOff = new int[Colors];
    static readonly int[] _colCnt = new int[Colors];

    static int Idx(int i, int j) => j * GN + i;

    public static void Init()
    {
        M = GN * GN;

        // --- particles -----------------------------------------------------
        var pos = new float[M * 4];
        var prev = new float[M * 4];
        float top = Width * 0.5f;
        for (int j = 0; j < GN; j++)
            for (int i = 0; i < GN; i++)
            {
                int k = Idx(i, j);
                float x = (i / (float)(GN - 1) - 0.5f) * Width;
                float y = top - j / (float)(GN - 1) * Width;
                float invMass = (j == 0) ? 0f : 1f; // pin the whole top edge (banner)
                pos[k * 4] = x; pos[k * 4 + 1] = y; pos[k * 4 + 2] = 0f; pos[k * 4 + 3] = invMass;
                prev[k * 4] = x; prev[k * 4 + 1] = y; prev[k * 4 + 2] = 0f; prev[k * 4 + 3] = 0f;
            }

        // --- constraints, grouped by colour --------------------------------
        // Analytic graph coloring on the grid — no two constraints in a colour
        // share a particle, so each colour is one race-free dispatch:
        //   colours 0,1:   horizontal structural, by i&1
        //   colours 2,3:   vertical   structural, by j&1
        //   colours 4,5:   '\' shear, by i&1
        //   colours 6,7:   '/' shear, by i&1
        //   colours 8,9:   horizontal bend (i,j)-(i+2,j), by (i>>1)&1
        //   colours 10,11: vertical   bend (i,j)-(i,j+2), by (j>>1)&1
        // Bend colouring: two H-bend constraints conflict only when their start
        // columns differ by exactly 2 (they share the middle/end particle), and
        // (i>>1)&1 flips every 2 columns, so conflicting pairs always land in
        // different colours. Constraints 4+ columns apart share nothing.
        var buckets = new List<(int a, int b, float rest)>[Colors];
        for (int c = 0; c < Colors; c++) buckets[c] = new List<(int, int, float)>();

        Vector3 P(int i, int j) => new(pos[Idx(i, j) * 4], pos[Idx(i, j) * 4 + 1], pos[Idx(i, j) * 4 + 2]);
        void AddCon(int colour, int i0, int j0, int i1, int j1)
        {
            int a = Idx(i0, j0), b = Idx(i1, j1);
            buckets[colour].Add((a, b, Vector3.Distance(P(i0, j0), P(i1, j1))));
        }

        for (int j = 0; j < GN; j++)
            for (int i = 0; i < GN; i++)
            {
                if (i + 1 < GN) AddCon(i & 1, i, j, i + 1, j);          // H structural
                if (j + 1 < GN) AddCon(2 + (j & 1), i, j, i, j + 1);    // V structural
                if (i + 1 < GN && j + 1 < GN) AddCon(4 + (i & 1), i, j, i + 1, j + 1); // '\'
                if (i + 1 < GN && j + 1 < GN) AddCon(6 + (i & 1), i + 1, j, i, j + 1); // '/'
                if (i + 2 < GN) AddCon(8 + ((i >> 1) & 1), i, j, i + 2, j);   // H bend
                if (j + 2 < GN) AddCon(10 + ((j >> 1) & 1), i, j, i, j + 2);  // V bend
            }

        ConCount = 0;
        for (int c = 0; c < Colors; c++) ConCount += buckets[c].Count;

        // flatten into one int[] (a, b, restBits, pad) ordered by colour. The
        // compliance is *not* baked in: it arrives as a per-dispatch uniform
        // chosen from the active material, so material switches are free.
        var conData = new int[ConCount * 4];
        int w = 0;
        for (int c = 0; c < Colors; c++)
        {
            _colOff[c] = w / 4;
            _colCnt[c] = buckets[c].Count;
            foreach (var (a, b, rest) in buckets[c])
            {
                conData[w++] = a;
                conData[w++] = b;
                conData[w++] = BitConverter.SingleToInt32Bits(rest);
                conData[w++] = 0;
            }
        }

        // --- render index buffer (static; no tearing on the GPU path) ------
        var idx = new uint[(GN - 1) * (GN - 1) * 6];
        int e = 0;
        for (int j = 0; j < GN - 1; j++)
            for (int i = 0; i < GN - 1; i++)
            {
                uint a = (uint)Idx(i, j), b = (uint)Idx(i + 1, j);
                uint c = (uint)Idx(i, j + 1), d = (uint)Idx(i + 1, j + 1);
                idx[e++] = a; idx[e++] = b; idx[e++] = c;
                idx[e++] = b; idx[e++] = d; idx[e++] = c;
            }
        IdxCount = idx.Length;

        // --- GPU buffers ---------------------------------------------------
        _posBuf = MakeSsbo(pos);
        _prevBuf = MakeSsbo(prev);
        _lamBuf = MakeSsboF(ConCount);                 // zero-initialised lambda
        _conBuf = MakeSsboI(conData);
        _vtxBuf = MakeSsboEmpty(M * 8 * sizeof(float)); // pos3+nrm3+uv2, written by build pass

        // render VAO pulls vertices from _vtxBuf
        _vao = 0; GL.glGenVertexArrays(1, ref _vao); GL.glBindVertexArray(_vao);
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, _vtxBuf);
        int stride = 8 * sizeof(float);
        GL.glVertexAttribPointer(0, 3, GL.GL_FLOAT, 0, stride, 0); GL.glEnableVertexAttribArray(0);
        GL.glVertexAttribPointer(1, 3, GL.GL_FLOAT, 0, stride, 3 * sizeof(float)); GL.glEnableVertexAttribArray(1);
        GL.glVertexAttribPointer(2, 2, GL.GL_FLOAT, 0, stride, 6 * sizeof(float)); GL.glEnableVertexAttribArray(2);
        _ebo = 0; GL.glGenBuffers(1, ref _ebo);
        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, _ebo);
        UploadU(GL.GL_ELEMENT_ARRAY_BUFFER, idx);

        // --- compute programs ----------------------------------------------
        _predict = BuildCompute(PredictCS);
        _clear = BuildCompute(ClearCS);
        _solve = BuildCompute(SolveCS);
        _floor = BuildCompute(FloorCS);
        _build = BuildCompute(BuildCS);
    }

    static uint Groups(int n) => (uint)((n + Local - 1) / Local);

    public static void Step(float time, float windScale, int windOn, int aeroOn,
                            float structComp, float shearComp, float bendComp)
    {
        GL.glBindBufferBase(GL.GL_SHADER_STORAGE_BUFFER, 0, _posBuf);
        GL.glBindBufferBase(GL.GL_SHADER_STORAGE_BUFFER, 1, _prevBuf);
        GL.glBindBufferBase(GL.GL_SHADER_STORAGE_BUFFER, 2, _lamBuf);
        GL.glBindBufferBase(GL.GL_SHADER_STORAGE_BUFFER, 3, _conBuf);
        GL.glBindBufferBase(GL.GL_SHADER_STORAGE_BUFFER, 4, _vtxBuf);

        // colour -> compliance for the active material: 0-3 structural,
        // 4-7 shear, 8-11 bend (see the colouring table in Init).
        float CompForColor(int c) => c < 4 ? structComp : c < 8 ? shearComp : bendComp;

        for (int s = 0; s < SubSteps; s++)
        {
            // predict
            GL.glUseProgram(_predict);
            Set1i(_predict, "uCount", M);
            Set1i(_predict, "uGN", GN);
            Set1f(_predict, "uDt", Dt);
            Set1f(_predict, "uTime", time);
            Set1f(_predict, "uDamp", Damp);
            Set1f(_predict, "uGravity", Gravity);
            Set3f(_predict, "uWindDir", 0.9428f, 0.0471f, 0.3300f); // normalize(1,0.05,0.35)
            Set1f(_predict, "uBaseBreeze", 5.0f);
            Set1f(_predict, "uCurlStrength", 7.0f);
            Set1f(_predict, "uNoiseFreq", 1.1f);
            Set1f(_predict, "uScrollSpeed", 0.6f);
            Set1f(_predict, "uWindScale", windScale);
            Set1i(_predict, "uWindOn", windOn);
            Set1i(_predict, "uAero", aeroOn);
            Set1f(_predict, "uAeroCoeff", AeroCoeff);
            Set1f(_predict, "uAeroMaxRel", AeroMaxRel);
            GL.glDispatchCompute(Groups(M), 1, 1);
            GL.glMemoryBarrier(GL.GL_SHADER_STORAGE_BARRIER_BIT);

            // reset lambda for this substep
            GL.glUseProgram(_clear);
            Set1i(_clear, "uCount", ConCount);
            GL.glDispatchCompute(Groups(ConCount), 1, 1);
            GL.glMemoryBarrier(GL.GL_SHADER_STORAGE_BARRIER_BIT);

            // graph-colored constraint relaxation
            GL.glUseProgram(_solve);
            Set1f(_solve, "uDt", Dt);
            for (int it = 0; it < Iters; it++)
                for (int c = 0; c < Colors; c++)
                {
                    if (_colCnt[c] == 0) continue;
                    Set1i(_solve, "uOffset", _colOff[c]);
                    Set1i(_solve, "uColorCount", _colCnt[c]);
                    Set1f(_solve, "uCompliance", CompForColor(c));
                    GL.glDispatchCompute(Groups(_colCnt[c]), 1, 1);
                    GL.glMemoryBarrier(GL.GL_SHADER_STORAGE_BARRIER_BIT);
                }

            // ground plane: project out + tangential friction (same as CPU path)
            GL.glUseProgram(_floor);
            Set1i(_floor, "uCount", M);
            Set1f(_floor, "uFloorY", FloorY + FloorEps);
            Set1f(_floor, "uFriction", FloorFriction);
            GL.glDispatchCompute(Groups(M), 1, 1);
            GL.glMemoryBarrier(GL.GL_SHADER_STORAGE_BARRIER_BIT);
        }

        // rebuild render mesh (positions + normals) from the solved positions
        GL.glUseProgram(_build);
        Set1i(_build, "uGN", GN);
        GL.glDispatchCompute(Groups(M), 1, 1);
        GL.glMemoryBarrier(GL.GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT | GL.GL_SHADER_STORAGE_BARRIER_BIT);
    }

    public static void Draw()
    {
        GL.glBindVertexArray(_vao);
        GL.glDrawElements(GL.GL_TRIANGLES, IdxCount, GL.GL_UNSIGNED_INT, IntPtr.Zero);
    }

    // ---- buffer + program helpers -----------------------------------------

    static uint MakeSsbo(float[] data)
    {
        uint b = 0; GL.glGenBuffers(1, ref b);
        GL.glBindBuffer(GL.GL_SHADER_STORAGE_BUFFER, b);
        GCHandle h = GCHandle.Alloc(data, GCHandleType.Pinned);
        try { GL.glBufferData(GL.GL_SHADER_STORAGE_BUFFER, data.Length * sizeof(float), h.AddrOfPinnedObject(), GL.GL_DYNAMIC_DRAW); }
        finally { h.Free(); }
        return b;
    }

    static uint MakeSsboI(int[] data)
    {
        uint b = 0; GL.glGenBuffers(1, ref b);
        GL.glBindBuffer(GL.GL_SHADER_STORAGE_BUFFER, b);
        GCHandle h = GCHandle.Alloc(data, GCHandleType.Pinned);
        try { GL.glBufferData(GL.GL_SHADER_STORAGE_BUFFER, data.Length * sizeof(int), h.AddrOfPinnedObject(), GL.GL_DYNAMIC_DRAW); }
        finally { h.Free(); }
        return b;
    }

    static uint MakeSsboF(int count)
    {
        uint b = 0; GL.glGenBuffers(1, ref b);
        GL.glBindBuffer(GL.GL_SHADER_STORAGE_BUFFER, b);
        GL.glBufferData(GL.GL_SHADER_STORAGE_BUFFER, count * sizeof(float), IntPtr.Zero, GL.GL_DYNAMIC_DRAW);
        return b;
    }

    static uint MakeSsboEmpty(int bytes)
    {
        uint b = 0; GL.glGenBuffers(1, ref b);
        GL.glBindBuffer(GL.GL_SHADER_STORAGE_BUFFER, b);
        GL.glBufferData(GL.GL_SHADER_STORAGE_BUFFER, bytes, IntPtr.Zero, GL.GL_DYNAMIC_DRAW);
        return b;
    }

    static void UploadU(uint target, uint[] data)
    {
        GCHandle h = GCHandle.Alloc(data, GCHandleType.Pinned);
        try { GL.glBufferData(target, data.Length * sizeof(uint), h.AddrOfPinnedObject(), GL.GL_STATIC_DRAW); }
        finally { h.Free(); }
    }

    static byte[] Ascii(string s)
    {
        var b = new byte[s.Length + 1];
        System.Text.Encoding.ASCII.GetBytes(s, 0, s.Length, b, 0);
        return b;
    }

    static void Set1i(uint p, string n, int v) => GL.glUniform1i(GL.glGetUniformLocation(p, Ascii(n)), v);
    static void Set1f(uint p, string n, float v) => GL.glUniform1f(GL.glGetUniformLocation(p, Ascii(n)), v);
    static void Set3f(uint p, string n, float a, float b, float c) => GL.glUniform3f(GL.glGetUniformLocation(p, Ascii(n)), a, b, c);

    static uint BuildCompute(string src)
    {
        uint sh = GL.glCreateShader(GL.GL_COMPUTE_SHADER);
        IntPtr str = Marshal.StringToHGlobalAnsi(src);
        try { GL.glShaderSource(sh, 1, [str], IntPtr.Zero); }
        finally { Marshal.FreeHGlobal(str); }
        GL.glCompileShader(sh);
        int ok = 0; GL.glGetShaderiv(sh, GL.GL_COMPILE_STATUS, ref ok);
        if (ok == 0)
        {
            var log = new byte[4096]; int len = 0;
            GL.glGetShaderInfoLog(sh, log.Length, ref len, log);
            throw new Exception("Compute compile error: " + System.Text.Encoding.ASCII.GetString(log, 0, len));
        }
        uint prog = GL.glCreateProgram();
        GL.glAttachShader(prog, sh);
        GL.glLinkProgram(prog);
        int linked = 0; GL.glGetProgramiv(prog, GL.GL_LINK_STATUS, ref linked);
        if (linked == 0)
        {
            var log = new byte[4096]; int len = 0;
            GL.glGetProgramInfoLog(prog, log.Length, ref len, log);
            throw new Exception("Compute link error: " + System.Text.Encoding.ASCII.GetString(log, 0, len));
        }
        GL.glDeleteShader(sh);
        return prog;
    }

    // ---- compute shaders ---------------------------------------------------

    const string NoiseGLSL = @"
float fade(float t){ return t*t*t*(t*(t*6.0-15.0)+10.0); }
float hash3(ivec3 p){
    int h = p.x*374761393 + p.y*668265263 + p.z*1442695040;
    h = (h ^ (h >> 13)) * 1274126177;
    h ^= h >> 16;
    return float(h & 0xFFFF) / 32767.5 - 1.0;
}
float vnoise(vec3 P){
    ivec3 i = ivec3(floor(P)); vec3 f = P - vec3(i);
    float u=fade(f.x), v=fade(f.y), w=fade(f.z);
    float c000=hash3(i+ivec3(0,0,0)), c100=hash3(i+ivec3(1,0,0));
    float c010=hash3(i+ivec3(0,1,0)), c110=hash3(i+ivec3(1,1,0));
    float c001=hash3(i+ivec3(0,0,1)), c101=hash3(i+ivec3(1,0,1));
    float c011=hash3(i+ivec3(0,1,1)), c111=hash3(i+ivec3(1,1,1));
    float x00=mix(c000,c100,u), x10=mix(c010,c110,u);
    float x01=mix(c001,c101,u), x11=mix(c011,c111,u);
    return mix(mix(x00,x10,v), mix(x01,x11,v), w);
}
vec3 pot(vec3 s){ return vec3(vnoise(s), vnoise(s+vec3(31.416,17.073,47.853)), vnoise(s+vec3(-19.34,83.155,-5.926))); }
vec3 curl(vec3 p){
    float e=0.12; vec3 dx=vec3(e,0,0), dy=vec3(0,e,0), dz=vec3(0,0,e);
    vec3 pxp=pot(p+dx), pxm=pot(p-dx), pyp=pot(p+dy), pym=pot(p-dy), pzp=pot(p+dz), pzm=pot(p-dz);
    float inv=1.0/(2.0*e);
    return vec3(((pyp.z-pym.z)-(pzp.y-pzm.y))*inv,
                ((pzp.x-pzm.x)-(pxp.z-pxm.z))*inv,
                ((pxp.y-pxm.y)-(pyp.x-pym.x))*inv);
}";

    static readonly string PredictCS = @"#version 430
layout(local_size_x=256) in;
layout(std430,binding=0) buffer Pos  { vec4 pos[];  };
layout(std430,binding=1) buffer Prev { vec4 prev[]; };
layout(std430,binding=4) readonly buffer Vtx { float vtx[]; }; // last frame's mesh (normals)
uniform int uCount, uGN; uniform float uDt, uTime, uDamp, uGravity;
uniform vec3 uWindDir; uniform float uBaseBreeze, uCurlStrength, uNoiseFreq, uScrollSpeed, uWindScale;
uniform int uWindOn, uAero; uniform float uAeroCoeff, uAeroMaxRel;
" + NoiseGLSL + @"
vec3 windField(vec3 P){
    if (uWindOn==0) return vec3(0.0);
    float gust = 0.55 + 0.45*sin(uTime*0.7)*sin(uTime*0.23+1.3);
    vec3 s = P*uNoiseFreq + uWindDir*(uTime*uScrollSpeed);
    return (uWindDir*uBaseBreeze + curl(s)*uCurlStrength) * (gust*uWindScale);
}
void main(){
    uint g = gl_GlobalInvocationID.x; if (g >= uint(uCount)) return;
    vec4 P = pos[g]; if (P.w == 0.0) return;       // pinned
    vec3 cur = P.xyz;
    vec3 vel = cur - prev[g].xyz;
    vec3 acc = vec3(0.0,-uGravity,0.0);
    if (uAero != 0) {
        // Aerodynamic coupling: quadratic normal pressure from the relative
        // flow (see Program.cs). The surface normal is read from the render
        // vertex buffer written by last frame's build pass — race-free, one
        // frame stale, invisible at 240 substeps/s.
        vec3 n = vec3(vtx[g*8u+3u], vtx[g*8u+4u], vtx[g*8u+5u]);
        vec3 vrel = windField(cur) - vel/uDt;
        float vr = length(vrel);
        if (vr > uAeroMaxRel) vrel *= uAeroMaxRel/vr;
        float f = dot(n, vrel);
        acc += n * (uAeroCoeff * f * abs(f));
    } else {
        acc += windField(cur);
    }
    vec3 np = cur + vel*uDamp + acc*(uDt*uDt);
    pos[g]  = vec4(np, P.w);
    prev[g] = vec4(cur, 0.0);
}";

    static readonly string ClearCS = @"#version 430
layout(local_size_x=256) in;
layout(std430,binding=2) buffer Lam { float lam[]; };
uniform int uCount;
void main(){ uint g=gl_GlobalInvocationID.x; if (g<uint(uCount)) lam[g]=0.0; }";

    static readonly string SolveCS = @"#version 430
layout(local_size_x=256) in;
layout(std430,binding=0) buffer Pos { vec4 pos[]; };
layout(std430,binding=2) buffer Lam { float lam[]; };
struct Con { ivec2 ab; float rest; float pad; };
layout(std430,binding=3) buffer Cons { Con cons[]; };
uniform int uOffset, uColorCount; uniform float uDt;
uniform float uCompliance; // per colour group, from the active material preset
void main(){
    uint t = gl_GlobalInvocationID.x; if (t >= uint(uColorCount)) return;
    int c = uOffset + int(t);
    Con con = cons[c];
    int a = con.ab.x, b = con.ab.y;
    vec4 PA = pos[a], PB = pos[b];
    float wa = PA.w, wb = PB.w, wsum = wa + wb; if (wsum == 0.0) return;
    vec3 d = PA.xyz - PB.xyz; float len = length(d); if (len < 1e-6) return;
    vec3 n = d/len; float C = len - con.rest;
    float at = uCompliance/(uDt*uDt);
    float dl = (-C - at*lam[c])/(wsum + at);
    lam[c] += dl;
    vec3 corr = dl*n;
    pos[a] = vec4(PA.xyz + wa*corr, PA.w);
    pos[b] = vec4(PB.xyz - wb*corr, PB.w);
}";

    // Ground plane: project particles up onto the floor and bleed off part of
    // the horizontal slide (Verlet friction: nudge prev toward pos along the
    // tangent), exactly mirroring FloorCollide() on the CPU path.
    static readonly string FloorCS = @"#version 430
layout(local_size_x=256) in;
layout(std430,binding=0) buffer Pos  { vec4 pos[];  };
layout(std430,binding=1) buffer Prev { vec4 prev[]; };
uniform int uCount; uniform float uFloorY, uFriction;
void main(){
    uint g = gl_GlobalInvocationID.x; if (g >= uint(uCount)) return;
    vec4 P = pos[g]; if (P.w == 0.0) return;       // pinned
    if (P.y >= uFloorY) return;
    pos[g] = vec4(P.x, uFloorY, P.z, P.w);
    vec3 v = pos[g].xyz - prev[g].xyz;
    prev[g].xz += v.xz * uFriction;
}";

    static readonly string BuildCS = @"#version 430
layout(local_size_x=256) in;
layout(std430,binding=0) buffer Pos { vec4 pos[]; };
layout(std430,binding=4) buffer Vtx { float vtx[]; };
uniform int uGN;
void main(){
    uint g = gl_GlobalInvocationID.x; int M = uGN*uGN; if (g >= uint(M)) return;
    int i = int(g) % uGN, j = int(g) / uGN;
    int il = max(i-1,0), ir = min(i+1,uGN-1), jl = max(j-1,0), jr = min(j+1,uGN-1);
    vec3 dxv = pos[j*uGN+ir].xyz - pos[j*uGN+il].xyz;
    vec3 dyv = pos[jr*uGN+i].xyz - pos[jl*uGN+i].xyz;
    vec3 nrm = cross(dyv, dxv); float l = length(nrm);
    nrm = (l > 1e-6) ? nrm/l : vec3(0.0,0.0,1.0);
    vec3 p = pos[g].xyz; int o = int(g)*8;
    vtx[o]=p.x; vtx[o+1]=p.y; vtx[o+2]=p.z;
    vtx[o+3]=nrm.x; vtx[o+4]=nrm.y; vtx[o+5]=nrm.z;
    vtx[o+6]=float(i)/float(uGN-1); vtx[o+7]=float(j)/float(uGN-1);
}";
}
