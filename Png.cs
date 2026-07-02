// Png — a minimal, dependency-free PNG encoder for screenshots.
//
// Writes an 8-bit RGB PNG using a zlib stream made of *stored* (uncompressed)
// deflate blocks. That keeps the encoder ~100 lines with zero dependencies
// (no System.IO.Compression) at the cost of file size — perfectly fine for
// the occasional screenshot. CRC-32 (PNG chunks) and Adler-32 (zlib) are
// implemented by hand.
//
// Input is expected bottom-up (as glReadPixels returns it); rows are flipped
// while writing, and each PNG scanline is prefixed with filter type 0 (None).

using System;
using System.IO;

namespace Cloth;

internal static class Png
{
    // rgb: w*h*3 bytes, bottom-up rows (OpenGL convention), tightly packed.
    public static void WriteRgbBottomUp(string path, byte[] rgb, int w, int h)
    {
        // --- raw PNG image data: (filter byte + RGB row) per scanline, top-down
        int rowBytes = w * 3;
        var raw = new byte[(rowBytes + 1) * h];
        for (int y = 0; y < h; y++)
        {
            int src = (h - 1 - y) * rowBytes;       // flip vertically
            int dst = y * (rowBytes + 1);
            raw[dst] = 0;                           // filter: None
            Buffer.BlockCopy(rgb, src, raw, dst + 1, rowBytes);
        }

        using var f = new FileStream(path, FileMode.Create, FileAccess.Write);

        // --- PNG signature
        f.Write(new byte[] { 137, 80, 78, 71, 13, 10, 26, 10 });

        // --- IHDR: width, height, bit depth 8, color type 2 (RGB)
        var ihdr = new byte[13];
        BE(ihdr, 0, (uint)w);
        BE(ihdr, 4, (uint)h);
        ihdr[8] = 8; ihdr[9] = 2; // depth, color type; compression/filter/interlace = 0
        Chunk(f, "IHDR", ihdr);

        // --- IDAT: zlib header + stored deflate blocks + Adler-32
        Chunk(f, "IDAT", Zlib(raw));

        // --- IEND
        Chunk(f, "IEND", Array.Empty<byte>());
    }

    // zlib stream: 2-byte header, then deflate "stored" blocks (BFINAL/BTYPE=00,
    // LEN, ~LEN, raw bytes; LEN caps at 65535), then Adler-32 of the raw data.
    private static byte[] Zlib(byte[] data)
    {
        const int Max = 65535;
        int blocks = Math.Max(1, (data.Length + Max - 1) / Max);
        var outBuf = new byte[2 + blocks * 5 + data.Length + 4];
        int o = 0;
        outBuf[o++] = 0x78; outBuf[o++] = 0x01; // CMF/FLG: deflate, 32K window, no dict

        int pos = 0;
        for (int b = 0; b < blocks; b++)
        {
            int len = Math.Min(Max, data.Length - pos);
            outBuf[o++] = (byte)(b == blocks - 1 ? 1 : 0); // BFINAL, BTYPE=00 (stored)
            outBuf[o++] = (byte)(len & 0xFF);
            outBuf[o++] = (byte)(len >> 8);
            outBuf[o++] = (byte)(~len & 0xFF);
            outBuf[o++] = (byte)((~len >> 8) & 0xFF);
            Buffer.BlockCopy(data, pos, outBuf, o, len);
            o += len; pos += len;
        }

        uint adler = Adler32(data);
        BE(outBuf, o, adler);
        return outBuf;
    }

    private static void Chunk(FileStream f, string type, byte[] data)
    {
        var hdr = new byte[8];
        BE(hdr, 0, (uint)data.Length);
        for (int i = 0; i < 4; i++) hdr[4 + i] = (byte)type[i];
        f.Write(hdr, 0, 8);
        f.Write(data, 0, data.Length);

        uint crc = Crc32(hdr, 4, 4);            // CRC covers type + data
        crc = Crc32Continue(crc, data, 0, data.Length);
        var tail = new byte[4];
        BE(tail, 0, crc);
        f.Write(tail, 0, 4);
    }

    private static void BE(byte[] b, int o, uint v)
    {
        b[o] = (byte)(v >> 24); b[o + 1] = (byte)(v >> 16);
        b[o + 2] = (byte)(v >> 8); b[o + 3] = (byte)v;
    }

    // --- CRC-32 (PNG polynomial 0xEDB88320), table built once -----------------

    private static readonly uint[] CrcTable = BuildCrcTable();

    private static uint[] BuildCrcTable()
    {
        var t = new uint[256];
        for (uint n = 0; n < 256; n++)
        {
            uint c = n;
            for (int k = 0; k < 8; k++)
                c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            t[n] = c;
        }
        return t;
    }

    // Fresh CRC = continue from 0 (the zlib crc32() convention: the register is
    // re-inverted on entry, so seeding with 0 starts it at 0xFFFFFFFF).
    private static uint Crc32(byte[] data, int off, int len) =>
        Crc32Continue(0u, data, off, len);

    // Running CRC. Callers start with Crc32(...) on the first span, then feed
    // further spans here. Internally the register is kept pre-inverted.
    private static uint Crc32Continue(uint crc, byte[] data, int off, int len)
    {
        uint c = crc ^ 0xFFFFFFFFu;
        for (int i = 0; i < len; i++)
            c = CrcTable[(c ^ data[off + i]) & 0xFF] ^ (c >> 8);
        return c ^ 0xFFFFFFFFu;
    }

    private static uint Adler32(byte[] data)
    {
        const uint Mod = 65521;
        uint a = 1, b = 0;
        int i = 0, len = data.Length;
        while (len > 0)
        {
            int n = Math.Min(len, 5552); // max bytes before 32-bit overflow
            for (int k = 0; k < n; k++) { a += data[i++]; b += a; }
            a %= Mod; b %= Mod;
            len -= n;
        }
        return (b << 16) | a;
    }
}
