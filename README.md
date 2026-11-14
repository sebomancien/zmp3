# zmp3

A small MPEG-1 Layer III (MP3) decoder in Zig: a plain port of ISO/IEC 11172-3,
std only, no allocation, `f32` math (fits a Cortex-M4F single-precision FPU).
Requires Zig 0.16.

- MPEG-1 Layer III at 32, 44.1 and 48 kHz, CBR or VBR, mono, stereo, joint
  stereo (mid/side and intensity).
- Output: 16-bit **mono** (stereo is averaged), 1152 samples per frame.
- Streaming: feed chunks of any size; frames may span calls.
- Skips a leading ID3v2 tag and a Xing/Info (LAME) first frame. Corrupt frames
  decode as silence; the decoder never panics on bad input.
- MPEG-2/2.5, Layers I/II and free format return `error.Unsupported`.
- State: ~19 KB in `Decoder` (all-zero default, so a static goes in `.bss`);
  ~23 KB of code and tables on Thumb-2 (ReleaseSmall).

## Use

`build.zig.zon`:

```zig
.dependencies = .{
    .zmp3 = .{ .path = "../zmp3" },
    // or: zig fetch --save git+https://github.com/sebomancien/zmp3#<commit>
},
```

`build.zig`:

```zig
const zmp3 = b.dependency("zmp3", .{}).module("zmp3");
exe.root_module.addImport("zmp3", zmp3);
```

Code:

```zig
const zmp3 = @import("zmp3");

var decoder: zmp3.Decoder = .{}; // static: ~19 KB
var pcm: [zmp3.samples_per_frame]i16 = undefined;

fn feed(bytes: []const u8) !void {
    var rest = bytes;
    while (rest.len > 0) {
        const r = try decoder.decode(rest, &pcm);
        rest = rest[r.consumed..];
        use(pcm[0..r.samples], decoder.sample_rate);
    }
}
```

Call `decoder.reset()` before a new stream.

## Tests

`zig build test` decodes the vectors in `src/testdata` and compares them with
ffmpeg's decoder (≥ 90 dB SNR, ≤ 2 LSB). It also checks chunking, ID3/Xing
skipping, unsupported headers, random and corrupt input, and the Huffman tables.
To regenerate the vectors (needs ffmpeg with libmp3lame), run
`cd src/testdata && ./gen.sh`.

## Sources

The constant tables (Huffman codes, scalefactor bands, synthesis window) are
those defined by ISO/IEC 11172-3 Annex B (tables 3-B.7, 3-B.8 and 3-B.3); the
others are computed at comptime from the standard's formulas.

## License

MIT, see [LICENSE](LICENSE).
