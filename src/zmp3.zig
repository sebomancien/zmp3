//! zmp3: a small MPEG-1 Layer III (MP3) decoder. A plain port of ISO/IEC
//! 11172-3 in `f32` (single precision, e.g. a Cortex-M4F FPU), favouring
//! clarity over speed. Pure Zig, std only, no allocation.
//!
//! Supported: MPEG-1 Layer III at 32, 44.1 and 48 kHz, CBR or VBR, mono,
//! stereo, joint stereo (mid/side and intensity). The output is 16-bit mono:
//! stereo is averaged. MPEG-2/2.5, Layers I/II and free format are
//! `error.Unsupported`. A leading ID3v2 tag and a Xing/Info (LAME) first
//! frame are skipped.
//!
//! Streaming: feed the bitstream as it arrives, in chunks of any size (a
//! 128 kbit/s frame is 417 bytes). The decoder copies the bytes into its
//! frame buffer and decodes each frame once whole, so a frame may span any
//! number of calls. Corrupt or undecodable frames (bit
//! reservoir missing after a skip, bad side info) give a frame of silence.
//!
//! All state (~19 KB) is in `Decoder`, meant to be a static: nothing large
//! goes on the stack. Its default value is all zeros (`.bss`, no flash
//! image).
//!
//! ```zig
//! var rest = bytes;
//! while (rest.len > 0) {
//!     const r = try decoder.decode(rest, &pcm); // pcm: [1152]i16
//!     rest = rest[r.consumed..];
//!     use(pcm[0..r.samples]);
//! }
//! ```
//!
//! Decoding steps per granule (576 samples) and channel: scalefactors,
//! Huffman, requantization, stereo, short block reordering, alias
//! reduction, IMDCT with overlap-add. Then the channels are averaged and a
//! single polyphase synthesis filterbank gives the PCM.

const std = @import("std");
const tables = @import("tables.zig");

/// Samples output per decoded frame (`decode` needs room for them).
pub const samples_per_frame = 1152;
/// Largest MPEG-1 Layer III frame: 320 kbit/s at 32 kHz, padded.
pub const max_frame_len = 1441;

pub const Error = error{
    /// A valid MPEG audio header of another kind (MPEG-2/2.5, Layer I/II,
    /// free format) before any decodable frame.
    Unsupported,
};

pub const Decoder = struct {
    /// Current frame: header (or ID3 header) bytes while searching, then
    /// the whole frame once its header is found.
    frame: [max_frame_len]u8 = @splat(0),
    frame_len: u16 = 0,
    /// Length of the frame being assembled; 0 while searching for a header.
    frame_need: u16 = 0,
    /// ID3v2 bytes still to skip.
    skip: u32 = 0,
    /// Sample rate of the stream (Hz), set by its first frame; 0 before.
    /// Frames at another rate are ignored.
    sample_rate: u32 = 0,
    /// A frame was seen (Xing/Info detection on the first one).
    seen_frame: bool = false,
    /// Frames decoded since the reset (silent ones included).
    frames: u32 = 0,

    /// Bit reservoir: the main data of the previous frames (only the last
    /// 511 bytes are kept before appending a frame's).
    reservoir: [max_main_data_begin + max_frame_len]u8 = @splat(0),
    reservoir_len: u16 = 0,

    /// Scalefactors per channel: long blocks (kept from granule 0 to 1 for
    /// scfsi), short blocks [band][window].
    sf_long: [2][22]u8 = @splat(@splat(0)),
    sf_short: [2][13][3]u8 = @splat(@splat(@splat(0))),
    /// Per-granule spectrum, then time samples ([subband * 18 + slot]).
    xr: [2][576]f32 = @splat(@splat(0)),
    /// Short block reordering scratch.
    tmp: [576]f32 = @splat(0),
    /// IMDCT overlap-add, the second half of the previous granule.
    overlap: [2][32][18]f32 = @splat(@splat(@splat(0))),
    /// Polyphase synthesis FIFO (ring, newest 64 values at `v_off`).
    v: [1024]f32 = @splat(0),
    v_off: u16 = 0,

    pub const Result = struct {
        /// Bytes of `in` consumed (the rest must be passed again).
        consumed: usize,
        /// Samples written to `out`: 0 or `samples_per_frame`.
        samples: usize,
    };

    /// Forget everything: a new stream starts.
    /// Field by field: `self.* = .{}` would build a 19 KB temporary on the
    /// stack in Debug builds.
    pub fn reset(self: *Decoder) void {
        self.frame_len = 0;
        self.frame_need = 0;
        self.skip = 0;
        self.sample_rate = 0;
        self.seen_frame = false;
        self.frames = 0;
        self.reservoir_len = 0;
        for (&self.sf_long) |*s| @memset(s, 0);
        for (&self.sf_short) |*s| @memset(s, @splat(0));
        for (&self.overlap) |*o| @memset(o, @splat(0));
        @memset(&self.v, 0);
        self.v_off = 0;
    }

    /// Decode from `in` into `out`: consumes bytes up to
    /// the end of the next whole frame and outputs its samples (none for a
    /// Xing/Info frame). `consumed` is nonzero unless `in` is empty.
    pub fn decode(self: *Decoder, in: []const u8, out: []i16) Error!Result {
        std.debug.assert(out.len >= samples_per_frame);
        var used: usize = 0;
        while (used < in.len) {
            if (self.skip > 0) {
                const n: u32 = @intCast(@min(self.skip, in.len - used));
                self.skip -= n;
                used += n;
                continue;
            }
            if (self.frame_need == 0) {
                self.frame[self.frame_len] = in[used];
                self.frame_len += 1;
                used += 1;
                try self.scan();
                continue;
            }
            const n = @min(self.frame_need - self.frame_len, in.len - used);
            @memcpy(self.frame[self.frame_len..][0..n], in[used..][0..n]);
            self.frame_len += @intCast(n);
            used += n;
            if (self.frame_len == self.frame_need) {
                const samples = self.decodeFrame(out[0..samples_per_frame]);
                self.frame_len = 0;
                self.frame_need = 0;
                return .{ .consumed = used, .samples = samples };
            }
        }
        return .{ .consumed = used, .samples = 0 };
    }

    /// Header search on the bytes in `frame`: drops bytes until they start
    /// with a frame header (then sets `frame_need`) or an ID3v2 tag (then
    /// sets `skip`), or could still become one.
    fn scan(self: *Decoder) Error!void {
        while (self.frame_len > 0) {
            const b = self.frame[0..self.frame_len];
            switch (b[0]) {
                0xFF => {
                    if (b.len < 2) return;
                    if (b[1] & 0xE0 == 0xE0) {
                        if (b.len < 4) return;
                        switch (parseHeader(b[0..4].*)) {
                            .mp3 => |h| if (self.sample_rate == 0 or h.sample_rate == self.sample_rate) {
                                self.frame_need = h.frame_len;
                                return;
                            },
                            .other => if (self.frames == 0) return error.Unsupported,
                            .invalid => {},
                        }
                    }
                },
                'I' => {
                    const id3 = "ID3";
                    if (!std.mem.eql(u8, b[0..@min(b.len, 3)], id3[0..@min(b.len, 3)])) {
                        // Not "ID3": drop.
                    } else if (b.len < 10) {
                        return;
                    } else if (b[3] != 0xFF and b[4] != 0xFF and
                        (b[6] | b[7] | b[8] | b[9]) & 0x80 == 0)
                    {
                        const size = @as(u32, b[6]) << 21 | @as(u32, b[7]) << 14 |
                            @as(u32, b[8]) << 7 | b[9];
                        const footer: u32 = if (b[5] & 0x10 != 0) 10 else 0;
                        self.skip = size + footer;
                        self.frame_len = 0;
                        return;
                    }
                },
                else => {},
            }
            std.mem.copyForwards(u8, self.frame[0 .. self.frame_len - 1], self.frame[1..self.frame_len]);
            self.frame_len -= 1;
        }
    }

    /// Decodes the whole frame in `frame` into `out`. Returns the sample
    /// count (0 for a Xing/Info frame).
    fn decodeFrame(self: *Decoder, out: *[samples_per_frame]i16) usize {
        const f = self.frame[0..self.frame_len];
        const h = parseHeader(f[0..4].*).mp3;
        const nch = h.channels();
        const side_start: usize = if (h.crc) 6 else 4;
        const side_len: usize = if (nch == 1) 17 else 32;
        const main_start = side_start + side_len;
        const first = !self.seen_frame;
        self.seen_frame = true;
        self.frames +%= 1;
        if (self.sample_rate == 0) self.sample_rate = h.sample_rate;
        if (main_start > f.len) return silence(out);
        if (first and f.len >= main_start + 4 and
            (std.mem.eql(u8, f[main_start..][0..4], "Xing") or std.mem.eql(u8, f[main_start..][0..4], "Info")))
        {
            self.frames = 0;
            return 0;
        }

        var side: SideInfo = undefined;
        var sr: BitReader = .{ .data = f[side_start..main_start] };
        const side_ok = side.read(&sr, nch);

        // Bit reservoir: keep the last 511 bytes, append this frame's.
        const keep = @min(self.reservoir_len, max_main_data_begin);
        std.mem.copyForwards(u8, self.reservoir[0..keep], self.reservoir[self.reservoir_len - keep .. self.reservoir_len]);
        const main = f[main_start..];
        @memcpy(self.reservoir[keep..][0..main.len], main);
        self.reservoir_len = @intCast(keep + main.len);
        if (!side_ok or side.main_data_begin > keep) return silence(out);

        var r: BitReader = .{ .data = self.reservoir[keep - side.main_data_begin .. self.reservoir_len] };
        const sfreq: usize = h.sample_rate_index;
        for (0..2) |gr| {
            var count: [2]usize = .{ 0, 0 };
            for (0..nch) |ch| {
                const gi = &side.gr[gr][ch];
                const end = r.pos + gi.part2_3_length;
                if (gi.valid) {
                    self.readScalefactors(&r, gr, ch, gi, side.scfsi[ch]);
                    count[ch] = self.huffman(&r, ch, gi, sfreq, end);
                    self.requantize(ch, gi, sfreq, count[ch]);
                } else {
                    @memset(&self.xr[ch], 0);
                }
                r.pos = end;
            }
            if (nch == 2) {
                if (h.mode == .joint_stereo) self.jointStereo(&side.gr[gr], h.mode_ext, sfreq, count);
            }
            for (0..nch) |ch| {
                const gi = &side.gr[gr][ch];
                if (gi.block_type == 2) self.reorder(ch, gi.mixed, sfreq);
                self.antialias(ch, gi);
                self.hybrid(ch, gi);
            }
            if (nch == 2) {
                for (&self.xr[0], &self.xr[1]) |*l, rv| l.* = (l.* + rv) * 0.5;
            }
            self.synthesis(out[gr * 576 ..][0..576]);
        }
        return samples_per_frame;
    }

    /// MPEG-1 scalefactors of granule `gr`, channel `ch` (scfsi: granule 1
    /// reuses granule 0's long-block scalefactors per group of bands).
    fn readScalefactors(self: *Decoder, r: *BitReader, gr: usize, ch: usize, gi: *const GranuleInfo, scfsi: u4) void {
        const slen1 = tables.slen[gi.scalefac_compress][0];
        const slen2 = tables.slen[gi.scalefac_compress][1];
        if (gi.block_type == 2) {
            if (gi.mixed) {
                for (self.sf_long[ch][0..8]) |*s| s.* = @intCast(r.bits(slen1));
            }
            const first: usize = if (gi.mixed) 3 else 0;
            for (first..12) |sfb| {
                const n = if (sfb < 6) slen1 else slen2;
                for (&self.sf_short[ch][sfb]) |*s| s.* = @intCast(r.bits(n));
            }
            self.sf_short[ch][12] = .{ 0, 0, 0 };
        } else {
            const bounds = [5]usize{ 0, 6, 11, 16, 21 };
            for (0..4) |g| {
                if (gr == 1 and (scfsi >> @intCast(3 - g)) & 1 != 0) continue;
                const n = if (g < 2) slen1 else slen2;
                for (self.sf_long[ch][bounds[g]..bounds[g + 1]]) |*s| s.* = @intCast(r.bits(n));
            }
            self.sf_long[ch][21] = 0;
        }
    }

    /// Huffman decoding of the spectrum into `xr[ch]` (signed integers,
    /// requantized afterwards), up to bit `end`. Returns the count of lines
    /// decoded (the rest is zero).
    fn huffman(self: *Decoder, r: *BitReader, ch: usize, gi: *const GranuleInfo, sfreq: usize, end: usize) usize {
        const xr = &self.xr[ch];
        @memset(xr, 0);
        const big: usize = @as(usize, gi.big_values) * 2;
        var region1: usize = 36;
        var region2: usize = 576;
        if (!gi.window_switching) {
            const starts = &sfb_long_start[sfreq];
            region1 = starts[@min(@as(usize, gi.region0_count) + 1, 22)];
            region2 = starts[@min(@as(usize, gi.region0_count) + gi.region1_count + 2, 22)];
        }
        var i: usize = 0;
        while (i < big) : (i += 2) {
            const t = gi.table_select[if (i < region1) 0 else if (i < region2) 1 else 2];
            const tree = big_value_trees[t];
            if (tree.len == 0) continue; // table 0 (or invalid 4, 14): zeros
            const leaf = decodeTree(r, tree);
            const linbits = big_value_linbits[t];
            xr[i] = bigValue(r, leaf >> 4, linbits);
            xr[i + 1] = bigValue(r, leaf & 15, linbits);
        }
        // Count1 region: quadruples of -1/0/1 until the bits run out.
        const tree = if (gi.count1table_select) &count1_b_tree else &count1_a_tree;
        while (i + 4 <= 576 and r.pos < end) {
            const vwxy = decodeTree(r, tree) >> 4; // leaf: x = vwxy, y = 0
            var q: [4]f32 = undefined;
            for (&q, 0..) |*v, k| {
                v.* = 0;
                if (vwxy >> @intCast(3 - k) & 1 != 0) v.* = if (r.bit() == 1) -1 else 1;
            }
            if (r.pos > end) break; // the last quadruple overran: dropped
            xr[i..][0..4].* = q;
            i += 4;
        }
        return @min(i, 576);
    }

    /// Requantization: `xr = sign * |xr|^(4/3) * 2^(gain / 4)`, the gain from
    /// the global gain, subblock gain and scalefactors of the line's band.
    fn requantize(self: *Decoder, ch: usize, gi: *const GranuleInfo, sfreq: usize, count: usize) void {
        const xr = &self.xr[ch];
        const global: i32 = @as(i32, gi.global_gain) - 210;
        const shift: i32 = if (gi.scalefac_scale) 4 else 2;
        var line: usize = 0;
        if (gi.block_type != 2 or gi.mixed) {
            // Long blocks (all of them, or the first 36 lines of a mixed block).
            const nbands: usize = if (gi.block_type == 2) 8 else 22;
            for (0..nbands) |sfb| {
                const pre: i32 = if (gi.preflag) tables.pretab[sfb] else 0;
                const gain = global - shift * (@as(i32, self.sf_long[ch][sfb]) + pre);
                const width = tables.sfb_long_width[sfreq][sfb];
                for (xr[line..][0..width]) |*x| x.* = requant(x.*, gain);
                line += width;
                if (line >= count) return;
            }
        }
        if (gi.block_type != 2) return;
        const first: usize = if (gi.mixed) 3 else 0;
        for (first..13) |sfb| {
            const width = tables.sfb_short_width[sfreq][sfb];
            for (0..3) |w| {
                const gain = global - 8 * @as(i32, gi.subblock_gain[w]) -
                    shift * @as(i32, self.sf_short[ch][sfb][w]);
                for (xr[line..][0..width]) |*x| x.* = requant(x.*, gain);
                line += width;
            }
            if (line >= count) return;
        }
    }

    /// Joint stereo of a granule (before reordering): intensity stereo in
    /// the bands above the last nonzero one of the right channel, mid/side
    /// below (and where the intensity position is the illegal 7).
    fn jointStereo(self: *Decoder, gi: *const [2]GranuleInfo, mode_ext: u2, sfreq: usize, count: [2]usize) void {
        const ms = mode_ext & 2 != 0;
        const intensity = mode_ext & 1 != 0;
        if (!intensity) {
            if (ms) self.midSide(0, @max(count[0], count[1]));
            return;
        }
        const right = &gi[1];
        if (right.block_type == 2) {
            // Mid/side for the long part of a mixed block (no intensity there).
            const first: usize = if (right.mixed) 3 else 0;
            if (right.mixed and ms) self.midSide(0, 36);
            const starts = &sfb_short_start[sfreq];
            for (0..3) |w| {
                // Last band of this window with nonzero right lines.
                var is_start: usize = first;
                var sfb: usize = 13;
                while (sfb > first) {
                    sfb -= 1;
                    const width = starts[sfb + 1] - starts[sfb];
                    const lo = 3 * starts[sfb] + w * width;
                    if (!allZero(self.xr[1][lo..][0..width])) {
                        is_start = sfb + 1;
                        break;
                    }
                }
                for (first..13) |b| {
                    const width = starts[b + 1] - starts[b];
                    const lo = 3 * starts[b] + w * width;
                    if (b < is_start) {
                        if (ms) self.midSide(lo, lo + width);
                    } else {
                        self.intensityBand(lo, lo + width, self.sf_short[1][@min(b, 11)][w], ms);
                    }
                }
            }
        } else {
            const starts = &sfb_long_start[sfreq];
            var is_start: usize = 0;
            var sfb: usize = 22;
            while (sfb > 0) {
                sfb -= 1;
                if (!allZero(self.xr[1][starts[sfb]..starts[sfb + 1]])) {
                    is_start = sfb + 1;
                    break;
                }
            }
            if (ms) self.midSide(0, starts[is_start]);
            for (is_start..22) |b| {
                self.intensityBand(starts[b], starts[b + 1], self.sf_long[1][@min(b, 20)], ms);
            }
        }
    }

    fn midSide(self: *Decoder, lo: usize, hi: usize) void {
        for (self.xr[0][lo..hi], self.xr[1][lo..hi]) |*m, *s| {
            const l = (m.* + s.*) * std.math.sqrt1_2;
            s.* = (m.* - s.*) * std.math.sqrt1_2;
            m.* = l;
        }
    }

    /// Intensity stereo of lines `lo..hi` at position `pos` (0..6; 7 is
    /// illegal: mid/side or plain stereo).
    fn intensityBand(self: *Decoder, lo: usize, hi: usize, pos: u8, ms: bool) void {
        if (pos >= 7) {
            if (ms) self.midSide(lo, hi);
            return;
        }
        const k = intensity_ratio[pos];
        for (self.xr[0][lo..hi], self.xr[1][lo..hi]) |*l, *rv| {
            rv.* = l.* * k[1];
            l.* = l.* * k[0];
        }
    }

    /// Short blocks are coded band by band, window after window; the IMDCT
    /// wants the windows interleaved (line `3 * f + window`).
    fn reorder(self: *Decoder, ch: usize, mixed: bool, sfreq: usize) void {
        const xr = &self.xr[ch];
        const starts = &sfb_short_start[sfreq];
        const first: usize = if (mixed) 3 else 0;
        for (first..13) |sfb| {
            const width = starts[sfb + 1] - starts[sfb];
            const base = 3 * starts[sfb];
            for (0..3) |w| for (0..width) |j| {
                self.tmp[base + 3 * j + w] = xr[base + w * width + j];
            };
        }
        const lo = 3 * starts[first];
        @memcpy(xr[lo..], self.tmp[lo..]);
    }

    /// Alias reduction butterflies between adjacent subbands of long blocks
    /// (only between subbands 0 and 1 for mixed blocks, none for short).
    fn antialias(self: *Decoder, ch: usize, gi: *const GranuleInfo) void {
        const xr = &self.xr[ch];
        const last: usize = if (gi.block_type != 2) 32 else if (gi.mixed) 2 else return;
        for (1..last) |sb| {
            for (0..8) |i| {
                const lo = &xr[18 * sb - 1 - i];
                const hi = &xr[18 * sb + i];
                const a = lo.*;
                const b = hi.*;
                lo.* = a * alias_cs[i] - b * alias_ca[i];
                hi.* = b * alias_cs[i] + a * alias_ca[i];
            }
        }
    }

    /// IMDCT, windowing and overlap-add of each subband, then frequency
    /// inversion: `xr[ch]` becomes 18 time samples per subband.
    fn hybrid(self: *Decoder, ch: usize, gi: *const GranuleInfo) void {
        for (0..32) |sb| {
            const bt: u2 = if (gi.mixed and sb < 2) 0 else gi.block_type;
            const x = self.xr[ch][sb * 18 ..][0..18];
            var z: [36]f32 = undefined;
            if (bt == 2) imdctShort(x, &z) else imdctLong(x, &z, &imdct_window[bt]);
            const ov = &self.overlap[ch][sb];
            for (0..18) |i| {
                x[i] = z[i] + ov[i];
                ov[i] = z[18 + i];
            }
            if (sb & 1 == 1) {
                var i: usize = 1;
                while (i < 18) : (i += 2) x[i] = -x[i];
            }
        }
    }

    /// Polyphase synthesis of the 18 time slots of `xr[0]` into `out`.
    fn synthesis(self: *Decoder, out: *[576]i16) void {
        for (0..18) |t| {
            self.v_off = (self.v_off -% 64) & 1023;
            // V[i] = sum N[i][k] S[k] from X[j] = sum cos(j (2k + 1) pi / 64) S[k]:
            // V[0..16] = X[16..32], V[16] = 0, V[17..48] = -X[31..0],
            // V[48..64] = -X[0..16].
            var x: [32]f32 = undefined;
            for (&x, &synth_matrix) |*xj, *row| {
                var acc: f32 = 0;
                for (row, 0..) |c, k| acc += c * self.xr[0][k * 18 + t];
                xj.* = acc;
            }
            const v = self.v[self.v_off..][0..64];
            for (0..16) |i| v[i] = x[16 + i];
            v[16] = 0;
            for (17..48) |i| v[i] = -x[48 - i];
            for (48..64) |i| v[i] = -x[i - 48];
            for (0..32) |j| {
                var acc: f32 = 0;
                for (0..8) |i| {
                    acc += self.v[(self.v_off + i * 128 + j) & 1023] * synth_window[i * 64 + j];
                    acc += self.v[(self.v_off + i * 128 + 96 + j) & 1023] * synth_window[i * 64 + 32 + j];
                }
                out[t * 32 + j] = toPcm(acc);
            }
        }
    }
};

// Frame header

const max_main_data_begin = 511;

const Mode = enum(u2) { stereo, joint_stereo, dual_channel, mono };

const Header = struct {
    crc: bool,
    sample_rate: u32,
    sample_rate_index: u2,
    frame_len: u16,
    mode: Mode,
    mode_ext: u2,

    fn channels(h: Header) usize {
        return if (h.mode == .mono) 1 else 2;
    }
};

const HeaderKind = union(enum) {
    /// MPEG-1 Layer III.
    mp3: Header,
    /// A valid header of another kind (or free format).
    other,
    /// Not a header (reserved values).
    invalid,
};

const bitrates_kbps = [15]u16{ 0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320 };
const sample_rates = [3]u32{ 44_100, 48_000, 32_000 };

fn parseHeader(b: [4]u8) HeaderKind {
    if (b[0] != 0xFF or b[1] & 0xE0 != 0xE0) return .invalid;
    const version: u2 = @truncate(b[1] >> 3); // 3: MPEG-1, 2: MPEG-2, 0: 2.5
    const layer: u2 = @truncate(b[1] >> 1); // 1: Layer III
    const bitrate_index: u4 = @truncate(b[2] >> 4);
    const sr_index: u2 = @truncate(b[2] >> 2);
    if (version == 1 or layer == 0 or bitrate_index == 15 or sr_index == 3) return .invalid;
    if (version != 3 or layer != 1 or bitrate_index == 0) return .other;
    const padding: u16 = (b[2] >> 1) & 1;
    const sample_rate = sample_rates[sr_index];
    return .{ .mp3 = .{
        .crc = b[1] & 1 == 0,
        .sample_rate = sample_rate,
        .sample_rate_index = sr_index,
        .frame_len = @intCast(144_000 * @as(u32, bitrates_kbps[bitrate_index]) / sample_rate + padding),
        .mode = @enumFromInt(b[3] >> 6),
        .mode_ext = @truncate(b[3] >> 4),
    } };
}

// Side information

const GranuleInfo = struct {
    part2_3_length: u12,
    big_values: u9,
    global_gain: u8,
    scalefac_compress: u4,
    window_switching: bool,
    /// 0 normal, 1 start, 2 short, 3 stop.
    block_type: u2,
    mixed: bool,
    table_select: [3]u5,
    subblock_gain: [3]u3,
    region0_count: u4,
    region1_count: u3,
    preflag: bool,
    scalefac_scale: bool,
    count1table_select: bool,
    /// Decodable (else the granule is silent for this channel).
    valid: bool,
};

const SideInfo = struct {
    main_data_begin: u9,
    scfsi: [2]u4,
    gr: [2][2]GranuleInfo,

    /// Reads the MPEG-1 side info. False if it cannot be decoded at all.
    fn read(s: *SideInfo, r: *BitReader, nch: usize) bool {
        s.main_data_begin = @intCast(r.bits(9));
        _ = r.bits(if (nch == 1) 5 else 3); // private bits
        s.scfsi = .{ 0, 0 };
        for (0..nch) |ch| s.scfsi[ch] = @intCast(r.bits(4));
        for (&s.gr) |*gr| for (gr[0..nch]) |*g| {
            g.part2_3_length = @intCast(r.bits(12));
            g.big_values = @intCast(r.bits(9));
            g.global_gain = @intCast(r.bits(8));
            g.scalefac_compress = @intCast(r.bits(4));
            g.window_switching = r.bit() == 1;
            g.valid = g.big_values <= 288;
            if (g.window_switching) {
                g.block_type = @intCast(r.bits(2));
                g.mixed = r.bit() == 1;
                for (g.table_select[0..2]) |*t| t.* = @intCast(r.bits(5));
                g.table_select[2] = 0;
                for (&g.subblock_gain) |*sg| sg.* = @intCast(r.bits(3));
                if (g.block_type == 0) g.valid = false; // reserved
                g.region0_count = if (g.block_type == 2 and !g.mixed) 8 else 7;
                g.region1_count = 0; // implicit: region 1 up to line 576
            } else {
                g.block_type = 0;
                g.mixed = false;
                for (&g.table_select) |*t| t.* = @intCast(r.bits(5));
                g.subblock_gain = .{ 0, 0, 0 };
                g.region0_count = @intCast(r.bits(4));
                g.region1_count = @intCast(r.bits(3));
            }
            g.preflag = r.bit() == 1;
            g.scalefac_scale = r.bit() == 1;
            g.count1table_select = r.bit() == 1;
            // Mixed blocks keep the long-block layout in their first 2 subbands.
            if (g.block_type != 2) g.mixed = false;
        };
        return r.pos <= r.data.len * 8;
    }
};

/// MSB-first bit reader. Reading past the end gives zeros (corrupt
/// streams), and `pos` keeps counting.
const BitReader = struct {
    data: []const u8,
    pos: usize = 0,

    fn bit(r: *BitReader) u1 {
        const p = r.pos;
        r.pos += 1;
        if (p >= r.data.len * 8) return 0;
        return @truncate(r.data[p >> 3] >> @intCast(7 - (p & 7)));
    }

    fn bits(r: *BitReader, n: u5) u32 {
        var v: u32 = 0;
        for (0..n) |_| v = v << 1 | r.bit();
        return v;
    }
};

// Huffman

/// A decoding tree: node 0 is the root; each node has two children, a node
/// index or a leaf (`leaf_flag | value`).
const Node = [2]u16;
const leaf_flag: u16 = 0x8000;

/// Walks `tree` with the stream's bits; returns the leaf value.
fn decodeTree(r: *BitReader, tree: []const Node) u8 {
    var n: u16 = 0;
    while (true) {
        const next = tree[n][r.bit()];
        if (next & leaf_flag != 0) return @truncate(next);
        n = next; // children come after their parent: always terminates
    }
}

/// One big value from its Huffman part `v` (0..15): linbits extension,
/// then sign.
fn bigValue(r: *BitReader, v: u8, linbits: u4) f32 {
    var x: u32 = v;
    if (x == 15 and linbits > 0) x += r.bits(linbits);
    if (x == 0) return 0;
    const f: f32 = @floatFromInt(x);
    return if (r.bit() == 1) -f else f;
}

/// Builds the decoding tree of a code table, checking at comptime that the
/// code is prefix-free and complete.
fn buildTree(comptime codes: []const tables.Code) [codes.len - 1]Node {
    @setEvalBranchQuota(200_000);
    const empty: u16 = 0xFFFF;
    var nodes: [codes.len - 1]Node = @splat(.{ empty, empty });
    var count: u16 = 1;
    for (codes) |c| {
        var n: u16 = 0;
        for (0..c.len) |b| {
            const bit = (c.code >> @intCast(c.len - 1 - b)) & 1;
            const child = &nodes[n][bit];
            if (b == c.len - 1) {
                if (child.* != empty) @compileError("Huffman code is not prefix-free");
                child.* = leaf_flag | @as(u16, c.x) << 4 | c.y;
            } else {
                if (child.* == empty) {
                    if (count == nodes.len) @compileError("Huffman code is not complete");
                    child.* = count;
                    count += 1;
                } else if (child.* & leaf_flag != 0) {
                    @compileError("Huffman code is not prefix-free");
                }
                n = child.*;
            }
        }
    }
    for (nodes) |node| for (node) |child| if (child == empty) @compileError("Huffman code is not complete");
    return nodes;
}

/// One tree per distinct code table (tables 16..23 and 24..31 share theirs).
fn TreeOf(comptime codes: []const tables.Code) type {
    return struct {
        const tree = buildTree(codes);
    };
}

const big_value_trees = blk: {
    var t: [32][]const Node = undefined;
    for (tables.big_value_tables, 0..) |bt, i| {
        t[i] = if (bt.codes.len == 0) &.{} else &TreeOf(bt.codes).tree;
    }
    break :blk t;
};
/// Linbits by table (apart: indexing `tables.big_value_tables` at runtime
/// would put the code word lists in flash).
const big_value_linbits = blk: {
    var t: [32]u4 = undefined;
    for (&t, tables.big_value_tables) |*l, bt| l.* = bt.linbits;
    break :blk t;
};
const count1_a_tree = buildTree(&tables.tree_count1_a);
const count1_b_tree = buildTree(&tables.tree_count1_b);

// Requantization

/// |i|^(4/3) for the common small values.
const pow43_table = blk: {
    @setEvalBranchQuota(100_000);
    var t: [256]f32 = undefined;
    for (&t, 0..) |*v, i| v.* = @floatCast(std.math.pow(f64, @floatFromInt(i), 4.0 / 3.0));
    break :blk t;
};

/// 2^(i / 4), i = 0..3.
const quarter_pow2 = [4]f32{ 1.0, 1.189207115, 1.414213562, 1.681792831 };

/// `x` (a signed integer) to `sign * |x|^(4/3) * 2^(gain / 4)`.
fn requant(x: f32, gain: i32) f32 {
    if (x == 0) return 0;
    const a: u32 = @intFromFloat(@abs(x));
    const fa: f32 = @floatFromInt(a);
    const m = if (a < pow43_table.len) pow43_table[a] else fa * std.math.cbrt(fa);
    const g = std.math.ldexp(quarter_pow2[@intCast(gain & 3)], gain >> 2);
    return if (x < 0) -m * g else m * g;
}

/// Band start lines (and 576 / 192 at the end), from the band widths.
const sfb_long_start = blk: {
    var t: [3][23]u16 = undefined;
    for (&t, tables.sfb_long_width) |*s, widths| {
        s[0] = 0;
        for (widths, 0..) |w, i| s[i + 1] = s[i] + w;
        if (s[22] != 576) @compileError("long scalefactor bands do not cover 576 lines");
    }
    break :blk t;
};
const sfb_short_start = blk: {
    var t: [3][14]u16 = undefined;
    for (&t, tables.sfb_short_width) |*s, widths| {
        s[0] = 0;
        for (widths, 0..) |w, i| s[i + 1] = s[i] + w;
        if (s[13] != 192) @compileError("short scalefactor bands do not cover 192 lines");
    }
    break :blk t;
};

// Stereo

/// Intensity stereo (left, right) factors for `is_pos` 0..6:
/// tan(pos * pi / 12) / (1 + tan), 1 / (1 + tan).
const intensity_ratio = blk: {
    @setEvalBranchQuota(10_000);
    var t: [7][2]f32 = undefined;
    for (&t, 0..) |*k, pos| {
        const a = @as(f64, @floatFromInt(pos)) * std.math.pi / 12.0;
        const s = @sin(a);
        const c = @cos(a);
        k.* = .{ @floatCast(s / (s + c)), @floatCast(c / (s + c)) };
    }
    break :blk t;
};

fn allZero(xs: []const f32) bool {
    for (xs) |x| if (x != 0) return false;
    return true;
}

// Hybrid filterbank

const alias_c = [8]f64{ -0.6, -0.535, -0.33, -0.185, -0.095, -0.041, -0.0142, -0.0037 };
const alias_cs = blk: {
    var t: [8]f32 = undefined;
    for (&t, alias_c) |*v, c| v.* = @floatCast(1.0 / @sqrt(1.0 + c * c));
    break :blk t;
};
const alias_ca = blk: {
    var t: [8]f32 = undefined;
    for (&t, alias_c) |*v, c| v.* = @floatCast(c / @sqrt(1.0 + c * c));
    break :blk t;
};

/// cos(pi / 72 * (2i + 1 + 18) * (2k + 1)), i = 0..35, k = 0..17.
const imdct_long_cos = blk: {
    @setEvalBranchQuota(20_000);
    var t: [36][18]f32 = undefined;
    for (&t, 0..) |*row, i| for (row, 0..) |*v, k| {
        v.* = @floatCast(@cos(std.math.pi / 72.0 * @as(f64, @floatFromInt((2 * i + 1 + 18) * (2 * k + 1)))));
    };
    break :blk t;
};

/// cos(pi / 24 * (2i + 1 + 6) * (2k + 1)) * sin(pi / 12 * (i + 0.5)),
/// i = 0..11, k = 0..5 (window included).
const imdct_short_cos = blk: {
    @setEvalBranchQuota(10_000);
    var t: [12][6]f32 = undefined;
    for (&t, 0..) |*row, i| for (row, 0..) |*v, k| {
        const fi: f64 = @floatFromInt(i);
        v.* = @floatCast(@cos(std.math.pi / 24.0 * (2.0 * fi + 7.0) * @as(f64, @floatFromInt(2 * k + 1))) *
            @sin(std.math.pi / 12.0 * (fi + 0.5)));
    };
    break :blk t;
};

/// Long IMDCT windows by block type (0 normal, 1 start, 3 stop; 2 unused).
const imdct_window = blk: {
    @setEvalBranchQuota(10_000);
    var t: [4][36]f32 = @splat(@splat(0));
    for (0..36) |i| {
        const fi: f64 = @floatFromInt(i);
        const long = @sin(std.math.pi / 36.0 * (fi + 0.5));
        t[0][i] = @floatCast(long);
        t[1][i] = @floatCast(if (i < 18) long else if (i < 24) 1.0 else if (i < 30)
            @sin(std.math.pi / 12.0 * (fi - 18.0 + 0.5))
        else
            0.0);
        t[3][i] = @floatCast(if (i < 6) 0.0 else if (i < 12)
            @sin(std.math.pi / 12.0 * (fi - 6.0 + 0.5))
        else if (i < 18) 1.0 else long);
    }
    break :blk t;
};

fn imdctLong(x: *const [18]f32, z: *[36]f32, window: *const [36]f32) void {
    for (z, &imdct_long_cos, window) |*zi, *row, w| {
        var acc: f32 = 0;
        for (x, row) |xk, c| acc += xk * c;
        zi.* = acc * w;
    }
}

/// Three 12-point IMDCTs (windows interleaved in `x`: line 3k + w), placed
/// at 6, 12 and 18 of `z`.
fn imdctShort(x: *const [18]f32, z: *[36]f32) void {
    @memset(z, 0);
    for (0..3) |w| {
        for (&imdct_short_cos, 0..) |*row, i| {
            var acc: f32 = 0;
            for (row, 0..) |c, k| acc += x[3 * k + w] * c;
            z[6 + 6 * w + i] += acc;
        }
    }
}

// Polyphase synthesis

/// cos(j * (2k + 1) * pi / 64), j, k = 0..31: the matrixing
/// N[i][k] = cos((16 + i) * (2k + 1) * pi / 64) folded by its symmetries
/// (see `synthesis`).
const synth_matrix = blk: {
    @setEvalBranchQuota(100_000);
    var t: [32][32]f32 = undefined;
    for (&t, 0..) |*row, j| for (row, 0..) |*v, k| {
        v.* = @floatCast(@cos(@as(f64, @floatFromInt(j * (2 * k + 1))) * std.math.pi / 64.0));
    };
    break :blk t;
};

const synth_window = blk: {
    var t: [512]f32 = undefined;
    for (&t, tables.synth_window) |*v, d| v.* = @as(f32, @floatFromInt(d)) / 65536.0;
    break :blk t;
};

fn toPcm(x: f32) i16 {
    const s = @round(x * 32768.0);
    return @intFromFloat(std.math.clamp(s, -32768.0, 32767.0));
}

fn silence(out: *[samples_per_frame]i16) usize {
    @memset(out, 0);
    return samples_per_frame;
}

// Tests (vectors and references: `testdata/gen.sh`)

const testing = std.testing;

const vectors = struct {
    const mono = @embedFile("testdata/mono_64k.mp3");
    const mono_ref = @embedFile("testdata/mono_64k.pcm");
    const stereo = @embedFile("testdata/stereo_js_128k.mp3");
    const stereo_ref = @embedFile("testdata/stereo_js_128k.pcm");
    const vbr = @embedFile("testdata/vbr_noise.mp3");
    const vbr_ref = @embedFile("testdata/vbr_noise.pcm");
    const vbr_xing = @embedFile("testdata/vbr_noise_xing.mp3");
};

/// Decodes `mp3` fed in chunks of `chunk` bytes. Caller frees.
fn decodeAll(mp3: []const u8, chunk: usize) ![]i16 {
    const d = try testing.allocator.create(Decoder);
    defer testing.allocator.destroy(d);
    d.reset();
    var pcm: std.ArrayList(i16) = .empty;
    errdefer pcm.deinit(testing.allocator);
    var buf: [samples_per_frame]i16 = undefined;
    var pos: usize = 0;
    while (pos < mp3.len) {
        var rest = mp3[pos..@min(pos + chunk, mp3.len)];
        pos += rest.len;
        while (rest.len > 0) {
            const r = try d.decode(rest, &buf);
            try testing.expect(r.consumed > 0 or r.samples > 0);
            rest = rest[r.consumed..];
            try pcm.appendSlice(testing.allocator, buf[0..r.samples]);
        }
    }
    return pcm.toOwnedSlice(testing.allocator);
}

fn expectMatches(mp3: []const u8, ref_bytes: []const u8) !void {
    const out = try decodeAll(mp3, mp3.len);
    defer testing.allocator.free(out);
    const ref = std.mem.bytesAsSlice(i16, @as([]align(1) const u8, ref_bytes));
    try testing.expectEqual(ref.len, out.len);
    var signal: f64 = 0;
    var noise: f64 = 0;
    var max_diff: u32 = 0;
    for (ref, out) |a, b| {
        const fa: f64 = @floatFromInt(std.mem.littleToNative(i16, a));
        const d = fa - @as(f64, @floatFromInt(b));
        signal += fa * fa;
        noise += d * d;
        max_diff = @max(max_diff, @as(u32, @intFromFloat(@abs(d))));
    }
    const snr = 10.0 * std.math.log10(signal / @max(noise, 1e-9));
    errdefer std.debug.print("SNR {d:.1} dB, max diff {d}\n", .{ snr, max_diff });
    try testing.expect(snr >= 90.0);
    try testing.expect(max_diff <= 2);
}

test "mono CBR matches the reference decoder" {
    try expectMatches(vectors.mono, vectors.mono_ref);
}

test "joint stereo with short blocks matches the reference decoder" {
    try expectMatches(vectors.stereo, vectors.stereo_ref);
}

test "stereo VBR matches the reference decoder" {
    try expectMatches(vectors.vbr, vectors.vbr_ref);
}

test "chunking does not change the output" {
    const whole = try decodeAll(vectors.stereo, vectors.stereo.len);
    defer testing.allocator.free(whole);
    for ([_]usize{ 192, 1, 417, 1000 }) |chunk| {
        const out = try decodeAll(vectors.stereo, chunk);
        defer testing.allocator.free(out);
        try testing.expectEqualSlices(i16, whole, out);
    }
}

test "Xing/Info frame is skipped" {
    const plain = try decodeAll(vectors.vbr, 192);
    defer testing.allocator.free(plain);
    const xing = try decodeAll(vectors.vbr_xing, 192);
    defer testing.allocator.free(xing);
    try testing.expectEqualSlices(i16, plain, xing);
}

test "ID3v2 tag is skipped" {
    // The vectors start with ffmpeg's 44-byte tag ("ID3", size 0x22 + 10).
    try testing.expectEqualStrings("ID3", vectors.mono[0..3]);
    const tagged = try decodeAll(vectors.mono, 192);
    defer testing.allocator.free(tagged);
    const bare = try decodeAll(vectors.mono[44..], 192);
    defer testing.allocator.free(bare);
    try testing.expectEqualSlices(i16, tagged, bare);
    try testing.expectEqual(@as(usize, 21 * samples_per_frame), bare.len);
}

test "other MPEG audio is unsupported" {
    var buf: [samples_per_frame]i16 = undefined;
    var d: Decoder = .{};
    // MPEG-2 Layer III, 64 kbit/s, 22.05 kHz.
    try testing.expectError(error.Unsupported, d.decode(&.{ 0xFF, 0xF3, 0x80, 0xC4 }, &buf));
    d.reset();
    // MPEG-1 Layer II, 128 kbit/s, 44.1 kHz.
    try testing.expectError(error.Unsupported, d.decode(&.{ 0xFF, 0xFD, 0x90, 0x00 }, &buf));
    d.reset();
    // Free format.
    try testing.expectError(error.Unsupported, d.decode(&.{ 0xFF, 0xFB, 0x00, 0x00 }, &buf));
}

test "headers" {
    const h = parseHeader(.{ 0xFF, 0xFB, 0x90, 0x64 }).mp3; // 128k, 44.1k, joint
    try testing.expectEqual(@as(u16, 417), h.frame_len);
    try testing.expectEqual(Mode.joint_stereo, h.mode);
    try testing.expect(!h.crc);
    try testing.expectEqual(@as(u16, 418), parseHeader(.{ 0xFF, 0xFB, 0x92, 0x64 }).mp3.frame_len);
    try testing.expectEqual(@as(u16, 1441), parseHeader(.{ 0xFF, 0xFA, 0xEA, 0xC0 }).mp3.frame_len);
    try testing.expect(parseHeader(.{ 0xFF, 0xFB, 0xF0, 0x00 }) == .invalid); // bitrate 15
    try testing.expect(parseHeader(.{ 0xFF, 0xFB, 0x9C, 0x00 }) == .invalid); // rate 3
}

test "requantization" {
    try testing.expectEqual(@as(f32, 0), requant(0, 0));
    try testing.expectApproxEqRel(@as(f32, 1), requant(1, 0), 1e-6);
    try testing.expectApproxEqRel(@as(f32, -16), requant(-8, 0), 1e-6);
    try testing.expectApproxEqRel(@as(f32, 8), requant(8, -4), 1e-6);
    // Above the table: 1000^(4/3) = 10^4.
    try testing.expectApproxEqRel(@as(f32, 10_000), requant(1000, 0), 1e-5);
    try testing.expectApproxEqRel(@as(f32, 1.189207115), requant(1, 1), 1e-6);
    try testing.expectApproxEqRel(@as(f32, 0.840896415), requant(1, -1), 1e-6);
}

test "Huffman trees decode their codes" {
    // Table 1: (0,0) "1", (1,0) "01", (1,1) "000", (0,1) "001".
    var r: BitReader = .{ .data = &.{ 0b1_01_000_00, 0b1_0000000 } };
    try testing.expectEqual(@as(u8, 0x00), decodeTree(&r, big_value_trees[1]));
    try testing.expectEqual(@as(u8, 0x10), decodeTree(&r, big_value_trees[1]));
    try testing.expectEqual(@as(u8, 0x11), decodeTree(&r, big_value_trees[1]));
    try testing.expectEqual(@as(u8, 0x01), decodeTree(&r, big_value_trees[1]));
    try testing.expectEqual(@as(usize, 9), r.pos);
    // Every code word of every table decodes to its value.
    for (tables.big_value_tables, 0..) |t, i| for (t.codes) |c| {
        var bytes: [4]u8 = undefined;
        std.mem.writeInt(u32, &bytes, c.code << @intCast(32 - @as(u6, c.len)), .big);
        var br: BitReader = .{ .data = &bytes };
        errdefer std.debug.print("table {d} code {b}\n", .{ i, c.code });
        try testing.expectEqual(@as(u8, c.x) << 4 | c.y, decodeTree(&br, big_value_trees[i]));
        try testing.expectEqual(@as(usize, c.len), br.pos);
    };
}

test "count1 quadruples" {
    const d = try testing.allocator.create(Decoder);
    defer testing.allocator.destroy(d);
    d.reset();
    var gi = std.mem.zeroes(GranuleInfo);
    gi.valid = true;
    // Table A: vwxy 0001 "0101" sign -, vwxy 1000 "0111" sign +, 0000 "1".
    var r: BitReader = .{ .data = &.{ 0b0101_1_011, 0b1_0_1_00000 } };
    try testing.expectEqual(@as(usize, 12), d.huffman(&r, 0, &gi, 0, 11));
    try testing.expectEqualSlices(f32, &.{ 0, 0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0 }, d.xr[0][0..12]);
    // Table B: vwxy = 15 - code: 0110 "1001" signs +, -.
    gi.count1table_select = true;
    r = .{ .data = &.{0b1001_0_1_00} };
    try testing.expectEqual(@as(usize, 4), d.huffman(&r, 0, &gi, 0, 6));
    try testing.expectEqualSlices(f32, &.{ 0, 1, -1, 0 }, d.xr[0][0..4]);
}

test "corrupt and random input never panics" {
    var prng: std.Random.DefaultPrng = .init(0x5eed);
    const rand = prng.random();
    const d = try testing.allocator.create(Decoder);
    defer testing.allocator.destroy(d);
    var buf: [samples_per_frame]i16 = undefined;
    var junk: [4096]u8 = undefined;
    for (0..200) |round| {
        d.reset();
        // Valid stream with flipped bytes, truncated at random, or random
        // bytes with sync patterns sprinkled in.
        const src = switch (round % 3) {
            0 => vectors.stereo,
            1 => vectors.vbr,
            else => vectors.mono,
        };
        var data: []u8 = junk[0..];
        if (round % 4 == 3) {
            rand.bytes(&junk);
            var k: usize = 0;
            while (k + 4 < junk.len) : (k += 1 + rand.uintLessThan(usize, 400)) {
                junk[k] = 0xFF;
                junk[k + 1] = 0xFB;
                junk[k + 2] &= 0xEF; // valid bitrate and rate most of the time
            }
        } else {
            const n = @min(junk.len, rand.uintLessThan(usize, src.len));
            @memcpy(junk[0..n], src[src.len - n ..]);
            data = junk[0..n];
            for (0..rand.uintLessThan(usize, 40)) |_| {
                if (data.len == 0) break;
                data[rand.uintLessThan(usize, data.len)] = rand.int(u8);
            }
        }
        var rest: []const u8 = data;
        while (rest.len > 0) {
            const r = d.decode(rest, &buf) catch break;
            rest = rest[r.consumed..];
        }
    }
}
