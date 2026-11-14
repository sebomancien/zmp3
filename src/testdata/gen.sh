#!/bin/sh
# Regenerates the MP3 decoder test vectors (`zmp3.zig` tests) with ffmpeg
# (libmp3lame encoder, `mp3float` reference decoder). Run from this folder.
#
# Each `X.mp3` comes with `X.pcm`: ffmpeg's decode of it, downmixed to mono
# as the decoder does ((L + R) / 2), 16-bit little-endian.
set -eu

enc() { # name, lavfi source, encoder options...
    name=$1 src=$2
    shift 2
    ffmpeg -nostdin -loglevel error -y -f lavfi -i "$src" -c:a libmp3lame "$@" "$name.mp3"
}

ref() { # name, channels
    mix=anull
    [ "$2" = 2 ] && mix="pan=mono|c0=0.5*c0+0.5*c1"
    ffmpeg -nostdin -loglevel error -y -c:a mp3float -i "$1.mp3" -af "$mix" -f s16le -c:a pcm_s16le "$1.pcm"
}

# Mono chirp 200 Hz -> 8 kHz, CBR 64 kbit/s.
enc mono_64k \
    "aevalsrc=0.5*sin(2*PI*(200*t+7800*t*t)):s=44100:d=0.5" \
    -ac 1 -b:a 64k -write_xing 0
ref mono_64k 1

# Different tones left/right plus clicks every 0.1 s (short blocks),
# joint stereo, CBR 128 kbit/s.
enc stereo_js_128k \
    "aevalsrc=0.3*sin(2*PI*440*t)+0.6*lt(mod(t\,0.1)\,0.002)*sin(2*PI*3000*t)|0.3*sin(2*PI*660*t)+0.2*sin(2*PI*5000*t):s=44100:d=0.5" \
    -b:a 128k -joint_stereo 1 -write_xing 0
ref stereo_js_128k 2

# Noise plus tones, stereo VBR (bit reservoir, linbits tables).
vbr_src="aevalsrc=0.25*(random(0)-0.5)+0.4*sin(2*PI*1000*t)|0.25*(random(1)-0.5)+0.4*sin(2*PI*1500*t):s=44100:d=0.5"
enc vbr_noise "$vbr_src" -q:a 2 -write_xing 0
ref vbr_noise 2
# The same with the Xing/Info frame first (no reference: decodes as
# vbr_noise after that frame).
enc vbr_noise_xing "$vbr_src" -q:a 2 -write_xing 1
