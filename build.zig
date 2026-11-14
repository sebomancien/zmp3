//! zmp3 package: the `zmp3` module and its unit tests.
//!
//! In a consumer's build.zig (after adding `zmp3` to build.zig.zon):
//!
//! ```zig
//! const zmp3 = b.dependency("zmp3", .{}).module("zmp3");
//! exe.root_module.addImport("zmp3", zmp3);
//! ```

const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    // Preferred mode: `--release` reaches dependencies too (a consumer's
    // `zig build --release` fails without one).
    const optimize = b.standardOptimizeOption(.{ .preferred_optimize_mode = .ReleaseSmall });

    // No target or optimize mode: the module takes the importer's (e.g. a
    // firmware's).
    _ = b.addModule("zmp3", .{ .root_source_file = b.path("src/zmp3.zig") });

    const tests = b.addTest(.{ .root_module = b.createModule(.{
        .root_source_file = b.path("src/zmp3.zig"),
        .target = target,
        .optimize = optimize,
    }) });
    b.step("test", "Run the unit tests").dependOn(&b.addRunArtifact(tests).step);
}
