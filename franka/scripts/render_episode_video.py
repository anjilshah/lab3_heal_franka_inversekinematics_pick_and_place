#!/usr/bin/env python3
"""Render a reproducible Franka pick-and-place run to an MP4 video."""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "utils"))
from core import METHODS, load_model, run_episode  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--method", choices=METHODS, default="dls")
    parser.add_argument("--seed", type=int, default=20261013)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "submission" / "franka_nominal_dls_run.mp4")
    parser.add_argument("--fps", type=int, default=25)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise SystemExit("ffmpeg is required to encode the rendered frames")
    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite {args.output}; choose a new output path")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    model, data, *rest = load_model()
    indices = tuple(rest)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.16, 0.0, 0.44]
    camera.distance = 1.28
    camera.azimuth = 135
    camera.elevation = -24

    encode = [ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
              "-f", "rawvideo", "-pix_fmt", "rgb24", "-s:v",
              f"{args.width}x{args.height}", "-r", str(args.fps), "-i", "-",
              "-an", "-c:v", "libx264", "-preset", "medium", "-crf", "20",
              "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.output)]
    process = subprocess.Popen(encode, stdin=subprocess.PIPE)
    frame_stride = max(1, round(1 / (args.fps * model.opt.timestep)))
    frame_count = 0

    def capture(current_model, current_data, phase, step):
        nonlocal frame_count
        if step % frame_stride:
            return
        renderer.update_scene(current_data, camera=camera)
        frame = Image.fromarray(renderer.render(), mode="RGB")
        draw = ImageDraw.Draw(frame)
        draw.rectangle((0, 0, args.width, 42), fill=(18, 25, 36))
        draw.text((18, 13), f"Franka Panda | {args.method.upper()} | {phase}",
                  fill=(255, 255, 255))
        process.stdin.write(np.asarray(frame, dtype=np.uint8).tobytes())
        frame_count += 1

    try:
        result = run_episode(args.method, model, data, indices, seed=args.seed,
                             frame_callback=capture)
        if frame_count == 0:
            raise RuntimeError("No frames were rendered")
        process.stdin.close()
        code = process.wait()
        if code:
            raise RuntimeError(f"ffmpeg exited with status {code}")
    except Exception:
        if process.stdin and not process.stdin.closed:
            process.stdin.close()
        process.wait()
        args.output.unlink(missing_ok=True)
        raise
    finally:
        renderer.close()

    print(f"video={args.output} frames={frame_count} fps={args.fps} "
          f"duration_s={frame_count / args.fps:.2f}")
    print(f"episode method={result['method']} success={result['success']} "
          f"target_error_m={result['target_error_m']:.5f} "
          f"failure={result['failure_reason']}")


if __name__ == "__main__":
    main()
