#!/usr/bin/env python3
"""Benchmark the installed metric Depth Anything V2 model on one image."""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import torch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("--sizes", default="168,196,224,256")
    parser.add_argument("--threads", default="2,4,6")
    args = parser.parse_args()
    code = os.path.expanduser("~/Depth-Anything-V2/metric_depth")
    sys.path.insert(0, code)
    from depth_anything_v2.dpt import DepthAnythingV2

    image = cv2.imread(args.image)
    if image is None:
        raise RuntimeError(f"Unable to read {args.image}")
    model = DepthAnythingV2(
        encoder="vits", features=64, out_channels=[48, 96, 192, 384],
        max_depth=80.0,
    )
    checkpoint = os.path.expanduser(
        "~/Depth-Anything-V2/metric_depth/checkpoints/"
        "depth_anything_v2_metric_vkitti_vits.pth"
    )
    model.load_state_dict(torch.load(checkpoint, map_location="cpu"))
    model.eval()
    for threads in [int(value) for value in args.threads.split(",")]:
        torch.set_num_threads(threads)
        for size in [int(value) for value in args.sizes.split(",")]:
            samples = []
            output = None
            with torch.inference_mode():
                for index in range(4):
                    started = time.perf_counter()
                    output = model.infer_image(image, size)
                    elapsed = time.perf_counter() - started
                    if index:
                        samples.append(elapsed)
            mean = float(np.mean(samples))
            print(
                f"threads={threads} size={size} latency={mean:.3f}s "
                f"rate={1.0 / mean:.2f}Hz output={output.shape}"
            )


if __name__ == "__main__":
    main()
