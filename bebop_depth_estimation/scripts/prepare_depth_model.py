#!/usr/bin/env python3
"""Prepare an official metric checkpoint; optionally benchmark CPU/CUDA inference."""

import argparse
import json
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='Model directory to share with students')
    parser.add_argument('--scene', choices=['indoor', 'outdoor'], default='indoor')
    parser.add_argument('--revision', default='main', help='HF commit/tag; resolved commit is recorded')
    parser.add_argument('--benchmark', type=int, default=0, metavar='N', help='N timed inferences after warmup')
    parser.add_argument('--input-size', type=int, default=518)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--device', choices=['auto', 'cpu', 'cuda'], default='auto')
    parser.add_argument('--local-only', action='store_true',
                        help='Use the existing output model folder without downloading')
    args = parser.parse_args()
    if args.benchmark < 0:
        parser.error('--benchmark must be nonnegative')
    output = Path(args.output).expanduser().resolve()
    # Keep download caches beside the model, including in restricted workspaces.
    os.environ.setdefault('HF_HOME', str(output.parent / '.hf_cache'))
    if args.local_only:
        if not output.is_dir():
            parser.error('--local-only requires an existing model folder')
        print('Using local model at {}'.format(output))
    else:
        from huggingface_hub import snapshot_download
        repo_id = 'depth-anything/Depth-Anything-V2-Metric-{}-Small-hf'.format(args.scene.capitalize())
        snapshot = Path(snapshot_download(
            repo_id, revision=args.revision,
            allow_patterns=['*.json', '*.safetensors', 'README.md', 'LICENSE*', 'NOTICE*']))
        output.mkdir(parents=True, exist_ok=True)
        import shutil
        for item in snapshot.iterdir():
            if item.is_file():
                shutil.copy2(str(item), str(output / item.name))
        metadata = {'repo_id': repo_id, 'revision': snapshot.name, 'depth_units': 'metres'}
        (output / 'source.json').write_text(json.dumps(metadata, indent=2) + '\n')
        print('Model ready at {} (revision {})'.format(output, snapshot.name))
    if args.benchmark:
        import numpy as np
        from bebop_depth_estimation.inference import MetricDepth
        model = MetricDepth(output, args.input_size, args.threads, device=args.device)
        image = np.zeros((480, 856, 3), dtype=np.uint8)
        model.predict(image)
        durations = []
        for _ in range(args.benchmark):
            start = time.monotonic()
            depth = model.predict(image)
            durations.append(time.monotonic() - start)
            if depth.shape != image.shape[:2] or not np.isfinite(depth).all() or (depth <= 0).any():
                raise RuntimeError('Invalid metric depth output')
        seconds = float(np.median(durations))
        print('{} median: {:.3f} s/image ({:.2f} FPS), {} CPU threads, input edge {} px'.format(
            model.device, seconds, 1 / seconds, args.threads, args.input_size))
        print('Synthetic image benchmark measures speed and output validity, not depth accuracy.')


if __name__ == '__main__':
    main()
