"""Depth Anything V2 metric inference on CPU or NVIDIA CUDA, without ROS."""

from pathlib import Path

import numpy as np


def select_device(torch, requested='auto'):
    if requested not in ('auto', 'cpu', 'cuda'):
        raise ValueError('device must be auto, cpu or cuda')
    if requested == 'cpu':
        return torch.device('cpu')
    available = torch.cuda.is_available()
    if requested == 'cuda' and not available:
        raise RuntimeError(
            'CUDA requested but unavailable. Install CUDA-enabled PyTorch and a compatible '
            'NVIDIA driver, or use device:=cpu / device:=auto. '
            'A CPU-only PyTorch installation cannot use the GPU.')
    return torch.device('cuda' if available else 'cpu')


class MetricDepth:
    def __init__(self, model_path, input_size=518, num_threads=4, device='auto'):
        import torch

        if input_size < 56 or input_size % 14 or num_threads < 1:
            raise ValueError('input_size must be a multiple of 14 >=56; threads must be positive')
        self.device = select_device(torch, device)
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation

        path = Path(model_path).expanduser().resolve()
        if not path.is_dir():
            raise ValueError('Missing model folder: {}. Run prepare_depth_model.py first.'.format(path))
        torch.set_num_threads(num_threads)
        self.torch = torch
        self.input_size = input_size
        self.processor = AutoImageProcessor.from_pretrained(str(path), local_files_only=True)
        self.model = AutoModelForDepthEstimation.from_pretrained(
            str(path), local_files_only=True, use_safetensors=True).to(self.device).eval()
        if (self.model.config.model_type != 'depth_anything'
                or self.model.config.depth_estimation_type != 'metric'):
            raise ValueError('A Depth Anything METRIC checkpoint is required; relative depth is not metres')

    def predict(self, rgb):
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise ValueError('Expected uint8 RGB image with shape HxWx3')
        h, w = rgb.shape[:2]
        # Bound the longest edge to keep inference work bounded for widescreen images.
        ratio = self.input_size / max(h, w)
        size = {'height': max(14, round(h * ratio / 14) * 14),
                'width': max(14, round(w * ratio / 14) * 14)}
        inputs = self.processor(images=rgb, return_tensors='pt', size=size,
                                keep_aspect_ratio=False, ensure_multiple_of=14)
        with self.torch.inference_mode():
            inputs = {name: tensor.to(self.device) for name, tensor in inputs.items()}
            prediction = self.model(**inputs).predicted_depth
            depth = self.torch.nn.functional.interpolate(
                prediction.unsqueeze(1), size=(h, w), mode='bilinear', align_corners=False)
        # predicted_depth is metric. Do not invert or min/max normalize it.
        # This CPU transfer waits for CUDA work too, so predict() timings include it.
        return depth[0, 0].cpu().numpy().astype(np.float32)


class MetricDepthCPU(MetricDepth):
    """Keep the original explicit-CPU Python API available."""

    def __init__(self, model_path, input_size=518, num_threads=4):
        super().__init__(model_path, input_size, num_threads, device='cpu')
