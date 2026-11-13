"""Device routing tests use mocks for CUDA; they do not require a GPU or weights."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
import torch

from bebop_depth_estimation.inference import MetricDepth, MetricDepthCPU, select_device


@pytest.mark.parametrize('requested,available,expected', [
    ('auto', False, 'cpu'), ('auto', True, 'cuda'),
    ('cpu', False, 'cpu'), ('cpu', True, 'cpu'), ('cuda', True, 'cuda')])
def test_device_selection(requested, available, expected):
    with patch.object(torch.cuda, 'is_available', return_value=available):
        assert select_device(torch, requested).type == expected


def test_explicit_cpu_does_not_probe_cuda():
    with patch.object(torch.cuda, 'is_available', side_effect=AssertionError('Probed CUDA')):
        assert select_device(torch, 'cpu').type == 'cpu'


def test_required_cuda_fails_with_setup_instructions():
    with patch.object(torch.cuda, 'is_available', return_value=False):
        with pytest.raises(RuntimeError, match='CUDA-enabled PyTorch'):
            select_device(torch, 'cuda')


@pytest.mark.parametrize('requested', ['gpu', 'mps', ''])
def test_invalid_device_is_rejected(requested):
    with pytest.raises(ValueError, match='auto, cpu or cuda'):
        select_device(torch, requested)


@pytest.mark.parametrize('requested,available,expected', [
    ('auto', False, 'cpu'), ('auto', True, 'cuda'), ('cpu', True, 'cpu'),
    ('cuda', True, 'cuda')])
def test_model_and_inputs_use_selected_device(tmp_path, requested, available, expected):
    # CUDA transfers are spies: actual interpolation runs on CPU in this test.
    # This verifies routing, not GPU execution or GPU performance.
    tensor = Mock()
    moved_tensor = torch.ones(1, 3, 14, 28)
    tensor.to.return_value = moved_tensor
    processor = Mock(return_value={'pixel_values': tensor})
    model = Mock()
    model.to.return_value = model
    model.eval.return_value = model
    model.config = SimpleNamespace(model_type='depth_anything', depth_estimation_type='metric')
    model.return_value = SimpleNamespace(predicted_depth=torch.full((1, 14, 28), 2.5))
    with patch.object(torch.cuda, 'is_available', return_value=available), \
            patch('transformers.AutoImageProcessor.from_pretrained', return_value=processor), \
            patch('transformers.AutoModelForDepthEstimation.from_pretrained', return_value=model) as load:
        estimator = MetricDepth(tmp_path, device=requested)
        result = estimator.predict(np.zeros((30, 60, 3), dtype=np.uint8))
    expected_device = torch.device(expected)
    model.to.assert_called_once_with(expected_device)
    tensor.to.assert_called_once_with(expected_device)
    assert model.call_args.kwargs['pixel_values'] is moved_tensor
    assert load.call_args.kwargs['local_files_only'] is True
    assert load.call_args.kwargs['use_safetensors'] is True
    assert result.shape == (30, 60)
    assert result.dtype == np.float32
    np.testing.assert_allclose(result, 2.5)


def test_legacy_cpu_api_stays_on_cpu(tmp_path):
    processor, model = Mock(), Mock()
    model.to.return_value = model
    model.eval.return_value = model
    model.config = SimpleNamespace(model_type='depth_anything', depth_estimation_type='metric')
    with patch.object(torch.cuda, 'is_available', side_effect=AssertionError('Probed CUDA')), \
            patch('transformers.AutoImageProcessor.from_pretrained', return_value=processor), \
            patch('transformers.AutoModelForDepthEstimation.from_pretrained', return_value=model):
        assert MetricDepthCPU(tmp_path).device.type == 'cpu'


def test_relative_model_is_rejected(tmp_path):
    model = Mock()
    model.to.return_value = model
    model.eval.return_value = model
    model.config = SimpleNamespace(model_type='depth_anything', depth_estimation_type='relative')
    with patch('transformers.AutoImageProcessor.from_pretrained'), \
            patch('transformers.AutoModelForDepthEstimation.from_pretrained', return_value=model):
        with pytest.raises(ValueError, match='METRIC checkpoint'):
            MetricDepth(tmp_path, device='cpu')
