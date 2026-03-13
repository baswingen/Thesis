import torch
import torch.nn as nn
import platform
import numpy as np

def count_parameters(model):
    """Count total trainable and non-trainable parameters of a PyTorch model."""
    if not isinstance(model, nn.Module):
        return 0, 0
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    total_params = sum(p.numel() for p in model.parameters())
    return total_params, trainable_params

def estimate_flops(model, input_size=None, seq_len=None):
    """
    Estimate GFLOPs for a single inference pass.
    This is a simplified estimation for common layers in this project.
    """
    if not isinstance(model, nn.Module):
        return 0.0

    total_flops = 0
    
    for module in model.modules():
        if isinstance(module, nn.Linear):
            # 2 * in * out
            total_flops += 2 * module.in_features * module.out_features
        elif isinstance(module, nn.LSTM):
            # 8 * seq_len * hidden * (hidden + input + 1)
            # We use seq_len if provided, else assume 1
            s = seq_len if seq_len else 1
            h = module.hidden_size
            i = module.input_size
            total_flops += 8 * s * h * (h + i + 1) * module.num_layers
        elif isinstance(module, nn.GRU):
            # 3 gates, each roughly 2 * h * (h + i)
            s = seq_len if seq_len else 1
            h = module.hidden_size
            i = module.input_size
            total_flops += 3 * 2 * s * h * (h + i + 1) * module.num_layers
        elif isinstance(module, nn.Conv1d):
            # 2 * in * out * kernel * out_len
            s = seq_len if seq_len else 1
            # simplified out_len as seq_len
            total_flops += 2 * module.in_channels * module.out_channels * module.kernel_size[0] * s
        elif isinstance(module, nn.TransformerEncoderLayer):
            # Approx 12 * seq_len * d_model^2
            s = seq_len if seq_len else 1
            d = module.linear1.in_features # d_model
            total_flops += 12 * s * (d ** 2)

    return total_flops / 1e9 # Convert to GFLOPs

def get_device_info():
    """Detect and return execution device information."""
    info = {
        "os": platform.system(),
        "processor": platform.processor(),
        "machine": platform.machine(),
        "torch_device": "CPU"
    }
    
    if torch.cuda.is_available():
        info["torch_device"] = f"CUDA ({torch.cuda.get_device_name(0)})"
    elif torch.backends.mps.is_available():
        info["torch_device"] = "Apple Silicon (MPS)"
    
    return info

def format_device_string(info):
    return f"{info['torch_device']} on {info['processor']} ({info['machine']})"
