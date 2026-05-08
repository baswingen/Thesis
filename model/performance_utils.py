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

def get_generalization_metrics(participant_stats):
    """
    Calculate detailed generalization performance metrics across participants.
    """
    if not participant_stats:
        return None
        
    maes = [p['MAE'] for p in participant_stats]
    rmses = [p['RMSE'] for p in participant_stats]
    
    return {
        'mean_mae': np.mean(maes),
        'std_mae': np.std(maes),
        'min_mae': np.min(maes),
        'max_mae': np.max(maes),
        'mean_rmse': np.mean(rmses),
        'std_rmse': np.std(rmses),
        'min_rmse': np.min(rmses),
        'max_rmse': np.max(rmses),
    }

def calculate_anova_metrics(y_true, y_pred):
    """
    Calculate ANOVA statistics for predictions and absolute errors grouped by true weight.
    """
    from scipy import stats
    
    y_true = np.asarray(y_true)
    y_pred = np.asarray(y_pred)
    abs_err = np.abs(y_true - y_pred)
    
    unique_weights = np.sort(np.unique(y_true))
    if len(unique_weights) < 2:
        return None
        
    pred_groups = [y_pred[y_true == w] for w in unique_weights]
    err_groups = [abs_err[y_true == w] for w in unique_weights]
    
    # 1. Prediction Separability
    try:
        f_pred, p_pred = stats.f_oneway(*pred_groups)
    except Exception:
        f_pred, p_pred = float('nan'), float('nan')
    
    pred_tukey = None
    if not np.isnan(p_pred) and p_pred < 0.05:
        try:
            res = stats.tukey_hsd(*pred_groups)
            diffs = []
            n_groups = len(unique_weights)
            for i in range(n_groups):
                for j in range(i+1, n_groups):
                    p_val = res.pvalue[i, j]
                    if p_val < 0.05:
                        diff = res.statistic[i, j]
                        diffs.append({
                            'g1': unique_weights[i],
                            'g2': unique_weights[j],
                            'diff': abs(diff),
                            'p': p_val
                        })
            diffs.sort(key=lambda x: x['diff'], reverse=True)
            pred_tukey = diffs[:5]
        except Exception:
            pass

    # 2. Error Consistency
    try:
        f_err, p_err = stats.f_oneway(*err_groups)
    except Exception:
        f_err, p_err = float('nan'), float('nan')
    
    err_tukey = None
    if not np.isnan(p_err) and p_err < 0.05:
        try:
            res = stats.tukey_hsd(*err_groups)
            diffs = []
            n_groups = len(unique_weights)
            for i in range(n_groups):
                for j in range(i+1, n_groups):
                    p_val = res.pvalue[i, j]
                    if p_val < 0.05:
                        diff = res.statistic[i, j]
                        w_high = unique_weights[i] if diff > 0 else unique_weights[j]
                        w_low = unique_weights[j] if diff > 0 else unique_weights[i]
                        diffs.append({
                            'w_high': w_high,
                            'w_low': w_low,
                            'diff': abs(diff),
                            'p': p_val
                        })
            diffs.sort(key=lambda x: x['diff'], reverse=True)
            err_tukey = diffs[:5]
        except Exception:
            pass

    return {
        'pred': {'F': f_pred, 'p': p_pred, 'tukey': pred_tukey},
        'err': {'F': f_err, 'p': p_err, 'tukey': err_tukey}
    }
