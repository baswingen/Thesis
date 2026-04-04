"""
data_augmentation.py
====================
Data augmentation for sequential feature data (LSTM / GRU training).

Augmentations operate on *scaled* numpy arrays of shape (seq_len, n_features),
i.e. after StandardScaler has been applied and before the data is wrapped in
PyTorch tensors.  This means noise magnitudes are expressed on the z-score
scale, which is consistent across features.

All augmentations are configured via AUGMENTATION_CONFIG in config_model.py.
"""

import numpy as np
import sys
from pathlib import Path
from typing import List, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from model.config_model import AUGMENTATION_CONFIG


class SequenceAugmenter:
    """
    Applies stochastic augmentations to a list of feature sequences.

    Parameters
    ----------
    config : dict
        Augmentation configuration dict (from AUGMENTATION_CONFIG).
        If None, AUGMENTATION_CONFIG from config_model.py is used.
    """

    def __init__(self, config: dict = None):
        self.config = config if config is not None else AUGMENTATION_CONFIG

    # ──────────────────────────────────────────────────────────────────
    #  Individual augmentations (each operates on one (seq_len, n_feat) array)
    # ──────────────────────────────────────────────────────────────────

    def gaussian_noise(self, seq: np.ndarray) -> np.ndarray:
        """
        Add zero-mean Gaussian noise to every feature at every time step.
        std is expressed on the standardised (z-score) scale.
        """
        std = self.config.get('noise_std', 0.05)
        noise = np.random.randn(*seq.shape).astype(np.float32) * std
        return seq + noise

    def temporal_stretch(self, seq: np.ndarray) -> np.ndarray:
        """
        Resample the sequence along the time axis by a random factor,
        simulating different contraction / lifting speeds.

        The output is interpolated back to the original seq_len so that
        batch collation still works normally.
        """
        lo, hi = self.config.get('stretch_factor_range', (0.85, 1.15))
        factor = np.random.uniform(lo, hi)
        orig_len = seq.shape[0]
        new_len = max(2, int(round(orig_len * factor)))

        # Interpolate each feature independently along the time axis
        old_indices = np.linspace(0, orig_len - 1, orig_len)
        new_indices = np.linspace(0, orig_len - 1, new_len)
        stretched = np.stack(
            [np.interp(new_indices, old_indices, seq[:, f]) for f in range(seq.shape[1])],
            axis=1
        ).astype(np.float32)

        # Resample back to original length so all sequences remain the same length
        back_indices = np.linspace(0, new_len - 1, orig_len)
        resampled = np.stack(
            [np.interp(back_indices, np.arange(new_len), stretched[:, f]) for f in range(stretched.shape[1])],
            axis=1
        ).astype(np.float32)

        return resampled

    def feature_dropout(self, seq: np.ndarray) -> np.ndarray:
        """
        Randomly zero out individual feature values across the sequence,
        simulating missing or noisy sensor channels for a given window.
        Each feature value is zeroed independently with probability p.
        """
        p = self.config.get('feature_dropout_p', 0.10)
        mask = (np.random.rand(*seq.shape) > p).astype(np.float32)
        return seq * mask

    def magnitude_scale(self, seq: np.ndarray) -> np.ndarray:
        """
        Scale the entire sequence by a random per-feature multiplicative factor.
        Safe for amplitude-like features; apply conservatively.
        """
        lo, hi = self.config.get('magnitude_scale_range', (0.85, 1.15))
        # One scale factor per feature, broadcast across time
        factors = np.random.uniform(lo, hi, size=(1, seq.shape[1])).astype(np.float32)
        return seq * factors

    # ──────────────────────────────────────────────────────────────────
    #  MixUp: blends two sequence-label pairs
    # ──────────────────────────────────────────────────────────────────

    def mixup_pair(
        self,
        seq_a: np.ndarray,
        label_a: float,
        seq_b: np.ndarray,
        label_b: float,
    ) -> Tuple[np.ndarray, float]:
        """
        Linearly interpolate two sequence-label pairs:
            x̃ = λ·xₐ + (1-λ)·x_b
            ỹ = λ·yₐ + (1-λ)·y_b
        λ is drawn from a Beta(α, α) distribution.

        Both sequences must have the same shape; temporal stretching is applied
        to seq_b if lengths differ to make them compatible.
        """
        alpha = self.config.get('mixup_alpha', 0.2)
        if alpha <= 0:
            # If alpha is invalid, fallback to no blending (lam=1.0)
            lam = 1.0
        else:
            lam = np.random.beta(alpha, alpha)

        # Align lengths if needed
        if seq_a.shape[0] != seq_b.shape[0]:
            target_len = seq_a.shape[0]
            old_len = seq_b.shape[0]
            idx_old = np.arange(old_len)
            idx_new = np.linspace(0, old_len - 1, target_len)
            seq_b = np.stack(
                [np.interp(idx_new, idx_old, seq_b[:, f]) for f in range(seq_b.shape[1])],
                axis=1
            ).astype(np.float32)

        mixed_seq = (lam * seq_a + (1 - lam) * seq_b).astype(np.float32)
        mixed_label = lam * label_a + (1 - lam) * label_b
        return mixed_seq, mixed_label

    # ──────────────────────────────────────────────────────────────────
    #  Dataset-level augmentation
    # ──────────────────────────────────────────────────────────────────

    def augment_dataset(
        self,
        sequences: List[np.ndarray],
        labels: np.ndarray,
    ) -> Tuple[List[np.ndarray], np.ndarray]:
        """
        Apply augmentations to the training dataset.

        For each original sample, a new augmented copy is created with
        probability `p` and *appended* to the dataset (original is kept).
        This means the dataset can grow up to 2× its original size.

        MixUp is handled separately: it picks a random partner for each
        selected sample and creates a blended copy.

        Parameters
        ----------
        sequences : list of np.ndarray, shape (seq_len, n_features)
            Scaled training sequences.
        labels : np.ndarray, shape (N,) or (N, 1)
            Corresponding target values.

        Returns
        -------
        aug_sequences : list of np.ndarray
        aug_labels    : np.ndarray
        """
        if not self.config.get('enabled', True):
            return sequences, labels

        p = self.config.get('p', 0.5)
        methods = self.config.get('methods', ['noise', 'stretch', 'feature_dropout'])
        rng = np.random.default_rng()  # fresh RNG each call

        labels_flat = labels.flatten()

        aug_sequences = list(sequences)           # start with originals
        aug_labels = list(labels_flat)

        for i, seq in enumerate(sequences):
            if rng.random() > p:
                continue  # skip this sample

            # Pick a random subset of active methods for variety
            active = [m for m in methods if m != 'mixup']
            if len(active) > 1:
                n_apply = rng.integers(1, len(active) + 1)
                chosen = list(rng.choice(active, size=n_apply, replace=False))
            else:
                chosen = active

            aug_seq = seq.copy()

            for method in chosen:
                if method == 'noise':
                    aug_seq = self.gaussian_noise(aug_seq)
                elif method == 'stretch':
                    aug_seq = self.temporal_stretch(aug_seq)
                elif method == 'feature_dropout':
                    aug_seq = self.feature_dropout(aug_seq)
                elif method == 'magnitude_scale':
                    aug_seq = self.magnitude_scale(aug_seq)

            aug_sequences.append(aug_seq)
            aug_labels.append(labels_flat[i])

        # ── MixUp pass (if enabled and alpha > 0) ────────────────────
        if 'mixup' in methods and self.config.get('mixup_alpha', 0.2) > 0:
            n_orig = len(sequences)
            for i in range(n_orig):
                if rng.random() > p:
                    continue
                j = rng.integers(0, n_orig)
                mixed_seq, mixed_label = self.mixup_pair(
                    sequences[i], labels_flat[i],
                    sequences[j], labels_flat[j],
                )
                aug_sequences.append(mixed_seq)
                aug_labels.append(mixed_label)

        aug_labels_arr = np.array(aug_labels, dtype=np.float32)
        return aug_sequences, aug_labels_arr
