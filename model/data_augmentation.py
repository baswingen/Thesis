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

    def channel_dropout(self, seq: np.ndarray) -> np.ndarray:
        """
        Randomly zero out entire channels (features) across the full sequence,
        simulating a missing or faulty sensor for the entire window.
        Each channel is zeroed independently with probability p.
        """
        p = self.config.get('channel_dropout_p', 0.10)
        # One mask value per feature, broadcast across all time steps
        mask = (np.random.rand(seq.shape[1]) > p).astype(np.float32)
        return seq * mask[np.newaxis, :]  # (1, n_features) broadcasts over time

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
        participant_ids: np.ndarray = None,
        return_pids: bool = False,
    ) -> Tuple[List[np.ndarray], np.ndarray]:
        """
        Apply augmentations to the training dataset with optional participant balancing.

        Mode 1: Standard (balance_participants=False)
            Each sample is augmented with probability `p`. Dataset grows up to 2x.

        Mode 2: Targeted Balancing (balance_participants=True)
            Every participant is brought to exactly `target_samples_per_participant`.
            If a person has too many samples, we randomly select a subset.
            If a person has too few, we generate synthetic variations to fill the gap.
            In all cases, standard augmentation (p) is still applied for diversity.
        """
        if not self.config.get('enabled', True):
            if return_pids:
                return sequences, labels, participant_ids
            return sequences, labels

        p = self.config.get('p', 0.5)
        methods = self.config.get('methods', ['noise', 'stretch', 'channel_dropout'])
        rng = np.random.default_rng()
        labels_flat = labels.flatten()
        
        # ── Setup ─────────────────────────────────────────────────────
        aug_sequences = []
        aug_labels = []
        aug_pids = [] if participant_ids is not None else None

        # Helper to apply a random subset of active augmentations
        def apply_random_aug(seq):
            active = [m for m in methods if m != 'mixup']
            if not active: return seq
            n_apply = rng.integers(1, len(active) + 1)
            chosen = rng.choice(active, size=n_apply, replace=False)
            res = seq.copy()
            for m in chosen:
                if m == 'noise': res = self.gaussian_noise(res)
                elif m == 'stretch': res = self.temporal_stretch(res)
                elif m == 'channel_dropout': res = self.channel_dropout(res)
                elif m == 'magnitude_scale': res = self.magnitude_scale(res)
            return res

        # ── Balancing Mode ────────────────────────────────────────────
        if self.config.get('balance_participants', False) and participant_ids is not None:
            target = self.config.get('target_samples_per_participant', 1500)
            
            # Group indices by participant
            from collections import defaultdict
            p_groups = defaultdict(list)
            for idx, pid in enumerate(participant_ids):
                p_groups[pid].append(idx)
            
            for pid, idxs in p_groups.items():
                # Case A: Participant has enough or too many samples
                if len(idxs) >= target:
                    selected_idxs = rng.choice(idxs, size=target, replace=False)
                    for i in selected_idxs:
                        seq = sequences[i]
                        # Apply stochastic augmentation (p)
                        if rng.random() <= p:
                            seq = apply_random_aug(seq)
                        aug_sequences.append(seq)
                        aug_labels.append(labels_flat[i])
                        aug_pids.append(pid)
                
                # Case B: Participant needs oversampling
                else:
                    # 1. Take all originals
                    for i in idxs:
                        seq = sequences[i]
                        if rng.random() <= p:
                            seq = apply_random_aug(seq)
                        aug_sequences.append(seq)
                        aug_labels.append(labels_flat[i])
                        aug_pids.append(pid)
                    
                    # 2. Fill the gap with synthetic variations
                    needed = target - len(idxs)
                    for _ in range(needed):
                        i = int(rng.choice(idxs))
                        # For fills, we FORCE augmentation to ensure diversity
                        aug_seq = apply_random_aug(sequences[i])
                        aug_sequences.append(aug_seq)
                        aug_labels.append(labels_flat[i])
                        aug_pids.append(pid)
            
            print(f"[Augmenter] Balanced {len(p_groups)} participants to {target} samples each. Total: {len(aug_sequences)}")

        # ── Standard Mode ─────────────────────────────────────────────
        else:
            # Fallback to the original stochastic logic
            aug_sequences = list(sequences)
            aug_labels = list(labels_flat)
            if participant_ids is not None:
                aug_pids = list(participant_ids)

            for i, seq in enumerate(sequences):
                if rng.random() <= p:
                    aug_seq = apply_random_aug(seq)
                    aug_sequences.append(aug_seq)
                    aug_labels.append(labels_flat[i])
                    if aug_pids is not None:
                        aug_pids.append(participant_ids[i])

        # ── MixUp (Optional Post-Pass) ───────────────────────────────
        if 'mixup' in methods and self.config.get('mixup_alpha', 0.2) > 0:
            # Simple MixUp on the newly balanced/augmented dataset
            n_curr = len(aug_sequences)
            for i in range(n_curr):
                if rng.random() <= p:
                    j = rng.integers(0, n_curr)
                    # Don't MixUp with self
                    if i == j: continue
                    mixed_seq, mixed_label = self.mixup_pair(
                        aug_sequences[i], aug_labels[i],
                        aug_sequences[j], aug_labels[j]
                    )
                    aug_sequences.append(mixed_seq)
                    aug_labels.append(mixed_label)
                    if aug_pids is not None:
                        aug_pids.append(aug_pids[i])

        aug_labels_arr = np.array(aug_labels, dtype=np.float32)
        if return_pids:
            return aug_sequences, aug_labels_arr, (np.array(aug_pids) if aug_pids is not None else None)
        return aug_sequences, aug_labels_arr
