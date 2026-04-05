"""
supcon_loss.py
==============
Supervised Contrastive (SupCon) loss for the CNN-LSTM weight-estimation model.

References
----------
Khosla et al. (2020) "Supervised Contrastive Learning"
https://arxiv.org/abs/2004.11362

Usage
-----
    from model.supcon_loss import SupConLoss

    supcon = SupConLoss(
        temperature=0.1,
        cross_participant_only=True,
        canonical_weights=[0.0, 0.899, 0.979, 1.966, 2.238, 2.945, 4.142, 5.922],
    )
    loss = supcon(z, weight_labels, participant_ids)   # scalar tensor
"""

import torch
import torch.nn.functional as F


class SupConLoss(torch.nn.Module):
    """
    Supervised Contrastive loss.

    Parameters
    ----------
    temperature : float
        Scaling temperature τ (default 0.1).  Lower → harder negatives.
    cross_participant_only : bool
        If True, a pair (i, j) is only counted as a positive when both have
        the same weight label AND different participant IDs.  This forces
        participant-invariant CNN embeddings.
    canonical_weights : list[float] or None
        If supplied, samples whose weight label is not within `label_tol` of
        any canonical weight are excluded from both the positive mask and the
        denominator.  This cleanly drops MixUp-blended samples (whose label
        is a fractional blend and therefore not a clean class member).
    label_tol : float
        Tolerance for `torch.isclose` when matching float weight labels.
        Default 0.01 kg.
    """

    def __init__(
        self,
        temperature: float = 0.1,
        cross_participant_only: bool = True,
        canonical_weights: list = None,
        label_tol: float = 0.01,
    ):
        super().__init__()
        self.temperature = temperature
        self.cross_participant_only = cross_participant_only
        self.label_tol = label_tol

        if canonical_weights is not None:
            self.register_buffer(
                "canonical_weights",
                torch.tensor(canonical_weights, dtype=torch.float32),
            )
        else:
            self.canonical_weights = None

    # ------------------------------------------------------------------

    def _canonical_mask(self, labels: torch.Tensor) -> torch.Tensor:
        """
        Returns a bool mask (N,) that is True where the label is close to any
        canonical weight, i.e. the sample is NOT a MixUp blend.
        """
        if self.canonical_weights is None:
            return torch.ones(len(labels), dtype=torch.bool, device=labels.device)

        cw = self.canonical_weights.to(labels.device)  # (K,)
        # labels: (N,) → expand to (N, K); cw: (K,) → broadcast
        diffs = (labels.unsqueeze(1) - cw.unsqueeze(0)).abs()  # (N, K)
        return diffs.min(dim=1).values <= self.label_tol          # (N,)

    # ------------------------------------------------------------------

    def forward(
        self,
        z: torch.Tensor,
        weight_labels: torch.Tensor,
        participant_ids: torch.Tensor = None,
    ) -> torch.Tensor:
        """
        Compute the SupCon loss.

        Parameters
        ----------
        z : Tensor, shape (B, D)
            L2-normalised projection embeddings.
        weight_labels : Tensor, shape (B,)
            Float weight in kg for each sample.
        participant_ids : Tensor of str / int, shape (B,), optional
            Participant identifier for each sample.  Required when
            ``cross_participant_only=True``.

        Returns
        -------
        Scalar tensor.  Returns ``torch.tensor(0.0)`` if no valid anchor
        exists in the batch (e.g. all samples are the same participant when
        cross_participant_only=True).
        """
        device = z.device
        B = z.size(0)

        # 1. Identify which samples are clean (non-MixUp) canonical-weight samples
        canonical_mask = self._canonical_mask(weight_labels)  # (B,)

        if canonical_mask.sum() < 2:
            return torch.tensor(0.0, device=device, requires_grad=True)

        # Only keep canonical samples as anchors and reference set
        idx = canonical_mask.nonzero(as_tuple=True)[0]  # (M,)
        z_c      = z[idx]                               # (M, D) — already L2 normalised
        labels_c = weight_labels[idx]                   # (M,)

        M = z_c.size(0)

        # 2. Build positive mask (M × M)
        # Positive if same weight label (within tolerance)
        same_weight = (labels_c.unsqueeze(0) - labels_c.unsqueeze(1)).abs() <= self.label_tol  # (M, M)

        if self.cross_participant_only and participant_ids is not None:
            # participant_ids may be a plain Python list — index with int list
            idx_list = idx.tolist()
            if isinstance(participant_ids, (list, tuple)):
                pids_c = [participant_ids[i] for i in idx_list]
            else:
                pids_c = [participant_ids[i] for i in idx_list]
            # Build (M, M) bool tensor: pids_c[i] != pids_c[j]
            diff_participant = torch.tensor(
                [[pids_c[i] != pids_c[j] for j in range(M)] for i in range(M)],
                dtype=torch.bool, device=device,
            )
            pos_mask = same_weight & diff_participant
        else:
            pos_mask = same_weight

        # Remove diagonal (self-pairs)
        diag = torch.eye(M, dtype=torch.bool, device=device)
        pos_mask = pos_mask & ~diag  # (M, M)

        # Drop anchors with no valid positive
        has_pos = pos_mask.any(dim=1)  # (M,)
        if has_pos.sum() == 0:
            return torch.tensor(0.0, device=device, requires_grad=True)

        z_a  = z_c[has_pos]      # (A, D)
        pm   = pos_mask[has_pos]  # (A, M)

        A = z_a.size(0)

        # 3. Similarity matrix (A × M), scaled by temperature
        sim = torch.mm(z_a, z_c.T) / self.temperature  # (A, M)

        # 4. Numerically stable log-denominator over all non-self entries.
        #    Instead of masking self to -inf (which can propagate NaN when all
        #    other similarities are also very negative), subtract the self-sim
        #    contribution in log-space.
        anchor_idx_in_c = has_pos.nonzero(as_tuple=True)[0]  # (A,)
        self_sim = sim[torch.arange(A, device=device), anchor_idx_in_c]  # (A,)

        # log Σ_{j≠i} exp(sim_ij) = log( Σ_j exp(sim_ij) - exp(sim_ii) )
        log_sum_all  = torch.logsumexp(sim, dim=1)                         # (A,)
        # Numerically safe subtraction in log-space
        log_denom = torch.log(
            (log_sum_all.exp() - self_sim.exp()).clamp(min=1e-9)
        )  # (A,)

        # 5. Mean log-prob over positives for each anchor
        log_prob = sim - log_denom.unsqueeze(1)  # (A, M)

        # Zero out self-position contribution
        log_prob[torch.arange(A, device=device), anchor_idx_in_c] = 0.0

        # Average over positives for each anchor
        n_pos = pm.float().sum(dim=1).clamp(min=1)       # (A,)
        loss_per_anchor = -(log_prob * pm.float()).sum(dim=1) / n_pos  # (A,)

        return loss_per_anchor.mean().clamp(min=0.0)
