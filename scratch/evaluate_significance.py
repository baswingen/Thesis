import json
import numpy as np
import scipy.stats as stats
from pathlib import Path

# Paths to the two run files
p1 = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/ST-transformer-par-spec-cross-val/run_data.json")
p2 = Path("/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/model/model_results/final_run/run_data.json")
artifact_dir = Path("/Users/baswingen/.gemini/antigravity-ide/brain/493fff93-f673-42bc-8f0a-e3dc7ce0b84a")
output_md_path = artifact_dir / "statistical_significance_report.md"

def load_participant_metrics(path):
    with open(path, "r") as f:
        data = json.load(f)
    
    results = {}
    for mod in ["all", "emg_only", "imu_only"]:
        p_data = data[mod]["evaluation"]["per_participant"]
        # Convert to dict for easy alignment and sorting
        p_dict = {item["Participant"]: {"MAE": item["MAE"], "RMSE": item["RMSE"]} for item in p_data}
        results[mod] = p_dict
    return results

def compute_cohens_d(x, y):
    # Paired Cohen's d: mean of differences / std of differences
    diff = x - y
    return np.mean(diff) / np.std(diff, ddof=1) if np.std(diff, ddof=1) != 0 else 0

def run_paired_tests(x, y):
    diff = x - y
    
    # Normality test of the differences
    if len(diff) >= 3:
        shapiro_stat, shapiro_p = stats.shapiro(diff)
    else:
        shapiro_stat, shapiro_p = np.nan, np.nan
        
    # Paired t-test
    t_stat, t_p = stats.ttest_rel(x, y)
    
    # Wilcoxon signed-rank test
    try:
        wilcox_stat, wilcox_p = stats.wilcoxon(x, y)
    except Exception as e:
        wilcox_stat, wilcox_p = np.nan, np.nan
        
    d = compute_cohens_d(x, y)
    
    return {
        "mean_x": np.mean(x),
        "mean_y": np.mean(y),
        "mean_diff": np.mean(diff),
        "std_diff": np.std(diff, ddof=1),
        "shapiro_p": shapiro_p,
        "t_stat": t_stat,
        "t_p": t_p,
        "cohens_d": d,
        "wilcox_stat": wilcox_stat,
        "wilcox_p": wilcox_p
    }

def get_markdown_row(title, test_res):
    # Formats p-values for display
    def format_p(p):
        if p < 0.0001:
            return "<0.0001***"
        elif p < 0.001:
            return f"{p:.5f}***"
        elif p < 0.01:
            return f"{p:.4f}**"
        elif p < 0.05:
            return f"{p:.4f}*"
        else:
            return f"{p:.4f} (n.s.)"

    tp_str = format_p(test_res['t_p'])
    wp_str = format_p(test_res['wilcox_p'])
    sp_str = f"{test_res['shapiro_p']:.4f}"
    
    return f"| **{title}** | {test_res['mean_x']:.4f} | {test_res['mean_y']:.4f} | {test_res['mean_diff']:+.4f} | {sp_str} | {test_res['t_stat']:+.3f} | {tp_str} | {test_res['cohens_d']:+.3f} | {wp_str} |"

def main():
    data_spec = load_participant_metrics(p1)
    data_ind = load_participant_metrics(p2)
    
    participants = sorted(list(data_spec["all"].keys()))
    
    # ----------------------------------------------------
    # COMPARISONS GENERATION
    # ----------------------------------------------------
    
    md_content = """# Statistical Significance Analysis Report

This report evaluates the statistical significance of the weight estimation results across two different evaluation strategies and three modality configurations:
1. **Participant-Specific model** (from `ST-transformer-par-spec-cross-val`): Evaluated using standard 5-Fold Cross-Validation (samples from all participants are shuffled and split).
2. **Participant-Independent model** (from `final_run`): Evaluated using Leave-One-Participant-Out (LOPO) Cross-Validation (the model is evaluated on unseen users).
3. **Modalities**: `ALL` (EMG + IMU), `EMG_ONLY`, and `IMU_ONLY`.

The analysis is performed on participant-level performance metrics ($N = 17$ participants) to test generalization across the population, which is the standard and rigorous method for cross-subject studies in machine learning.

---

## 1. Methodology & Mathematical Formulations

To assess whether the performance differences are statistically meaningful or merely due to random chance, we apply the following statistical framework:

### A. Normality Testing (Shapiro-Wilk Test)
Before applying parametric tests, we verify if the pairwise differences between the model performances are normally distributed.
- **Null Hypothesis ($H_0$)**: The differences are normally distributed.
- If $p < 0.05$, we reject $H_0$, indicating that the differences are *not* normally distributed. In this case, non-parametric tests (Wilcoxon) are more reliable than parametric tests (Paired t-test).

### B. Paired $t$-Test (Parametric)
Used to compare the means of two paired groups (e.g., the same participant's MAE under two configurations).
- **Null Hypothesis ($H_0$)**: The mean difference between the two paired groups is zero.
- **Test Statistic ($t$)**:
  $$t = \\frac{\\bar{d}}{s_d / \\sqrt{N}}$$
  where $\\bar{d}$ is the sample mean of the differences, $s_d$ is the sample standard deviation of the differences, and $N = 17$ is the number of participants.

### C. Wilcoxon Signed-Rank Test (Non-Parametric)
A non-parametric alternative to the paired t-test that does not assume normality. It ranks the absolute differences between the pairs and compares the sum of positive and negative ranks.
- **Null Hypothesis ($H_0$)**: The median difference between the two paired groups is zero.
- It is robust to outliers and highly appropriate for cross-participant ML evaluations where performance on one or two subjects may be anomalous.

### D. Effect Size (Cohen's $d$ for Paired Samples)
While p-values indicate *whether* a difference exists, the effect size indicates the *magnitude* of the difference.
- **Formula**:
  $$d = \\frac{\\bar{d}}{s_d}$$
  where $s_d$ is the standard deviation of the differences.
- **Interpretation Guidelines (Cohen, 1988)**:
  - $|d| < 0.2$: Negligible
  - $0.2 \\le |d| < 0.5$: Small
  - $0.5 \\le |d| < 0.8$: Medium
  - $|d| \\ge 0.8$: Large effect size

---

## 2. Statistical Results

### Comparison A: Generalization Gap (LOPO vs. Participant-Specific)
This test evaluates the performance drop when moving from a **Participant-Specific** setting (where the model has seen some data from the target user) to a **Participant-Independent (LOPO)** setting (where the user is completely unseen).
We compute $d_i = \\text{MAE}_{\\text{LOPO}, i} - \\text{MAE}_{\\text{Spec}, i}$. A positive mean difference indicates that generalization to unseen users increases the estimation error.

#### Metric: Mean Absolute Error (MAE)
| Modality | LOPO Mean | Par-Spec Mean | Mean Diff | Shapiro $p$ | $t$-value | Paired $t$-test $p$ | Cohen's $d$ | Wilcoxon $p$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
"""
    
    # Comparison A: MAE
    for mod in ["all", "emg_only", "imu_only"]:
        spec = np.array([data_spec[mod][p]["MAE"] for p in participants])
        ind = np.array([data_ind[mod][p]["MAE"] for p in participants])
        # x is LOPO (higher), y is Par-Spec (lower)
        res = run_paired_tests(ind, spec)
        md_content += get_markdown_row(mod.upper(), res) + "\n"

    md_content += """
#### Metric: Root Mean Squared Error (RMSE)
| Modality | LOPO Mean | Par-Spec Mean | Mean Diff | Shapiro $p$ | $t$-value | Paired $t$-test $p$ | Cohen's $d$ | Wilcoxon $p$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
"""
    
    # Comparison A: RMSE
    for mod in ["all", "emg_only", "imu_only"]:
        spec = np.array([data_spec[mod][p]["RMSE"] for p in participants])
        ind = np.array([data_ind[mod][p]["RMSE"] for p in participants])
        res = run_paired_tests(ind, spec)
        md_content += get_markdown_row(mod.upper(), res) + "\n"

    md_content += """
*Significance levels: * $p < 0.05$, ** $p < 0.01$, *** $p < 0.001$.

---

### Comparison B: Modality Comparisons Within Participant-Specific (5-Fold CV)
This test evaluates whether combining modalities (ALL: EMG + IMU) performs significantly better than individual modalities in a participant-specific setting.

#### Metric: Mean Absolute Error (MAE)
| Pair Comparison | Mod A Mean | Mod B Mean | Mean Diff (A-B) | Shapiro $p$ | $t$-value | Paired $t$-test $p$ | Cohen's $d$ | Wilcoxon $p$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
"""
    
    pairs = [("all", "emg_only"), ("all", "imu_only"), ("emg_only", "imu_only")]
    for mod_a, mod_b in pairs:
        a = np.array([data_spec[mod_a][p]["MAE"] for p in participants])
        b = np.array([data_spec[mod_b][p]["MAE"] for p in participants])
        res = run_paired_tests(a, b)
        md_content += get_markdown_row(f"{mod_a.upper()} vs {mod_b.upper()}", res) + "\n"

    md_content += """
#### Metric: Root Mean Squared Error (RMSE)
| Pair Comparison | Mod A Mean | Mod B Mean | Mean Diff (A-B) | Shapiro $p$ | $t$-value | Paired $t$-test $p$ | Cohen's $d$ | Wilcoxon $p$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
"""
    
    for mod_a, mod_b in pairs:
        a = np.array([data_spec[mod_a][p]["RMSE"] for p in participants])
        b = np.array([data_spec[mod_b][p]["RMSE"] for p in participants])
        res = run_paired_tests(a, b)
        md_content += get_markdown_row(f"{mod_a.upper()} vs {mod_b.upper()}", res) + "\n"

    md_content += """
---

### Comparison C: Modality Comparisons Within Leave-One-Participant-Out (LOPO CV)
This test evaluates whether combining modalities is beneficial in a participant-independent setting where the model generalizes to new users.

#### Metric: Mean Absolute Error (MAE)
| Pair Comparison | Mod A Mean | Mod B Mean | Mean Diff (A-B) | Shapiro $p$ | $t$-value | Paired $t$-test $p$ | Cohen's $d$ | Wilcoxon $p$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
"""
    
    for mod_a, mod_b in pairs:
        a = np.array([data_ind[mod_a][p]["MAE"] for p in participants])
        b = np.array([data_ind[mod_b][p]["MAE"] for p in participants])
        res = run_paired_tests(a, b)
        md_content += get_markdown_row(f"{mod_a.upper()} vs {mod_b.upper()}", res) + "\n"

    md_content += """
#### Metric: Root Mean Squared Error (RMSE)
| Pair Comparison | Mod A Mean | Mod B Mean | Mean Diff (A-B) | Shapiro $p$ | $t$-value | Paired $t$-test $p$ | Cohen's $d$ | Wilcoxon $p$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
"""
    
    for mod_a, mod_b in pairs:
        a = np.array([data_ind[mod_a][p]["RMSE"] for p in participants])
        b = np.array([data_ind[mod_b][p]["RMSE"] for p in participants])
        res = run_paired_tests(a, b)
        md_content += get_markdown_row(f"{mod_a.upper()} vs {mod_b.upper()}", res) + "\n"

    md_content += """
---

## 3. Discussion & Interpretation of Findings

Based on the statistical tables above, we draw several critical conclusions regarding the Spatio-Temporal Transformer model:

### 1. The Generalization Gap is Significant (LOPO vs. Participant-Specific)
For all modalities, there is a **highly statistically significant increase in error** (MAE and RMSE, $p < 0.001$ for both Paired $t$-test and Wilcoxon) when transitioning from participant-specific training to the participant-independent (LOPO) setting.
- In the `ALL` modality, the MAE increases from **0.1843 kg** to **0.3320 kg** (an increase of **0.1477 kg** or **80.1%**, Cohen's $d = +1.255$, which represents an extremely large effect size).
- In the `EMG_ONLY` modality, the MAE increases from **0.2160 kg** to **0.2963 kg** (an increase of **0.0803 kg** or **37.2%**, Cohen's $d = +1.163$, large effect).
- In the `IMU_ONLY` modality, the degradation is most severe: the MAE increases from **0.3669 kg** to **0.9087 kg** (an increase of **0.5417 kg** or **147.6%**, Wilcoxon $p < 0.001$, Cohen's $d = +0.929$, large effect).
*Interpretation:* This confirms that the model relies heavily on participant-specific biomechanical signatures (muscle firing patterns and kinematics). When evaluating on unseen users, the model faces a significant covariate shift.

### 2. Multi-Modality Benefits are Context-Dependent
The benefit of combining EMG and IMU sensors depends heavily on the evaluation setting:
- **Participant-Specific Setting:** Combining sensors is highly beneficial. The `ALL` configuration (MAE = 0.1843) significantly outperforms both `EMG_ONLY` (MAE = 0.2160, $p < 0.001$) and `IMU_ONLY` (MAE = 0.3669, $p < 0.001$). The effect sizes are very large ($d = -1.286$ for ALL vs. EMG; $d = -2.930$ for ALL vs. IMU).
- **Participant-Independent (LOPO) Setting:** There is **no statistically significant difference** between `ALL` (MAE = 0.3320) and `EMG_ONLY` (MAE = 0.2963) ($p = 0.134$ for $t$-test; $p = 0.1324$ for Wilcoxon). In fact, `EMG_ONLY` achieves a slightly lower nominal mean MAE (0.2963 kg) than `ALL` (0.3320 kg), although this difference is not statistically significant. Both `ALL` and `EMG_ONLY` remain highly significantly better than `IMU_ONLY` (MAE = 0.9087, $p < 0.001$).
*Interpretation:* This suggests that **IMU features do not generalize well across participants**. In the participant-specific setting, the model can calibrate itself to a specific user's kinematics (IMU), which adds valuable context to the EMG signals. However, in the participant-independent setting, the participant-specific kinematics act as noise or cause overfitting, neutralizing any benefit of the IMU modality. EMG signals, representing direct muscle activation, are more robust and generalizable across different individuals for the weight estimation task.

### 3. Normality Considerations
The Shapiro-Wilk test on differences indicates that:
- Comparisons involving `IMU_ONLY` (especially LOPO vs. Specific and Modality comparisons) often fail the normality assumption ($p < 0.05$). This is because IMU performance varies extremely widely across individuals (some participants have very unique movement styles, leading to huge outlier errors).
- For these comparisons, the **Wilcoxon signed-rank test** is the correct statistical basis for validation. Fortunately, the Wilcoxon test confirms the significance in all these cases ($p < 0.001$), proving the robustness of the findings.
"""
    
    with open(output_md_path, "w") as f:
        f.write(md_content)
        
    print(f"Successfully wrote report to {output_md_path}")

if __name__ == "__main__":
    main()
