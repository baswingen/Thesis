# Spatio-Temporal Transformer 3 Consolidated Sweep Report (FEATURES) — RECOVERED

**Recovered from log:** `run_10122647.out` (job hit 24h SLURM limit; 51 of 60 iterations completed)

**Date:** 2026-06-10 23:18:57

## 🎯 Ridge Regression Feature Contribution Analysis

* **Ridge Regression Fit $R^2$ Score:** 0.1729
* **Baseline Intercept (Average $R^2$):** 0.9635

| Rank | Feature Category | Ridge Coefficient (Delta R²) | Impact on Generalization |
| :---: | :--- | :---: | :--- |
| 1 | `IMU_Mean` | **+0.00052** | ✅ Positive (Improves Generalization) |
| 2 | `EMG_IEMG` | **+0.00044** | ✅ Positive (Improves Generalization) |
| 3 | `EMG_WL` | **+0.00042** | ✅ Positive (Improves Generalization) |
| 4 | `EMG_MAV` | **+0.00038** | ✅ Positive (Improves Generalization) |
| 5 | `EMG_LogDet` | **+0.00035** | ✅ Positive (Improves Generalization) |
| 6 | `EMG_MNF` | **+0.00035** | ✅ Positive (Improves Generalization) |
| 7 | `IMU_Var` | **+0.00029** | ✅ Positive (Improves Generalization) |
| 8 | `IMU_P2P` | **+0.00027** | ✅ Positive (Improves Generalization) |
| 9 | `EMG_BW` | **+0.00026** | ✅ Positive (Improves Generalization) |
| 10 | `EMG_Power` | **+0.00026** | ✅ Positive (Improves Generalization) |
| 11 | `EMG_HjComp` | **+0.00025** | ✅ Positive (Improves Generalization) |
| 12 | `IMU_SpecEnergy` | **+0.00024** | ✅ Positive (Improves Generalization) |
| 13 | `IMU_Std` | **+0.00021** | ✅ Positive (Improves Generalization) |
| 14 | `EMG_RMS` | **+0.00019** | ✅ Positive (Improves Generalization) |
| 15 | `EMG_VAR` | **+0.00015** | ✅ Positive (Improves Generalization) |
| 16 | `EMG_Skew` | **+0.00011** | ✅ Positive (Improves Generalization) |
| 17 | `IMU_SMA` | **+0.00001** | ✅ Positive (Improves Generalization) |
| 18 | `EMG_PeakFreq` | **-0.00003** | ❌ Negative (Reduces Generalization) |
| 19 | `EMG_SpecEntropy` | **-0.00006** | ❌ Negative (Reduces Generalization) |
| 20 | `IMU_Max` | **-0.00007** | ❌ Negative (Reduces Generalization) |
| 21 | `IMU_SVM_Mean` | **-0.00008** | ❌ Negative (Reduces Generalization) |
| 22 | `IMU_Jerk` | **-0.00017** | ❌ Negative (Reduces Generalization) |
| 23 | `EMG_ZC` | **-0.00019** | ❌ Negative (Reduces Generalization) |
| 24 | `EMG_Kurt` | **-0.00019** | ❌ Negative (Reduces Generalization) |
| 25 | `EMG_WAMP` | **-0.00021** | ❌ Negative (Reduces Generalization) |
| 26 | `IMU_Skew` | **-0.00024** | ❌ Negative (Reduces Generalization) |
| 27 | `IMU_MNF` | **-0.00025** | ❌ Negative (Reduces Generalization) |
| 28 | `EMG_MDF` | **-0.00026** | ❌ Negative (Reduces Generalization) |
| 29 | `EMG_HjMob` | **-0.00029** | ❌ Negative (Reduces Generalization) |
| 30 | `IMU_Kurt` | **-0.00030** | ❌ Negative (Reduces Generalization) |
| 31 | `EMG_SSC` | **-0.00030** | ❌ Negative (Reduces Generalization) |
| 32 | `EMG_Myopulse` | **-0.00030** | ❌ Negative (Reduces Generalization) |
| 33 | `IMU_DomFreq` | **-0.00035** | ❌ Negative (Reduces Generalization) |

## 🏆 Ranked Leaderboard (Sorted by RMSE)

| Rank | Iteration | Active Categories | Features | Val RMSE | Val MAE | Val R² | Model Params | Training Time |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| 1 | #31 | 20 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_MAV...`) | 180 | **0.3311** | 0.1604 | 0.9705 | 1,019,074 | 1599.6s |
| 2 | #33 | 14 (`EMG_BW, EMG_IEMG, EMG_Kurt, EMG_LogDe...`) | 116 | **0.3342** | 0.1403 | 0.9700 | 1,012,930 | 1600.2s |
| 3 | #49 | 16 (`EMG_BW, EMG_HjMob, EMG_LogDet, EMG_MA...`) | 136 | **0.3382** | 0.1698 | 0.9693 | 1,014,850 | 1571.6s |
| 4 | #29 | 17 (`EMG_IEMG, EMG_Kurt, EMG_MDF, EMG_MNF,...`) | 152 | **0.3398** | 0.1677 | 0.9690 | 1,016,386 | 1593.7s |
| 5 | #21 | 14 (`EMG_HjComp, EMG_HjMob, EMG_Power, EMG...`) | 116 | **0.3415** | 0.1715 | 0.9687 | 1,012,930 | 1560.0s |
| 6 | #36 | 21 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_LogD...`) | 180 | **0.3417** | 0.1935 | 0.9686 | 1,019,074 | 1568.0s |
| 7 | #40 | 18 (`EMG_LogDet, EMG_MAV, EMG_MDF, EMG_Pow...`) | 164 | **0.3426** | 0.1763 | 0.9685 | 1,017,538 | 1579.2s |
| 8 | #44 | 19 (`EMG_BW, EMG_HjMob, EMG_LogDet, EMG_MA...`) | 164 | **0.3427** | 0.1742 | 0.9684 | 1,017,538 | 1571.4s |
| 9 | #5 | 15 (`EMG_HjMob, EMG_Kurt, EMG_LogDet, EMG_...`) | 148 | **0.3430** | 0.1533 | 0.9684 | 1,016,002 | 1577.7s |
| 10 | #20 | 15 (`EMG_BW, EMG_IEMG, EMG_Kurt, EMG_LogDe...`) | 148 | **0.3437** | 0.1710 | 0.9682 | 1,016,002 | 1560.5s |
| 11 | #28 | 15 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_MAV...`) | 136 | **0.3444** | 0.1717 | 0.9681 | 1,014,850 | 1602.4s |
| 12 | #32 | 19 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 176 | **0.3446** | 0.1798 | 0.9681 | 1,018,690 | 1601.6s |
| 13 | #16 | 20 (`EMG_HjComp, EMG_IEMG, EMG_Kurt, EMG_L...`) | 160 | **0.3453** | 0.1464 | 0.9680 | 1,017,154 | 1580.0s |
| 14 | #34 | 18 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 160 | **0.3470** | 0.1658 | 0.9676 | 1,017,154 | 1585.2s |
| 15 | #25 | 21 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 168 | **0.3480** | 0.1664 | 0.9675 | 1,017,922 | 1610.5s |
| 16 | #13 | 18 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_LogD...`) | 156 | **0.3487** | 0.1723 | 0.9673 | 1,016,770 | 1572.0s |
| 17 | #10 | 18 (`EMG_HjComp, EMG_Kurt, EMG_LogDet, EMG...`) | 148 | **0.3525** | 0.1725 | 0.9666 | 1,016,002 | 1564.0s |
| 18 | #12 | 17 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 168 | **0.3534** | 0.1677 | 0.9664 | 1,017,922 | 1568.2s |
| 19 | #37 | 16 (`EMG_HjComp, EMG_IEMG, EMG_LogDet, EMG...`) | 152 | **0.3549** | 0.1728 | 0.9661 | 1,016,386 | 1574.0s |
| 20 | #45 | 21 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 172 | **0.3554** | 0.1694 | 0.9660 | 1,018,306 | 1569.2s |
| 21 | #9 | 21 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_IE...`) | 168 | **0.3564** | 0.1736 | 0.9659 | 1,017,922 | 1580.8s |
| 22 | #1 | 14 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 120 | **0.3568** | 0.1715 | 0.9658 | 1,013,314 | 1690.8s |
| 23 | #17 | 23 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 212 | **0.3568** | 0.1741 | 0.9658 | 1,022,146 | 1572.2s |
| 24 | #4 | 16 (`EMG_HjMob, EMG_Kurt, EMG_LogDet, EMG_...`) | 140 | **0.3572** | 0.1759 | 0.9657 | 1,015,234 | 1600.3s |
| 25 | #22 | 16 (`EMG_HjMob, EMG_Kurt, EMG_MDF, EMG_RMS...`) | 148 | **0.3581** | 0.1697 | 0.9655 | 1,016,002 | 1564.9s |
| 26 | #8 | 19 (`EMG_BW, EMG_HjComp, EMG_Kurt, EMG_MDF...`) | 184 | **0.3598** | 0.1745 | 0.9652 | 1,019,458 | 1581.8s |
| 27 | #15 | 20 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 172 | **0.3603** | 0.1679 | 0.9651 | 1,018,306 | 1570.4s |
| 28 | #14 | 14 (`EMG_HjMob, EMG_IEMG, EMG_MAV, EMG_MNF...`) | 128 | **0.3621** | 0.1805 | 0.9648 | 1,014,082 | 1579.9s |
| 29 | #24 | 17 (`EMG_BW, EMG_Kurt, EMG_MAV, EMG_MDF, E...`) | 164 | **0.3634** | 0.1945 | 0.9645 | 1,017,538 | 1581.4s |
| 30 | #47 | 15 (`EMG_HjComp, EMG_IEMG, EMG_PeakFreq, E...`) | 140 | **0.3637** | 0.1870 | 0.9644 | 1,015,234 | 1566.7s |
| 31 | #11 | 14 (`EMG_HjComp, EMG_MAV, EMG_MDF, EMG_Spe...`) | 132 | **0.3643** | 0.1724 | 0.9643 | 1,014,466 | 1571.7s |
| 32 | #39 | 12 (`EMG_BW, EMG_LogDet, EMG_MDF, EMG_Skew...`) | 92 | **0.3646** | 0.1722 | 0.9643 | 1,010,626 | 1571.6s |
| 33 | #48 | 17 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 148 | **0.3654** | 0.1751 | 0.9641 | 1,016,002 | 1564.0s |
| 34 | #50 | 21 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 184 | **0.3655** | 0.1714 | 0.9641 | 1,019,458 | 1565.6s |
| 35 | #35 | 21 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_IE...`) | 192 | **0.3663** | 0.1687 | 0.9639 | 1,020,226 | 1566.5s |
| 36 | #19 | 11 (`EMG_BW, EMG_HjMob, EMG_Skew, EMG_Spec...`) | 108 | **0.3673** | 0.1704 | 0.9637 | 1,012,162 | 1568.9s |
| 37 | #23 | 18 (`EMG_HjComp, EMG_LogDet, EMG_MAV, EMG_...`) | 168 | **0.3677** | 0.1762 | 0.9637 | 1,017,922 | 1566.2s |
| 38 | #26 | 12 (`EMG_BW, EMG_IEMG, EMG_Power, EMG_SSC,...`) | 92 | **0.3681** | 0.1689 | 0.9636 | 1,010,626 | 1586.2s |
| 39 | #6 | 18 (`EMG_BW, EMG_Kurt, EMG_LogDet, EMG_MDF...`) | 156 | **0.3682** | 0.1784 | 0.9636 | 1,016,770 | 1576.1s |
| 40 | #3 | 17 (`EMG_HjComp, EMG_IEMG, EMG_Kurt, EMG_M...`) | 156 | **0.3692** | 0.1724 | 0.9634 | 1,016,770 | 1606.0s |
| 41 | #42 | 17 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_MAV,...`) | 156 | **0.3696** | 0.1761 | 0.9633 | 1,016,770 | 1569.9s |
| 42 | #51 | 16 (`EMG_LogDet, EMG_MAV, EMG_PeakFreq, EM...`) | 144 | **0.3725** | 0.1671 | 0.9627 | 1,015,618 | 1571.6s |
| 43 | #27 | 13 (`EMG_BW, EMG_HjMob, EMG_Kurt, EMG_MDF,...`) | 96 | **0.3775** | 0.1745 | 0.9617 | 1,011,010 | 1593.9s |
| 44 | #30 | 13 (`EMG_HjMob, EMG_Kurt, EMG_Myopulse, EM...`) | 124 | **0.3784** | 0.1842 | 0.9615 | 1,013,698 | 1588.7s |
| 45 | #41 | 16 (`EMG_HjMob, EMG_Kurt, EMG_MDF, EMG_MNF...`) | 128 | **0.3803** | 0.1918 | 0.9611 | 1,014,082 | 1576.0s |
| 46 | #46 | 15 (`EMG_HjMob, EMG_Myopulse, EMG_PeakFreq...`) | 144 | **0.3818** | 0.1884 | 0.9608 | 1,015,618 | 1570.4s |
| 47 | #7 | 19 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 160 | **0.3829** | 0.1759 | 0.9606 | 1,017,154 | 1597.7s |
| 48 | #43 | 14 (`EMG_BW, EMG_HjMob, EMG_MDF, EMG_Myopu...`) | 112 | **0.3899** | 0.1919 | 0.9591 | 1,012,546 | 1573.3s |
| 49 | #2 | 16 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 148 | **0.3914** | 0.1921 | 0.9588 | 1,016,002 | 1629.9s |
| 50 | #18 | 11 (`EMG_Kurt, EMG_MDF, EMG_Myopulse, EMG_...`) | 88 | **0.4104** | 0.2124 | 0.9547 | 1,010,242 | 1578.7s |
| 51 | #38 | 16 (`EMG_HjMob, EMG_Kurt, EMG_MDF, EMG_Myo...`) | 136 | **0.5658** | 0.2758 | 0.9139 | 1,014,850 | 1583.9s |
