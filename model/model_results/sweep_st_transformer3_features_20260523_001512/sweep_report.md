# Spatio-Temporal Transformer 3 Consolidated Sweep Report (FEATURES)

**Date:** 2026-05-23 11:29:03

**DEV_MODE:** False

**Total Execution Time:** 654.6 minutes

**Cross-Participant Validation Group:** held out subjects `P01, P02, P06, P17` for testing.

## 🎯 Ridge Regression Feature Contribution Analysis

Fits a Ridge regression model mapping feature category binary indicators (0 or 1) to validation $R^2$ scores across all iterations. A **positive coefficient** means that including this feature group on average **increases** $R^2$ (improves generalization), while a **negative coefficient** indicates it degrades generalization or adds noise.

* **Ridge Regression Fit $R^2$ Score:** 0.1453
* **Baseline Intercept (Average $R^2$):** 0.8572

| Rank | Feature Category | Ridge Coefficient (Delta R²) | Impact on Generalization |
| :---: | :--- | :---: | :--- |
| 1 | `IMU_Std` | **+0.01619** | ✅ Positive (Improves Generalization) |
| 2 | `EMG_LogDet` | **+0.01598** | ✅ Positive (Improves Generalization) |
| 3 | `IMU_Var` | **+0.01571** | ✅ Positive (Improves Generalization) |
| 4 | `IMU_Mean` | **+0.01379** | ✅ Positive (Improves Generalization) |
| 5 | `EMG_HjMob` | **+0.01274** | ✅ Positive (Improves Generalization) |
| 6 | `IMU_P2P` | **+0.01239** | ✅ Positive (Improves Generalization) |
| 7 | `IMU_Jerk` | **+0.01139** | ✅ Positive (Improves Generalization) |
| 8 | `IMU_DomFreq` | **+0.01096** | ✅ Positive (Improves Generalization) |
| 9 | `EMG_WL` | **+0.00974** | ✅ Positive (Improves Generalization) |
| 10 | `EMG_RMS` | **+0.00740** | ✅ Positive (Improves Generalization) |
| 11 | `IMU_SMA` | **+0.00643** | ✅ Positive (Improves Generalization) |
| 12 | `IMU_Kurt` | **+0.00634** | ✅ Positive (Improves Generalization) |
| 13 | `EMG_BW` | **+0.00563** | ✅ Positive (Improves Generalization) |
| 14 | `EMG_Power` | **+0.00551** | ✅ Positive (Improves Generalization) |
| 15 | `IMU_Skew` | **+0.00539** | ✅ Positive (Improves Generalization) |
| 16 | `EMG_MNF` | **+0.00508** | ✅ Positive (Improves Generalization) |
| 17 | `EMG_ZC` | **+0.00438** | ✅ Positive (Improves Generalization) |
| 18 | `EMG_Myopulse` | **+0.00283** | ✅ Positive (Improves Generalization) |
| 19 | `IMU_Max` | **+0.00262** | ✅ Positive (Improves Generalization) |
| 20 | `EMG_SSC` | **+0.00232** | ✅ Positive (Improves Generalization) |
| 21 | `EMG_WAMP` | **+0.00179** | ✅ Positive (Improves Generalization) |
| 22 | `IMU_SVM_Mean` | **-0.00157** | ❌ Negative (Reduces Generalization) |
| 23 | `EMG_PeakFreq` | **-0.00326** | ❌ Negative (Reduces Generalization) |
| 24 | `EMG_IEMG` | **-0.00352** | ❌ Negative (Reduces Generalization) |
| 25 | `IMU_MNF` | **-0.00363** | ❌ Negative (Reduces Generalization) |
| 26 | `EMG_MAV` | **-0.00371** | ❌ Negative (Reduces Generalization) |
| 27 | `EMG_MDF` | **-0.00425** | ❌ Negative (Reduces Generalization) |
| 28 | `EMG_HjComp` | **-0.01047** | ❌ Negative (Reduces Generalization) |
| 29 | `EMG_Skew` | **-0.01149** | ❌ Negative (Reduces Generalization) |
| 30 | `EMG_VAR` | **-0.01334** | ❌ Negative (Reduces Generalization) |
| 31 | `IMU_SpecEnergy` | **-0.01337** | ❌ Negative (Reduces Generalization) |
| 32 | `EMG_SpecEntropy` | **-0.01440** | ❌ Negative (Reduces Generalization) |
| 33 | `EMG_Kurt` | **-0.01696** | ❌ Negative (Reduces Generalization) |

## 🏆 Ranked Leaderboard (Sorted by RMSE)

| Rank | Iteration | Active Categories | Features | Val RMSE | Val MAE | Val R² | Model Params | Training Time | Epochs |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| 1 | #74 | 14 (`EMG_HjComp, EMG_LogDet, EMG_MAV, EMG_...`) | 144 | **0.4376** | 0.2590 | 0.9485 | 1,354,498 | 188.0s | 28 |
| 2 | #51 | 16 (`EMG_LogDet, EMG_MAV, EMG_PeakFreq, EM...`) | 156 | **0.4514** | 0.2612 | 0.9452 | 1,356,034 | 149.1s | 22 |
| 3 | #52 | 15 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_LogD...`) | 144 | **0.4565** | 0.2903 | 0.9440 | 1,354,498 | 147.4s | 22 |
| 4 | #93 | 13 (`EMG_HjComp, EMG_IEMG, EMG_LogDet, EMG...`) | 116 | **0.4582** | 0.2512 | 0.9435 | 1,350,914 | 146.6s | 22 |
| 5 | #79 | 18 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 148 | **0.4605** | 0.2443 | 0.9430 | 1,355,010 | 209.9s | 30 |
| 6 | #73 | 15 (`EMG_HjComp, EMG_IEMG, EMG_Kurt, EMG_L...`) | 128 | **0.4607** | 0.2375 | 0.9429 | 1,352,450 | 280.4s | 43 |
| 7 | #42 | 17 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_MAV,...`) | 156 | **0.4659** | 0.2670 | 0.9416 | 1,356,034 | 154.1s | 22 |
| 8 | #96 | 17 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_IE...`) | 144 | **0.4737** | 0.2496 | 0.9397 | 1,354,498 | 183.3s | 28 |
| 9 | #80 | 17 (`EMG_HjComp, EMG_IEMG, EMG_MAV, EMG_Po...`) | 160 | **0.4754** | 0.2773 | 0.9392 | 1,356,546 | 143.8s | 21 |
| 10 | #85 | 20 (`EMG_BW, EMG_HjMob, EMG_Myopulse, EMG_...`) | 184 | **0.4760** | 0.2954 | 0.9391 | 1,359,618 | 141.6s | 21 |
| 11 | #62 | 17 (`EMG_HjComp, EMG_Kurt, EMG_MAV, EMG_MD...`) | 152 | **0.4772** | 0.2677 | 0.9388 | 1,355,522 | 159.6s | 23 |
| 12 | #10 | 18 (`EMG_HjComp, EMG_Kurt, EMG_LogDet, EMG...`) | 160 | **0.4772** | 0.2614 | 0.9388 | 1,356,546 | 152.8s | 21 |
| 13 | #25 | 21 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 180 | **0.4774** | 0.2963 | 0.9387 | 1,359,106 | 174.4s | 24 |
| 14 | #124 | 18 (`EMG_BW, EMG_IEMG, EMG_LogDet, EMG_MNF...`) | 156 | **0.4789** | 0.2511 | 0.9383 | 1,356,034 | 214.3s | 31 |
| 15 | #59 | 19 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 168 | **0.4812** | 0.2516 | 0.9377 | 1,357,570 | 165.4s | 25 |
| 16 | #23 | 18 (`EMG_HjComp, EMG_LogDet, EMG_MAV, EMG_...`) | 180 | **0.4833** | 0.2598 | 0.9372 | 1,359,106 | 209.9s | 29 |
| 17 | #137 | 17 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 152 | **0.4870** | 0.2646 | 0.9362 | 1,355,522 | 140.8s | 21 |
| 18 | #128 | 16 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_MNF,...`) | 160 | **0.4875** | 0.2733 | 0.9361 | 1,356,546 | 150.5s | 21 |
| 19 | #146 | 16 (`EMG_HjComp, EMG_HjMob, EMG_MAV, EMG_R...`) | 152 | **0.4876** | 0.2665 | 0.9361 | 1,355,522 | 154.2s | 22 |
| 20 | #77 | 20 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Log...`) | 184 | **0.4879** | 0.3245 | 0.9360 | 1,359,618 | 185.6s | 25 |
| 21 | #86 | 14 (`EMG_BW, EMG_HjComp, EMG_Kurt, EMG_MAV...`) | 132 | **0.4882** | 0.2625 | 0.9359 | 1,352,962 | 150.3s | 23 |
| 22 | #28 | 15 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_MAV...`) | 148 | **0.4890** | 0.2743 | 0.9357 | 1,355,010 | 223.4s | 31 |
| 23 | #40 | 18 (`EMG_LogDet, EMG_MAV, EMG_MDF, EMG_Pow...`) | 164 | **0.4905** | 0.2718 | 0.9353 | 1,357,058 | 154.0s | 23 |
| 24 | #84 | 14 (`EMG_LogDet, EMG_MAV, EMG_MDF, EMG_Myo...`) | 132 | **0.4905** | 0.3009 | 0.9353 | 1,352,962 | 152.0s | 23 |
| 25 | #15 | 20 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 184 | **0.4910** | 0.2677 | 0.9352 | 1,359,618 | 203.6s | 29 |
| 26 | #100 | 18 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 172 | **0.4923** | 0.2649 | 0.9348 | 1,358,082 | 163.3s | 24 |
| 27 | #55 | 16 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 132 | **0.4927** | 0.2623 | 0.9347 | 1,352,962 | 154.1s | 23 |
| 28 | #140 | 15 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 136 | **0.4932** | 0.2867 | 0.9346 | 1,353,474 | 150.6s | 22 |
| 29 | #109 | 17 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 144 | **0.4936** | 0.2611 | 0.9345 | 1,354,498 | 240.7s | 36 |
| 30 | #6 | 18 (`EMG_BW, EMG_Kurt, EMG_LogDet, EMG_MDF...`) | 168 | **0.4937** | 0.2622 | 0.9345 | 1,357,570 | 236.3s | 32 |
| 31 | #44 | 19 (`EMG_BW, EMG_HjMob, EMG_LogDet, EMG_MA...`) | 176 | **0.4939** | 0.2604 | 0.9344 | 1,358,594 | 158.9s | 21 |
| 32 | #56 | 18 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_MDF,...`) | 148 | **0.4973** | 0.3157 | 0.9335 | 1,355,010 | 140.4s | 21 |
| 33 | #66 | 15 (`EMG_BW, EMG_Kurt, EMG_MDF, EMG_MNF, E...`) | 136 | **0.4975** | 0.2644 | 0.9335 | 1,353,474 | 153.7s | 22 |
| 34 | #32 | 19 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 176 | **0.4996** | 0.2967 | 0.9329 | 1,358,594 | 172.9s | 24 |
| 35 | #69 | 22 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 212 | **0.4996** | 0.3082 | 0.9329 | 1,363,202 | 153.9s | 22 |
| 36 | #133 | 14 (`EMG_HjComp, EMG_HjMob, EMG_LogDet, EM...`) | 128 | **0.4997** | 0.2815 | 0.9329 | 1,352,450 | 164.4s | 24 |
| 37 | #102 | 19 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 184 | **0.4999** | 0.2727 | 0.9328 | 1,359,618 | 152.7s | 23 |
| 38 | #106 | 20 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_MAV,...`) | 176 | **0.5003** | 0.2982 | 0.9327 | 1,358,594 | 164.0s | 24 |
| 39 | #83 | 15 (`EMG_Kurt, EMG_LogDet, EMG_PeakFreq, E...`) | 136 | **0.5006** | 0.2812 | 0.9326 | 1,353,474 | 151.2s | 22 |
| 40 | #134 | 12 (`EMG_HjMob, EMG_IEMG, EMG_LogDet, EMG_...`) | 112 | **0.5023** | 0.2754 | 0.9322 | 1,350,402 | 173.8s | 25 |
| 41 | #139 | 20 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 180 | **0.5032** | 0.2600 | 0.9319 | 1,359,106 | 158.2s | 23 |
| 42 | #45 | 21 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 184 | **0.5036** | 0.2795 | 0.9318 | 1,359,618 | 177.2s | 24 |
| 43 | #90 | 22 (`EMG_BW, EMG_HjComp, EMG_Kurt, EMG_MAV...`) | 196 | **0.5038** | 0.2976 | 0.9318 | 1,361,154 | 162.0s | 24 |
| 44 | #150 | 19 (`EMG_HjComp, EMG_IEMG, EMG_Kurt, EMG_L...`) | 180 | **0.5039** | 0.2869 | 0.9317 | 1,359,106 | 201.7s | 29 |
| 45 | #48 | 17 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 148 | **0.5039** | 0.3027 | 0.9317 | 1,355,010 | 146.1s | 22 |
| 46 | #113 | 17 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Ku...`) | 148 | **0.5062** | 0.3191 | 0.9311 | 1,355,010 | 149.6s | 21 |
| 47 | #149 | 13 (`EMG_HjComp, EMG_IEMG, EMG_LogDet, EMG...`) | 124 | **0.5062** | 0.3281 | 0.9311 | 1,351,938 | 151.3s | 22 |
| 48 | #98 | 15 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_Kurt...`) | 132 | **0.5063** | 0.2826 | 0.9311 | 1,352,962 | 144.6s | 21 |
| 49 | #26 | 12 (`EMG_BW, EMG_IEMG, EMG_Power, EMG_SSC,...`) | 104 | **0.5068** | 0.2740 | 0.9309 | 1,349,378 | 172.1s | 24 |
| 50 | #43 | 14 (`EMG_BW, EMG_HjMob, EMG_MDF, EMG_Myopu...`) | 124 | **0.5087** | 0.2855 | 0.9304 | 1,351,938 | 179.1s | 25 |
| 51 | #67 | 19 (`EMG_BW, EMG_HjComp, EMG_MAV, EMG_MDF,...`) | 180 | **0.5090** | 0.2775 | 0.9303 | 1,359,106 | 210.7s | 29 |
| 52 | #75 | 16 (`EMG_BW, EMG_HjComp, EMG_Myopulse, EMG...`) | 160 | **0.5094** | 0.3443 | 0.9302 | 1,356,546 | 148.0s | 22 |
| 53 | #9 | 21 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_IE...`) | 180 | **0.5096** | 0.3322 | 0.9302 | 1,359,106 | 179.1s | 25 |
| 54 | #39 | 12 (`EMG_BW, EMG_LogDet, EMG_MDF, EMG_Skew...`) | 104 | **0.5099** | 0.2943 | 0.9301 | 1,349,378 | 160.2s | 24 |
| 55 | #72 | 18 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 160 | **0.5100** | 0.2763 | 0.9301 | 1,356,546 | 171.2s | 24 |
| 56 | #103 | 16 (`EMG_IEMG, EMG_LogDet, EMG_MDF, EMG_MN...`) | 148 | **0.5101** | 0.2865 | 0.9300 | 1,355,010 | 169.4s | 25 |
| 57 | #27 | 13 (`EMG_BW, EMG_HjMob, EMG_Kurt, EMG_MDF,...`) | 108 | **0.5102** | 0.3085 | 0.9300 | 1,349,890 | 243.5s | 34 |
| 58 | #131 | 19 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_Myop...`) | 184 | **0.5122** | 0.3254 | 0.9295 | 1,359,618 | 174.0s | 24 |
| 59 | #141 | 21 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 184 | **0.5125** | 0.2787 | 0.9294 | 1,359,618 | 199.8s | 29 |
| 60 | #49 | 16 (`EMG_BW, EMG_HjMob, EMG_LogDet, EMG_MA...`) | 148 | **0.5128** | 0.3079 | 0.9293 | 1,355,010 | 143.7s | 22 |
| 61 | #58 | 16 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_MD...`) | 148 | **0.5130** | 0.3306 | 0.9292 | 1,355,010 | 145.4s | 22 |
| 62 | #104 | 20 (`EMG_BW, EMG_IEMG, EMG_LogDet, EMG_MAV...`) | 192 | **0.5130** | 0.3003 | 0.9292 | 1,360,642 | 152.6s | 22 |
| 63 | #105 | 21 (`EMG_BW, EMG_HjMob, EMG_LogDet, EMG_MA...`) | 200 | **0.5136** | 0.2894 | 0.9291 | 1,361,666 | 148.6s | 21 |
| 64 | #22 | 16 (`EMG_HjMob, EMG_Kurt, EMG_MDF, EMG_RMS...`) | 160 | **0.5146** | 0.2901 | 0.9288 | 1,356,546 | 160.9s | 22 |
| 65 | #61 | 12 (`EMG_HjComp, EMG_MAV, EMG_Myopulse, EM...`) | 112 | **0.5147** | 0.2923 | 0.9288 | 1,350,402 | 150.8s | 22 |
| 66 | #123 | 19 (`EMG_HjMob, EMG_Kurt, EMG_MDF, EMG_Myo...`) | 176 | **0.5149** | 0.2813 | 0.9287 | 1,358,594 | 195.7s | 28 |
| 67 | #114 | 17 (`EMG_BW, EMG_HjMob, EMG_Kurt, EMG_MDF,...`) | 160 | **0.5150** | 0.2976 | 0.9287 | 1,356,546 | 179.7s | 26 |
| 68 | #117 | 17 (`EMG_BW, EMG_IEMG, EMG_Kurt, EMG_MNF, ...`) | 164 | **0.5153** | 0.2830 | 0.9286 | 1,357,058 | 152.1s | 22 |
| 69 | #115 | 17 (`EMG_BW, EMG_Kurt, EMG_MNF, EMG_Myopul...`) | 160 | **0.5155** | 0.2932 | 0.9286 | 1,356,546 | 156.5s | 22 |
| 70 | #129 | 16 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_MAV...`) | 152 | **0.5163** | 0.2919 | 0.9283 | 1,355,522 | 149.0s | 21 |
| 71 | #17 | 23 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 224 | **0.5169** | 0.3324 | 0.9282 | 1,364,738 | 186.6s | 26 |
| 72 | #16 | 20 (`EMG_HjComp, EMG_IEMG, EMG_Kurt, EMG_L...`) | 172 | **0.5169** | 0.3394 | 0.9282 | 1,358,082 | 159.1s | 22 |
| 73 | #14 | 14 (`EMG_HjMob, EMG_IEMG, EMG_MAV, EMG_MNF...`) | 128 | **0.5169** | 0.2875 | 0.9281 | 1,352,450 | 150.7s | 21 |
| 74 | #116 | 13 (`EMG_BW, EMG_HjComp, EMG_Kurt, EMG_MAV...`) | 120 | **0.5182** | 0.2909 | 0.9278 | 1,351,426 | 159.5s | 23 |
| 75 | #111 | 13 (`EMG_BW, EMG_MAV, EMG_Power, EMG_Skew,...`) | 120 | **0.5183** | 0.3090 | 0.9278 | 1,351,426 | 171.1s | 25 |
| 76 | #87 | 17 (`EMG_BW, EMG_IEMG, EMG_Kurt, EMG_LogDe...`) | 152 | **0.5192** | 0.2840 | 0.9275 | 1,355,522 | 143.4s | 21 |
| 77 | #95 | 17 (`EMG_BW, EMG_Kurt, EMG_MAV, EMG_MDF, E...`) | 160 | **0.5193** | 0.2974 | 0.9275 | 1,356,546 | 186.2s | 27 |
| 78 | #101 | 18 (`EMG_HjComp, EMG_HjMob, EMG_PeakFreq, ...`) | 184 | **0.5194** | 0.3012 | 0.9275 | 1,359,618 | 166.8s | 24 |
| 79 | #33 | 14 (`EMG_BW, EMG_IEMG, EMG_Kurt, EMG_LogDe...`) | 128 | **0.5202** | 0.3146 | 0.9272 | 1,352,450 | 159.3s | 24 |
| 80 | #89 | 18 (`EMG_BW, EMG_HjMob, EMG_MNF, EMG_Myopu...`) | 180 | **0.5203** | 0.2772 | 0.9272 | 1,359,106 | 163.4s | 24 |
| 81 | #119 | 18 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 164 | **0.5209** | 0.3582 | 0.9270 | 1,357,058 | 142.1s | 21 |
| 82 | #60 | 15 (`EMG_LogDet, EMG_MAV, EMG_MDF, EMG_MNF...`) | 128 | **0.5219** | 0.3068 | 0.9267 | 1,352,450 | 159.9s | 24 |
| 83 | #135 | 17 (`EMG_BW, EMG_HjComp, EMG_MDF, EMG_MNF,...`) | 164 | **0.5235** | 0.2845 | 0.9263 | 1,357,058 | 156.2s | 22 |
| 84 | #57 | 24 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 220 | **0.5245** | 0.2961 | 0.9260 | 1,364,226 | 148.8s | 22 |
| 85 | #47 | 15 (`EMG_HjComp, EMG_IEMG, EMG_PeakFreq, E...`) | 140 | **0.5255** | 0.3244 | 0.9258 | 1,353,986 | 152.9s | 23 |
| 86 | #7 | 19 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 172 | **0.5257** | 0.3193 | 0.9257 | 1,358,082 | 175.7s | 24 |
| 87 | #91 | 18 (`EMG_HjMob, EMG_IEMG, EMG_MAV, EMG_MNF...`) | 168 | **0.5260** | 0.3598 | 0.9256 | 1,357,570 | 155.5s | 22 |
| 88 | #92 | 15 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_IE...`) | 140 | **0.5264** | 0.2949 | 0.9255 | 1,353,986 | 146.5s | 22 |
| 89 | #13 | 18 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_LogD...`) | 168 | **0.5265** | 0.2837 | 0.9254 | 1,357,570 | 166.3s | 23 |
| 90 | #68 | 17 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_LogD...`) | 168 | **0.5269** | 0.2921 | 0.9253 | 1,357,570 | 177.8s | 26 |
| 91 | #36 | 21 (`EMG_BW, EMG_HjMob, EMG_IEMG, EMG_LogD...`) | 192 | **0.5274** | 0.3652 | 0.9252 | 1,360,642 | 146.1s | 21 |
| 92 | #2 | 16 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 148 | **0.5286** | 0.2876 | 0.9249 | 1,355,010 | 189.8s | 26 |
| 93 | #94 | 18 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 164 | **0.5291** | 0.3170 | 0.9247 | 1,357,058 | 142.1s | 21 |
| 94 | #82 | 14 (`EMG_HjComp, EMG_MAV, EMG_MNF, EMG_WAM...`) | 136 | **0.5306** | 0.3010 | 0.9243 | 1,353,474 | 196.8s | 29 |
| 95 | #112 | 13 (`EMG_HjComp, EMG_IEMG, EMG_Myopulse, E...`) | 116 | **0.5306** | 0.2862 | 0.9243 | 1,350,914 | 267.9s | 40 |
| 96 | #31 | 20 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_MAV...`) | 180 | **0.5307** | 0.3486 | 0.9243 | 1,359,106 | 170.8s | 24 |
| 97 | #37 | 16 (`EMG_HjComp, EMG_IEMG, EMG_LogDet, EMG...`) | 152 | **0.5307** | 0.3297 | 0.9243 | 1,355,522 | 171.1s | 26 |
| 98 | #142 | 20 (`EMG_BW, EMG_IEMG, EMG_Kurt, EMG_MAV, ...`) | 184 | **0.5320** | 0.2833 | 0.9239 | 1,359,618 | 198.7s | 29 |
| 99 | #35 | 21 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_IE...`) | 204 | **0.5367** | 0.3071 | 0.9225 | 1,362,178 | 153.3s | 22 |
| 100 | #148 | 15 (`EMG_HjComp, EMG_IEMG, EMG_MAV, EMG_MD...`) | 128 | **0.5370** | 0.2768 | 0.9225 | 1,352,450 | 189.1s | 28 |
| 101 | #81 | 15 (`EMG_HjMob, EMG_IEMG, EMG_MAV, EMG_MDF...`) | 136 | **0.5384** | 0.3156 | 0.9220 | 1,353,474 | 139.9s | 21 |
| 102 | #143 | 16 (`EMG_HjMob, EMG_IEMG, EMG_MDF, EMG_Myo...`) | 144 | **0.5387** | 0.3712 | 0.9220 | 1,354,498 | 207.1s | 29 |
| 103 | #76 | 18 (`EMG_BW, EMG_HjMob, EMG_Kurt, EMG_MAV,...`) | 164 | **0.5390** | 0.3116 | 0.9219 | 1,357,058 | 184.6s | 26 |
| 104 | #121 | 15 (`EMG_BW, EMG_IEMG, EMG_LogDet, EMG_MNF...`) | 144 | **0.5406** | 0.2956 | 0.9214 | 1,354,498 | 157.0s | 23 |
| 105 | #11 | 14 (`EMG_HjComp, EMG_MAV, EMG_MDF, EMG_Spe...`) | 132 | **0.5416** | 0.2938 | 0.9211 | 1,352,962 | 258.4s | 36 |
| 106 | #108 | 10 (`EMG_BW, EMG_MDF, EMG_PeakFreq, EMG_SS...`) | 96 | **0.5418** | 0.3195 | 0.9211 | 1,348,354 | 147.5s | 22 |
| 107 | #65 | 20 (`EMG_HjComp, EMG_HjMob, EMG_LogDet, EM...`) | 184 | **0.5424** | 0.2871 | 0.9209 | 1,359,618 | 205.2s | 29 |
| 108 | #53 | 14 (`EMG_HjComp, EMG_HjMob, EMG_MNF, EMG_P...`) | 128 | **0.5442** | 0.3094 | 0.9204 | 1,352,450 | 181.8s | 28 |
| 109 | #18 | 11 (`EMG_Kurt, EMG_MDF, EMG_Myopulse, EMG_...`) | 100 | **0.5459** | 0.3000 | 0.9199 | 1,348,866 | 155.2s | 22 |
| 110 | #127 | 18 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Ku...`) | 172 | **0.5483** | 0.3217 | 0.9192 | 1,358,082 | 159.8s | 22 |
| 111 | #34 | 18 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 160 | **0.5490** | 0.2966 | 0.9189 | 1,356,546 | 162.2s | 25 |
| 112 | #12 | 17 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 168 | **0.5491** | 0.3086 | 0.9189 | 1,357,570 | 220.1s | 31 |
| 113 | #125 | 16 (`EMG_HjComp, EMG_MNF, EMG_PeakFreq, EM...`) | 156 | **0.5501** | 0.2985 | 0.9186 | 1,356,034 | 150.7s | 22 |
| 114 | #50 | 21 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Lo...`) | 196 | **0.5518** | 0.3161 | 0.9181 | 1,361,154 | 190.0s | 29 |
| 115 | #20 | 15 (`EMG_BW, EMG_IEMG, EMG_Kurt, EMG_LogDe...`) | 148 | **0.5534** | 0.3247 | 0.9176 | 1,355,010 | 169.4s | 23 |
| 116 | #132 | 19 (`EMG_HjComp, EMG_HjMob, EMG_Kurt, EMG_...`) | 184 | **0.5543** | 0.3164 | 0.9174 | 1,359,618 | 160.7s | 22 |
| 117 | #78 | 13 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_Po...`) | 108 | **0.5567** | 0.3637 | 0.9167 | 1,349,890 | 149.5s | 21 |
| 118 | #64 | 19 (`EMG_BW, EMG_Kurt, EMG_LogDet, EMG_MNF...`) | 168 | **0.5569** | 0.3246 | 0.9166 | 1,357,570 | 163.6s | 23 |
| 119 | #4 | 16 (`EMG_HjMob, EMG_Kurt, EMG_LogDet, EMG_...`) | 140 | **0.5572** | 0.3114 | 0.9165 | 1,353,986 | 157.0s | 21 |
| 120 | #138 | 17 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_IE...`) | 152 | **0.5582** | 0.3212 | 0.9162 | 1,355,522 | 220.7s | 33 |
| 121 | #88 | 15 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_MDF...`) | 144 | **0.5587** | 0.3503 | 0.9161 | 1,354,498 | 159.6s | 24 |
| 122 | #1 | 14 (`EMG_HjComp, EMG_HjMob, EMG_IEMG, EMG_...`) | 120 | **0.5595** | 0.3086 | 0.9158 | 1,351,426 | 259.7s | 23 |
| 123 | #120 | 13 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Log...`) | 112 | **0.5604** | 0.3058 | 0.9155 | 1,350,402 | 188.2s | 28 |
| 124 | #63 | 16 (`EMG_HjComp, EMG_HjMob, EMG_MDF, EMG_M...`) | 140 | **0.5610** | 0.3777 | 0.9154 | 1,353,986 | 160.3s | 23 |
| 125 | #24 | 17 (`EMG_BW, EMG_Kurt, EMG_MAV, EMG_MDF, E...`) | 164 | **0.5621** | 0.3663 | 0.9150 | 1,357,058 | 152.2s | 21 |
| 126 | #8 | 19 (`EMG_BW, EMG_HjComp, EMG_Kurt, EMG_MDF...`) | 184 | **0.5621** | 0.2938 | 0.9150 | 1,359,618 | 267.7s | 37 |
| 127 | #147 | 13 (`EMG_BW, EMG_MAV, EMG_MNF, EMG_Power, ...`) | 112 | **0.5627** | 0.3223 | 0.9149 | 1,350,402 | 150.7s | 21 |
| 128 | #126 | 17 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_MD...`) | 160 | **0.5637** | 0.3803 | 0.9146 | 1,356,546 | 161.9s | 23 |
| 129 | #29 | 17 (`EMG_IEMG, EMG_Kurt, EMG_MDF, EMG_MNF,...`) | 164 | **0.5693** | 0.3440 | 0.9129 | 1,357,058 | 160.9s | 22 |
| 130 | #97 | 16 (`EMG_BW, EMG_HjMob, EMG_MNF, EMG_Myopu...`) | 148 | **0.5693** | 0.3137 | 0.9129 | 1,355,010 | 189.5s | 28 |
| 131 | #3 | 17 (`EMG_HjComp, EMG_IEMG, EMG_Kurt, EMG_M...`) | 156 | **0.5694** | 0.3376 | 0.9128 | 1,356,034 | 172.9s | 23 |
| 132 | #5 | 15 (`EMG_HjMob, EMG_Kurt, EMG_LogDet, EMG_...`) | 148 | **0.5791** | 0.3770 | 0.9098 | 1,355,010 | 192.9s | 26 |
| 133 | #30 | 13 (`EMG_HjMob, EMG_Kurt, EMG_Myopulse, EM...`) | 124 | **0.5804** | 0.3397 | 0.9094 | 1,351,938 | 165.7s | 23 |
| 134 | #122 | 13 (`EMG_LogDet, EMG_MDF, EMG_MNF, EMG_Myo...`) | 132 | **0.5846** | 0.3474 | 0.9081 | 1,352,962 | 150.3s | 22 |
| 135 | #130 | 15 (`EMG_Kurt, EMG_MDF, EMG_Myopulse, EMG_...`) | 132 | **0.5863** | 0.4179 | 0.9076 | 1,352,962 | 158.9s | 22 |
| 136 | #54 | 14 (`EMG_HjComp, EMG_LogDet, EMG_MAV, EMG_...`) | 140 | **0.5895** | 0.3301 | 0.9066 | 1,353,986 | 161.1s | 23 |
| 137 | #21 | 14 (`EMG_HjComp, EMG_HjMob, EMG_Power, EMG...`) | 128 | **0.5916** | 0.3662 | 0.9059 | 1,352,450 | 202.4s | 29 |
| 138 | #145 | 15 (`EMG_BW, EMG_HjComp, EMG_HjMob, EMG_IE...`) | 132 | **0.5975** | 0.3227 | 0.9040 | 1,352,962 | 151.9s | 21 |
| 139 | #46 | 15 (`EMG_HjMob, EMG_Myopulse, EMG_PeakFreq...`) | 156 | **0.6099** | 0.3433 | 0.9000 | 1,356,034 | 243.8s | 37 |
| 140 | #19 | 11 (`EMG_BW, EMG_HjMob, EMG_Skew, EMG_Spec...`) | 108 | **0.6101** | 0.3565 | 0.8999 | 1,349,890 | 165.3s | 22 |
| 141 | #41 | 16 (`EMG_HjMob, EMG_Kurt, EMG_MDF, EMG_MNF...`) | 140 | **0.6122** | 0.3853 | 0.8992 | 1,353,986 | 145.0s | 22 |
| 142 | #118 | 17 (`EMG_BW, EMG_MAV, EMG_MDF, EMG_RMS, EM...`) | 176 | **0.6153** | 0.3469 | 0.8982 | 1,358,594 | 198.7s | 29 |
| 143 | #99 | 15 (`EMG_IEMG, EMG_Kurt, EMG_LogDet, EMG_M...`) | 136 | **0.6634** | 0.4125 | 0.8817 | 1,353,474 | 176.4s | 26 |
| 144 | #110 | 16 (`EMG_BW, EMG_HjMob, EMG_Kurt, EMG_Myop...`) | 156 | **0.6684** | 0.5281 | 0.8799 | 1,356,034 | 149.6s | 21 |
| 145 | #70 | 10 (`EMG_BW, EMG_HjComp, EMG_Myopulse, EMG...`) | 100 | **0.7478** | 0.4555 | 0.8496 | 1,348,866 | 142.5s | 21 |
| 146 | #38 | 16 (`EMG_HjMob, EMG_Kurt, EMG_MDF, EMG_Myo...`) | 148 | **1.0631** | 0.6627 | 0.6961 | 1,355,010 | 175.8s | 27 |
| 147 | #144 | 11 (`EMG_BW, EMG_MDF, EMG_MNF, EMG_Myopuls...`) | 104 | **1.2100** | 0.6562 | 0.6063 | 1,349,378 | 163.9s | 24 |
| 148 | #71 | 14 (`EMG_BW, EMG_HjComp, EMG_IEMG, EMG_Kur...`) | 124 | **2.2130** | 1.3685 | -0.3168 | 1,351,938 | 138.3s | 20 |
| 149 | #136 | 15 (`EMG_HjComp, EMG_Kurt, EMG_MAV, EMG_MD...`) | 128 | **2.2690** | 1.4083 | -0.3844 | 1,352,450 | 137.8s | 20 |
| 150 | #107 | 14 (`EMG_HjComp, EMG_IEMG, EMG_Kurt, EMG_M...`) | 112 | **2.3408** | 1.3285 | -0.4733 | 1,350,402 | 137.2s | 20 |
