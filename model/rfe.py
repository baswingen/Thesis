import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime
from sklearn.feature_selection import RFECV
from sklearn.model_selection import StratifiedKFold
from sklearn.preprocessing import StandardScaler

# Add project root to sys.path so 'model' package can be found
sys.path.append(str(Path(__file__).parent.parent))

from model.data_loader import DataLoader
from model.run_model import initialize_model

###########################################################
# CONFIGURATION
###########################################################
# Choose model to train: "svr", "rf", or "gb"
MODEL_TYPE = "svr"

# Correlation Threshold to detect redundancy
# Pairs of features with a Spearman correlation > this will be flagged,
# and one of each pair will be dropped to reduce multicollinearity before RFE.
CORRELATION_THRESHOLD = 0.95
###########################################################

import re
from collections import defaultdict

def perform_correlation_analysis(X: pd.DataFrame, run_dir: Path):
    """
    Computes pairwise correlations between FEATURE TYPES (averaged across their channels)
    and identifies highly correlated feature types to drop, to help configuring config_model.py.
    """
    print(f"\n--- Performing Correlation Analysis (Feature Types) ---")
    
    def get_feature_type(col_name):
        m = re.match(r'^.+?_((?:EMG|IMU)_.*)$', col_name)
        return m.group(1) if m else col_name
        
    def get_channel(col_name, ftype):
        if col_name.endswith(f"_{ftype}"):
            return col_name[:-(len(ftype)+1)]
        return col_name

    type_to_cols = defaultdict(list)
    for col in X.columns:
        ftype = get_feature_type(col)
        type_to_cols[ftype].append(col)
        
    feature_types = list(type_to_cols.keys())
    n_types = len(feature_types)
    corr_matrix_types = pd.DataFrame(np.nan, index=feature_types, columns=feature_types)
    
    print(f"Grouped {X.shape[1]} raw columns into {n_types} feature types.")
    
    for i in range(n_types):
        for j in range(i, n_types):
            ft1 = feature_types[i]
            ft2 = feature_types[j]
            
            if i == j:
                corr_matrix_types.loc[ft1, ft2] = 1.0
                continue
                
            chs1 = {get_channel(c, ft1) for c in type_to_cols[ft1]}
            chs2 = {get_channel(c, ft2) for c in type_to_cols[ft2]}
            
            common_chs = chs1.intersection(chs2)
            
            if not common_chs:
                corr_matrix_types.loc[ft1, ft2] = 0.0
                corr_matrix_types.loc[ft2, ft1] = 0.0
                continue
                
            # Compute correlation for each common channel, then average
            corrs = []
            for ch in common_chs:
                c1 = f"{ch}_{ft1}"
                c2 = f"{ch}_{ft2}"
                r = X[c1].corr(X[c2], method='spearman')
                if not pd.isna(r):
                    corrs.append(abs(r))
                    
            if corrs:
                avg_corr = np.mean(corrs)
                corr_matrix_types.loc[ft1, ft2] = avg_corr
                corr_matrix_types.loc[ft2, ft1] = avg_corr
            else:
                corr_matrix_types.loc[ft1, ft2] = 0.0
                corr_matrix_types.loc[ft2, ft1] = 0.0
                
    # Select upper triangle of correlation matrix
    upper = corr_matrix_types.where(np.triu(np.ones(corr_matrix_types.shape), k=1).astype(bool))
    
    # Find feature TYPES with correlation greater than threshold
    to_drop_types = [column for column in upper.columns if any(upper[column] > CORRELATION_THRESHOLD)]
    
    # Save correlated pairs report
    report_file = run_dir / "correlated_features_report.txt"
    with open(report_file, "w") as f:
        f.write("=" * 70 + "\n")
        f.write("HIGHLY CORRELATED FEATURE TYPES REPORT\n")
        f.write("=" * 70 + "\n")
        f.write(f"Correlation metric: Average Spearman rank correlation across overlapping channels\n")
        f.write(f"Threshold: {CORRELATION_THRESHOLD}\n\n")
        
        correlated_pairs = []
        for col in upper.columns:
            for idx in upper.index:
                val = upper.loc[idx, col]
                if not pd.isna(val) and val > CORRELATION_THRESHOLD:
                    correlated_pairs.append((idx, col, val))
        
        correlated_pairs.sort(key=lambda x: x[2], reverse=True)
        
        f.write(f"Found {len(correlated_pairs)} highly correlated feature type pairs.\n\n")
        f.write(f"{'Feature Type 1':<28} | {'Feature Type 2':<28} | {'Avg Correlation':<15}\n")
        f.write("-" * 80 + "\n")
        for f1, f2, val in correlated_pairs:
            f.write(f"{f1:<28} | {f2:<28} | {val:.4f}\n")
            
        f.write("\nFeature TYPES to drop due to redundancy (You can disable these in config_model.py):\n")
        for drop_feat in sorted(to_drop_types):
            f.write(f"- {drop_feat}\n")
            
    print(f"Found {len(correlated_pairs)} pairs of highly correlated feature types (|r| > {CORRELATION_THRESHOLD}).")
    print(f"Identified {len(to_drop_types)} redundant feature types.")
    print(f"Correlation report saved to: {report_file}")
    
    # Map back to actual column names to drop
    cols_to_drop = []
    for dt in to_drop_types:
        cols_to_drop.extend(type_to_cols[dt])
        
    return cols_to_drop

def main():
    if MODEL_TYPE.lower() not in ["svr", "rf", "gb"]:
        raise ValueError(f"Model type '{MODEL_TYPE}' does not support scikit-learn's standard RFE. "
                         f"Please use one of: 'svr', 'rf', 'gb'.")

    # Define paths
    base_dir = Path(__file__).parent.parent
    segments_dir = base_dir / "database" / "segments"
    results_dir = base_dir / "model" / "model_results"
    
    # Create results directory if it doesn't exist
    results_dir.mkdir(parents=True, exist_ok=True)
    
    # Create a unique timestamped folder for this run's results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"rfe_{MODEL_TYPE}_{timestamp}"
    run_dir.mkdir(exist_ok=True)
    
    print(f"RFE Results will be saved to: {run_dir}")
    
    # Get all h5 segments files
    h5_paths = list(segments_dir.glob("*.h5"))
    
    if not h5_paths:
        print(f"No HDF5 segment files found in {segments_dir}.")
        return

    print(f"Found {len(h5_paths)} segment file(s).")
    
    # Load and extract features
    loader = DataLoader()
    print("Extracting features from HDF5 files. This may take a moment...")
    
    # DataLoader defaults to window properties in FEATURE_CONFIG
    df = loader.load_and_extract_features(h5_paths, is_sequence=False)
    
    if df.empty:
        print("Data extraction failed or produced an empty DataFrame.")
        return
        
    print(f"Extracted features dataframe shape: {df.shape}")
    
    target_col = "weight"
    X, y = loader.prepare_for_ml(df, target_col=target_col)
    
    print(f"Initial feature matrix (X) shape: {X.shape}")
    print(f"Label vector (y) shape: {y.shape} (Target: {target_col})")
    
    # Step 1: Correlation Analysis
    to_drop_cols = perform_correlation_analysis(X, run_dir)
    
    if to_drop_cols:
        print(f"Dropping {len(to_drop_cols)} correlated raw columns before RFE...")
        X = X.drop(columns=to_drop_cols)
        print(f"Reduced raw feature matrix shape: {X.shape}")

    # Step 1.5: Group by Feature Type (Averaging across channels)
    print("\n--- Grouping Features by Type (Averaging across channels) ---")
    def get_feature_type(col_name):
        m = re.match(r'^.+?_((?:EMG|IMU)_.*)$', col_name)
        return m.group(1) if m else col_name

    type_to_cols = defaultdict(list)
    for col in X.columns:
        ftype = get_feature_type(col)
        type_to_cols[ftype].append(col)
    
    feature_types = sorted(type_to_cols.keys())
    X_types = pd.DataFrame(index=X.index)
    
    for ftype in feature_types:
        # Create a brand new column which is the mean of all channel variants for this type
        cols = type_to_cols[ftype]
        X_types[ftype] = X[cols].mean(axis=1)
        
    print(f"Collapsed {X.shape[1]} raw columns into {len(feature_types)} feature types.")
    print(f"Feature Type matrix shape: {X_types.shape}")

    # Step 2: Scale data completely
    print("\n--- Scaling Features ---")
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X_types)
    X_scaled = pd.DataFrame(X_scaled, columns=feature_types, index=X_types.index)
    
    # Step 3: Initialize model and extract underlying scikit-learn estimator
    print(f"\n--- Initializing {MODEL_TYPE.upper()} Estimator ---")
    model_wrapper = initialize_model(MODEL_TYPE)
    if model_wrapper is None:
        print("Model initialization failed.")
        return
    
    estimator = model_wrapper.model
    print(f"Underlying estimator: {type(estimator).__name__}")
    
    # Step 4: Perform RFECV
    print("\n--- Running Recursive Feature Elimination on Feature TYPES ---")
    
    # Stratify by weights for balanced CV folds
    strat_labels = df["weight"].astype(str) if "weight" in df.columns else None
    cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=GLOBAL_RANDOM_STATE if 'GLOBAL_RANDOM_STATE' in globals() else 42)
    
    step_size = 1  # drop 1 feature per step
    min_features = 1
    scoring_metric = "neg_mean_absolute_error"
    
    print(f"CV: {cv.n_splits}-fold StratifiedKFold")
    print(f"Scoring: {scoring_metric}")
    print(f"Step size (feature types dropped iteratively): {step_size}")
    
    rfecv = RFECV(
        estimator=estimator,
        step=step_size,
        cv=list(cv.split(X_scaled, strat_labels)),
        scoring=scoring_metric,
        min_features_to_select=min_features,
        n_jobs=-1,
    )
    
    print("Fitting RFECV... (this will be fast as we only have ~40 inputs)")
    rfecv.fit(X_scaled, y)
    
    print(f"RFECV Optimal number of feature types: {rfecv.n_features_}")
    
    # Step 5: Save Results & Rank
    print("\n--- Saving RFE Results ---")
    
    ranking_df = pd.DataFrame({
        "Feature_Type": feature_types,
        "Selected": rfecv.support_,
        "Rank": rfecv.ranking_
    })
    
    ranking_df = ranking_df.sort_values(by="Rank")
    
    # Save CSV
    ranking_csv = run_dir / "rfe_feature_ranking.csv"
    ranking_df.to_csv(ranking_csv, index=False)
    
    # Save a human-readable text file
    ranking_txt = run_dir / "rfe_feature_ranking.txt"
    with open(ranking_txt, "w") as f:
        f.write("=" * 60 + "\n")
        f.write(f"RFECV FEATURE TYPE RANKING REPORT\n")
        f.write(f"Model: {MODEL_TYPE.upper()} ({type(estimator).__name__})\n")
        f.write(f"Timestamp: {timestamp}\n")
        f.write("=" * 60 + "\n\n")
        
        f.write(f"Initial raw features before correlation drop: {df.shape[1]}\n")
        f.write(f"Raw features dropped due to correlation (|r| > {CORRELATION_THRESHOLD}): {len(to_drop_cols)}\n")
        f.write(f"Feature Types evaluated in RFE: {len(feature_types)}\n")
        f.write(f"RFE Optimal number of feature types: {rfecv.n_features_}\n\n")
        
        f.write("OPTIMAL FEATURE TYPES (Enable these in config_model.py):\n")
        f.write("-" * 60 + "\n")
        
        selected = ranking_df[ranking_df["Selected"]]
        for _, row in selected.iterrows():
            f.write(f"- {row['Feature_Type']}\n")
            
        f.write("\nNON-SELECTED FEATURE TYPES (By Rank):\n")
        f.write("-" * 60 + "\n")
        rejected = ranking_df[~ranking_df["Selected"]]
        for _, row in rejected.iterrows():
            f.write(f"Rank {row['Rank']:<4} | {row['Feature_Type']}\n")
            
    # Step 6: Plot CV Score across feature numbers
    plot_file = run_dir / "rfecv_score_vs_features.png"
    plt.figure(figsize=(10, 6))
    plt.xlabel("Number of feature types selected")
    plt.ylabel(f"Cross-validation score ({scoring_metric})")
    plt.title(f"RFECV (Types): {MODEL_TYPE.upper()} - Optimal types: {rfecv.n_features_}")
    
    # Extract scores depending on sklearn version
    if hasattr(rfecv, 'cv_results_'):
        mean_test_scores = rfecv.cv_results_['mean_test_score']
    else:
        mean_test_scores = rfecv.grid_scores_
        
    plt.plot(
        range(min_features, len(mean_test_scores) + min_features),
        mean_test_scores,
        marker='o'
    )
    
    plt.grid(True, linestyle="--", alpha=0.7)
    plt.tight_layout()
    plt.savefig(plot_file, dpi=300)
    plt.close()
    
    print(f"RFE Type Ranking saved to: {ranking_txt}")
    print(f"RFECV Plot saved to: {plot_file}")
    print(f"Done!\n")

if __name__ == "__main__":
    main()