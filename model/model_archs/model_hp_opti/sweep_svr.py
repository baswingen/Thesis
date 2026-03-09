import sys
from pathlib import Path
from datetime import datetime
import pandas as pd
from sklearn.model_selection import train_test_split, RandomizedSearchCV

# Add project root to sys.path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent.parent))

from model.data_loader import DataLoader
from model.model_archs.svr import SVRRegressor
from model.config_model import SVR_CONFIG

def main():
    base_dir = Path(__file__).resolve().parent.parent.parent.parent
    segments_dir = base_dir / "database" / "segments"
    results_dir = base_dir / "model" / "model_results"
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = results_dir / f"sweep_svr_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Starting SVR hyperparameter sweep...")
    print(f"Results will be saved to: {run_dir}\n")
    
    # 1. Load Data
    h5_paths = list(segments_dir.glob("*.h5"))
    if not h5_paths:
        print(f"No HDF5 segment files found in {segments_dir}.")
        return

    loader = DataLoader()
    print("Extracting features (This only happens once for the sweep)...")
    
    # SVR doesn't need sequence dicts
    df = loader.load_and_extract_features(h5_paths)
    
    if df.empty:
        print("Data extraction failed.")
        return
        
    X, y = loader.prepare_for_ml(df, target_col="weight")
    
    # 2. Train/Test Split
    stratify = df["label"] if "label" in df.columns else None
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=7, stratify=stratify
    )
    
    print(f"\nTraining on {len(X_train)} samples, testing on {len(X_test)} samples.\n")
    
    # 3. Define Parameter Grid and Base Model
    param_grid = {
        'model__C': [0.1, 1, 10, 100, 1000],
        'model__epsilon': [0.01, 0.1, 0.2, 0.5],
        'model__gamma': ['scale', 'auto', 0.001, 0.01, 0.1],
        'model__kernel': ['rbf', 'linear', 'poly', 'sigmoid']
    }
    
    from sklearn.pipeline import Pipeline
    from sklearn.preprocessing import StandardScaler
    from sklearn.svm import SVR
    
    # Create a proper sklearn pipeline since RandomizedSearchCV needs it
    pipeline = Pipeline([
        ('scaler', StandardScaler()),
        ('model', SVR())
    ])
    
    # We apply RandomizedSearchCV directly on the pipeline
    search = RandomizedSearchCV(
        estimator=pipeline,
        param_distributions=param_grid,
        n_iter=20, # Number of random combinations
        scoring='neg_mean_absolute_error',
        cv=3,      # 3-fold internal CV for the sweep
        random_state=42,
        n_jobs=-1, # Use all cores
        verbose=1
    )
    
    print("Running Randomized Search...")
    search.fit(X_train, y_train)
    
    # 4. Save best run results
    best_params = search.best_params_
    
    # Remove the 'model__' prefix from the best_params
    cleaned_params = {k.replace('model__', ''): v for k, v in best_params.items()}
    
    print("\n" + "="*50)
    print("SWEEP COMPLETE")
    print("="*50)
    print("Best Parameters:")
    for k, v in cleaned_params.items():
        print(f"  {k}: {v}")
    
    # Retrain on the best parameters manually since the wrapper expects this
    best_model = SVRRegressor(
        C=cleaned_params.get('C', 1.0),
        epsilon=cleaned_params.get('epsilon', 0.1),
        gamma=cleaned_params.get('gamma', 'scale'),
        kernel=cleaned_params.get('kernel', 'rbf'),
        random_state=SVR_CONFIG.get('random_state', 42)
    )
    
    best_model.fit(X_train, y_train)
    metrics, report_str = best_model.evaluate(X_test, y_test)
    
    print("\nBest Metrics on Test Set:")
    print(report_str)
    print("="*50)
    
    # Save sweep details to text file
    report_file = run_dir / "sweep_report.txt"
    with open(report_file, "w") as f:
        f.write("SVR HYPERPARAMETER SWEEP REPORT\n")
        f.write("="*50 + "\n\n")
        f.write("BEST PARAMETERS:\n")
        for k, v in cleaned_params.items():
            f.write(f"{k}: {v}\n")
        f.write("\nBEST EVALUATION METRICS (On Hold-out set):\n")
        f.write(report_str)
        f.write("\n" + "-"*50 + "\n")
        f.write("ALL CONFIGURATIONS TESTED (Ranked by cross-validated MAE):\n")
        
        results_df = pd.DataFrame(search.cv_results_).sort_values('rank_test_score')
        for idx, row in results_df.iterrows():
            f.write(f"\nRank {row['rank_test_score']}\n")
            f.write(f"Params: {row['params']}\n")
            f.write(f"Mean Test Score (Negative MAE): {row['mean_test_score']:.4f}\n")

    # Save best model
    model_path = run_dir / "best_svr_model.joblib"
    best_model.save(model_path)
    print(f"Report and best model saved to {run_dir}")

if __name__ == "__main__":
    main()
