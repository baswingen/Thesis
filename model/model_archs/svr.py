import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from sklearn.svm import SVR
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import joblib


def _sanitise(arr: np.ndarray, clip: float = 1e9) -> np.ndarray:
    """Replace NaN/Inf with 0 and clip abs-values to `clip` to prevent
    float64 overflow inside sklearn's StandardScaler."""
    arr = np.where(np.isfinite(arr), arr, 0.0)
    return np.clip(arr, -clip, clip)

from model.config_model import SVR_CONFIG
from model import plotting_utils

class SVRRegressor:
    """
    SVRRegressor wraps the data scaling and Support Vector Regression model 
    training into a single class, matching the existing model infrastructure.
    """
    def __init__(self, 
                 kernel: str = SVR_CONFIG['kernel'], 
                 C: float = SVR_CONFIG['C'], 
                 epsilon: float = SVR_CONFIG['epsilon'],
                 gamma: str | float = SVR_CONFIG.get('gamma', 'scale'),
                 random_state: int = SVR_CONFIG['random_state'],
                 **kwargs):
        self.kernel = kernel
        self.C = C
        self.epsilon = epsilon
        self.gamma = gamma
        self.random_state = random_state
        
        self.scaler = StandardScaler()
        self.model = SVR(
            kernel=self.kernel, 
            C=self.C, 
            epsilon=self.epsilon,
            gamma=self.gamma
        )
        # Will be set during fit() — used to clip predictions to the training range
        self.y_train_max: float = np.inf
        
        
    def fit(self, X: pd.DataFrame, y: pd.Series, sample_weight: np.ndarray | None = None):
        """Fit the scaler on features and train the SVR."""
        self.y_train_max = float(y.max())
        X_scaled = self.scaler.fit_transform(_sanitise(X.values))
        self.model.fit(X_scaled, y, sample_weight=sample_weight)
        
    def predict(self, X: pd.DataFrame):
        """Scale test data based on fitted scaler and return clipped predictions.
        
        Predictions are clipped to [0, y_train_max] to prevent the SVR from
        extrapolating to physically impossible values for out-of-distribution
        participants (e.g. hundreds or thousands of kg in a 0–6 kg task).
        """
        X_scaled = self.scaler.transform(_sanitise(X.values))
        y_pred = self.model.predict(X_scaled)
        # Clip to [0, training max] — weight cannot be negative or beyond what
        # the model was trained to predict.
        return np.clip(y_pred, 0.0, self.y_train_max)
        
    def evaluate(self, X_test: pd.DataFrame, y_test: pd.Series):
        """
        Evaluate the regressor on the test set and return metrics.
        Returns a dictionary containing MAE, MSE, RMSE, and R2.
        """
        y_pred = self.predict(X_test)
        mae = mean_absolute_error(y_test, y_pred)
        mse = mean_squared_error(y_test, y_pred)
        rmse = np.sqrt(mse)
        r2 = r2_score(y_test, y_pred)
        
        metrics = {
            "MAE": mae,
            "MSE": mse,
            "RMSE": rmse,
            "R2": r2
        }
        
        report_str = (
            f"Mean Absolute Error: {mae:.4f}\n"
            f"Mean Squared Error: {mse:.4f}\n"
            f"Root Mean Squared Error: {rmse:.4f}\n"
            f"R-squared Score: {r2:.4f}\n"
        )
        
        return metrics, report_str
        
    def save(self, filepath: str | Path):
        """Save the model and scaler using joblib."""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        joblib.dump({'model': self.model, 'scaler': self.scaler, 'y_train_max': self.y_train_max}, filepath)
        print(f"SVR Model saved to {filepath}")
        
    @classmethod
    def load(cls, filepath: str | Path):
        """Load a saved model and scaler from joblib."""
        data = joblib.load(filepath)
        regressor = cls()
        regressor.model = data['model']
        regressor.scaler = data['scaler']
        regressor.y_train_max = data.get('y_train_max', np.inf)
        return regressor

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        plotting_utils.plot_regression_results(y_test, y_pred, save_path, model_name="SVR")
        print(f"Regression plot saved to {save_path}")
