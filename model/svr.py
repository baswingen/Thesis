import pandas as pd
import numpy as np
from pathlib import Path
from sklearn.svm import SVR
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import joblib

from model.config_model import SVR_CONFIG

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
                 random_state: int = SVR_CONFIG['random_state']):
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
        
    def fit(self, X: pd.DataFrame, y: pd.Series):
        """Fit the scaler on features and train the SVR."""
        X_scaled = self.scaler.fit_transform(X)
        self.model.fit(X_scaled, y)
        
    def predict(self, X: pd.DataFrame):
        """Scale test data based on fitted scaler and return continuous predictions."""
        X_scaled = self.scaler.transform(X)
        return self.model.predict(X_scaled)
        
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
        joblib.dump({'model': self.model, 'scaler': self.scaler}, filepath)
        print(f"SVR Model saved to {filepath}")
        
    @classmethod
    def load(cls, filepath: str | Path):
        """Load a saved model and scaler from joblib."""
        data = joblib.load(filepath)
        regressor = cls()
        regressor.model = data['model']
        regressor.scaler = data['scaler']
        return regressor
