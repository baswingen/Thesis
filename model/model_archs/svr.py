import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
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
        """Scale test data based on fitted scaler and return non-negative predictions."""
        X_scaled = self.scaler.transform(X)
        y_pred = self.model.predict(X_scaled)
        # Weight cannot be negative
        return np.maximum(0.0, y_pred)
        
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

    def plot_results(self, y_test: pd.Series, y_pred: np.ndarray, save_path: str | Path):
        """
        Creates a 'Predicted vs Actual' box plot and saves it to a file.
        Positions boxes according to their actual weight values.
        """
        plt.figure(figsize=(10, 7))
        
        # Extract unique actual weights and group predictions
        actual_weights = np.sort(np.unique(y_test))
        pred_groups = [y_pred[y_test == w] for w in actual_weights]
        
        # Use actual values for positions to ensure correct spacing along the axis
        positions = actual_weights
        
        # Create boxplot
        # Note: boxplot 'positions' can be any list of floats
        bp = plt.boxplot(pred_groups, positions=positions, widths=0.15, patch_artist=True, manage_ticks=False)
        for box in bp['boxes']:
            box.set(facecolor='royalblue', alpha=0.7)
        
        # Unity line (Actual = Predicted)
        # Since we are using actual values for positions, the diagonal x=y is the perfect prediction line
        min_val, max_val = -0.5, 7.5
        plt.plot([min_val, max_val], [min_val, max_val], 'r--', lw=1.5, alpha=0.6, label='Perfect Prediction')
        
        # Add a light grid for better readability
        plt.grid(True, linestyle='--', alpha=0.4)
        
        # Explicitly set ticks to actual weight values for clarity
        plt.xticks(actual_weights, [f"{w:.2g}" for w in actual_weights])
        
        # Metrics annotation
        r2 = r2_score(y_test, y_pred)
        mae = mean_absolute_error(y_test, y_pred)
        plt.text(0.05, 0.95, f"$R^2 = {r2:.3f}$\n$MAE = {mae:.3f}$ kg", 
                 transform=plt.gca().transAxes, verticalalignment='top',
                 bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray'))
        
        plt.xlabel("Actual Weight (kg)")
        plt.ylabel("Predicted Weight (kg)")
        plt.title(f"SVR Regression Performance (Kernel: {self.kernel}, C={self.C})")
        
        # Set symmetric limits for better interpretation of the diagonal
        plt.xlim(min_val, max_val)
        plt.ylim(min_val, max_val)
        
        plt.legend(loc='lower right')
        plt.tight_layout()
        
        save_path = Path(save_path)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Regression plot saved to {save_path}")
