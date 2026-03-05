import pandas as pd
from pathlib import Path
from sklearn.svm import SVC
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import classification_report, accuracy_score
import joblib

from model.data_loader import DataLoader
from model.config_model import SVM_CONFIG


class SVMClassifier:
    """
    SVMClassifier wraps the data scaling and SVM model training into a single class.
    It utilizes scikit-learn's SVC and StandardScaler.
    """
    def __init__(self, 
                 kernel: str = SVM_CONFIG['kernel'], 
                 C: float = SVM_CONFIG['C'], 
                 gamma: str | float = SVM_CONFIG.get('gamma', 'scale'),
                 class_weight: str | dict | None = SVM_CONFIG.get('class_weight'),
                 random_state: int = SVM_CONFIG['random_state']):
        self.kernel = kernel
        self.C = C
        self.gamma = gamma
        self.class_weight = class_weight
        self.random_state = random_state
        self.scaler = StandardScaler()
        self.model = SVC(
            kernel=self.kernel, 
            C=self.C, 
            gamma=self.gamma,
            class_weight=self.class_weight,
            random_state=self.random_state, 
            probability=True
        )
        
    def fit(self, X: pd.DataFrame, y: pd.Series):
        """Fit the scaler on features and train the SVM."""
        X_scaled = self.scaler.fit_transform(X)
        self.model.fit(X_scaled, y)
        
    def predict(self, X: pd.DataFrame):
        """Scale test data based on fitted scaler and return predictions."""
        X_scaled = self.scaler.transform(X)
        return self.model.predict(X_scaled)

    def predict_proba(self, X: pd.DataFrame):
        """Scale test data and return prediction probabilities."""
        X_scaled = self.scaler.transform(X)
        return self.model.predict_proba(X_scaled)
        
    def evaluate(self, X_test: pd.DataFrame, y_test: pd.Series):
        """Evaluate the custom test set and return accuracy and classification report."""
        y_pred = self.predict(X_test)
        acc = accuracy_score(y_test, y_pred)
        report = classification_report(y_test, y_pred)
        return acc, report
        
    def save(self, filepath: str | Path):
        """Save the model and scaler using joblib."""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        joblib.dump({'model': self.model, 'scaler': self.scaler}, filepath)
        print(f"Model saved to {filepath}")
        
    @classmethod
    def load(cls, filepath: str | Path):
        """Load a saved model and scaler from joblib."""
        data = joblib.load(filepath)
        classifier = cls()
        classifier.model = data['model']
        classifier.scaler = data['scaler']
        return classifier



