import pandas as pd
import numpy as np
from pathlib import Path
from sklearn.cluster import KMeans
from sklearn.metrics.pairwise import rbf_kernel
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import classification_report, accuracy_score
import joblib

from model.data_loader import DataLoader
from model.config_model import RBFNN_CONFIG

class RBFNNClassifier:
    """
    Radial Basis Function Neural Network (RBFNN) implementation using scikit-learn.
    
    This class combines scaling, prototype center selection via KMeans, RBF activation,
    and a linear output layer (Logistic Regression) into a single interface compatible
    with the surrounding model infrastructure.
    """
    def __init__(self, 
                 n_centers: int = RBFNN_CONFIG['n_centers'], 
                 gamma: float = RBFNN_CONFIG['gamma'], 
                 C: float = RBFNN_CONFIG['C'], 
                 random_state: int = RBFNN_CONFIG['random_state'],
                 class_weight: str | dict | None = RBFNN_CONFIG.get('class_weight')):
        self.n_centers = n_centers
        self.gamma = gamma
        self.C = C
        self.random_state = random_state
        self.class_weight = class_weight
        
        self.scaler = StandardScaler()
        self.kmeans = KMeans(n_clusters=self.n_centers, random_state=self.random_state, n_init=10)
        # We use a linear classifier for the output layer. 
        # class_weight handles imbalance by penalizing errors on minority classes more.
        self.output_layer = LogisticRegression(
            C=self.C, 
            max_iter=1000, 
            random_state=self.random_state,
            class_weight=self.class_weight
        )
        
        self.centers_ = None

    def _get_rbf_features(self, X_scaled: np.ndarray) -> np.ndarray:
        """Calculate the RBF kernel features (activations) given scaled input data."""
        return rbf_kernel(X_scaled, self.centers_, gamma=self.gamma)

    def fit(self, X: pd.DataFrame, y: pd.Series):
        """Fit the scaler, find RBF centers, map inputs to hidden layer, and train output layer."""
        # 1. Scale data
        X_scaled = self.scaler.fit_transform(X)
        
        # 2. Find prototype centers using KMeans
        self.kmeans.fit(X_scaled)
        self.centers_ = self.kmeans.cluster_centers_
        
        # 3. Compute hidden layer activations (distances to centers)
        H = self._get_rbf_features(X_scaled)
        
        # 4. Train the output layer
        self.output_layer.fit(H, y)
        
    def predict(self, X: pd.DataFrame):
        """Scale test data, get RBF features, and return predictions."""
        X_scaled = self.scaler.transform(X)
        H = self._get_rbf_features(X_scaled)
        return self.output_layer.predict(H)

    def predict_proba(self, X: pd.DataFrame):
        """Scale test data, get RBF features, and return prediction probabilities."""
        X_scaled = self.scaler.transform(X)
        H = self._get_rbf_features(X_scaled)
        return self.output_layer.predict_proba(H)
        
    def evaluate(self, X_test: pd.DataFrame, y_test: pd.Series):
        """Evaluate the custom test set and return accuracy and classification report."""
        y_pred = self.predict(X_test)
        acc = accuracy_score(y_test, y_pred)
        report = classification_report(y_test, y_pred)
        return acc, report
        
    def save(self, filepath: str | Path):
        """Save the model components using joblib."""
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        joblib.dump({
            'scaler': self.scaler,
            'kmeans': self.kmeans,
            'centers_': self.centers_,
            'output_layer': self.output_layer,
            'n_centers': self.n_centers,
            'gamma': self.gamma,
            'C': self.C,
            'random_state': self.random_state,
            'class_weight': self.class_weight
        }, filepath)
        print(f"RBFNN Model saved to {filepath}")
        
    @classmethod
    def load(cls, filepath: str | Path):
        """Load a saved RBFNN model from joblib."""
        data = joblib.load(filepath)
        classifier = cls(
            n_centers=data['n_centers'],
            gamma=data['gamma'], 
            C=data['C'],
            random_state=data['random_state'],
            class_weight=data.get('class_weight')
        )
        classifier.scaler = data['scaler']
        classifier.kmeans = data['kmeans']
        classifier.centers_ = data['centers_']
        classifier.output_layer = data['output_layer']
        return classifier

if __name__ == "__main__":
    # Example usage / basic test
    sample_file = Path(__file__).parent.parent / "database" / "segments" / "participant_P01_session_01_segments.h5"
    
    if sample_file.exists():
        print(f"Loading from: {sample_file}")
        loader = DataLoader()
        
        # Extract features
        df = loader.load_and_extract_features([sample_file])
        
        if not df.empty:
            # Prepare ML structures
            X, y = loader.prepare_for_ml(df, target_col="label")
            
            # Simple train/test split
            from sklearn.model_selection import train_test_split
            X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
            
            print(f"Training on {len(X_train)} samples, testing on {len(X_test)} samples.")
            
            # Initialize and Train RBFNN
            rbfnn = RBFNNClassifier(n_centers=10, gamma=0.1, C=1.0)
            print(f"Training RBFNNClassifier (n_centers={rbfnn.n_centers}, gamma={rbfnn.gamma}, C={rbfnn.C})...")
            rbfnn.fit(X_train, y_train)
            
            # Evaluate
            acc, report = rbfnn.evaluate(X_test, y_test)
            print(f"\nModel Accuracy: {acc:.4f}")
            print("Classification Report:")
            print(report)
            
            # Test Save and Load
            test_path = Path(__file__).parent.parent / "model_results" / "test_rbfnn.joblib"
            rbfnn.save(test_path)
            
            loaded_rbfnn = RBFNNClassifier.load(test_path)
            loaded_acc, _ = loaded_rbfnn.evaluate(X_test, y_test)
            print(f"Loaded Model Accuracy: {loaded_acc:.4f}")
            assert acc == loaded_acc
            print("Save and Load successful.")
            
            # Cleanup test save
            test_path.unlink(missing_ok=True)
            
    else:
        print(f"Could not perform test, sample file {sample_file} not found.")
