from tensorflow.keras.datasets import mnist
from sklearn.metrics import confusion_matrix, accuracy_score, precision_score, recall_score, f1_score
import numpy as np
import pandas as pd
from scipy.stats import mode

# Load the MNIST dataset
(train_X, train_y), (test_X, test_y) = mnist.load_data()

# Set the sample size
sample_size = 5000

# Randomly select 5000 indices for train and test data
train_indices = np.random.choice(len(train_X), sample_size, replace=False)
test_indices = np.random.choice(len(test_X), sample_size, replace=False)

# Subset the data
small_train_X = train_X[train_indices]
small_train_y = train_y[train_indices]
small_test_X = test_X[test_indices]
small_test_y = test_y[test_indices]

# Output the shapes to verify
print("Small training set shape:", small_train_X.shape, small_train_y.shape)
print("Small testing set shape:", small_test_X.shape, small_test_y.shape)

# Reshape and normalize the data
small_train_X = small_train_X.reshape(small_train_X.shape[0], -1) / 255.0
small_test_X = small_test_X.reshape(small_test_X.shape[0], -1) / 255.0

def knn_classify_with_metrics(train_X, train_y, test_X, k, test_y):
    """
    Classifies test set according to the k-Nearest Neighbors (kNN) rule and computes evaluation metrics.
    
    Parameters:
        train_X (ndarray): Training set feature matrix
        train_y (ndarray): Training set labels
        test_X (ndarray): Test set feature matrix
        test_y (ndarray): True labels for the test set
        k (int): Number of nearest neighbors to use for classification
        
    Returns:
        test_predictions (ndarray): Predicted labels for the test set
        metrics (dict): A dictionary containing evaluation metrics
                        (Accuracy, Precision, Recall, F1 Score)
    """
    # Check that k > 0 and k <= cardinality of the training set
    n = train_X.shape[0]
    if k <= 0 or k > n:
        raise ValueError(f"k must be greater than 0 and less than or equal to {n}")

    # Check that the number of columns in test_X equals the number of columns in train_X
    if train_X.shape[1] != test_X.shape[1]:
        raise ValueError("Number of columns in test set must match the training set")

    # Perform kNN classification
    test_predictions = []
    for test_sample in test_X:
        # Compute distances from test_sample to all training samples
        distances = np.linalg.norm(train_X - test_sample, axis=1)
        
        # Get the indices of the k nearest neighbors
        neighbor_indices = np.argsort(distances)[:k]
        
        # Get the labels of the k nearest neighbors
        neighbor_labels = train_y[neighbor_indices]
        
        # Determine the most frequent label (mode) among the neighbors
        predicted_label = mode(neighbor_labels, axis=None).mode  # Ensure axis=None for scalar input
        predicted_label = predicted_label[0] if isinstance(predicted_label, np.ndarray) else predicted_label
        
        # Append the predicted label to the list
        test_predictions.append(predicted_label)
    
    # Convert predictions to a NumPy array
    test_predictions = np.array(test_predictions)

    # Compute evaluation metrics
    if test_y is not None and len(test_y) > 0:  # Ensure test_y is not None and not empty
        accuracy = accuracy_score(test_y, test_predictions)
        precision = precision_score(test_y, test_predictions, average='weighted')
        recall = recall_score(test_y, test_predictions, average='weighted')
        f1 = f1_score(test_y, test_predictions, average='weighted')
    else:
        accuracy = 0
        precision = 0
        recall = 0
        f1 = 0

    

    # Store metrics in a dictionary
    metrics = {
        "Accuracy": accuracy,
        "Precision": precision,
        "Recall": recall,
        "F1": f1
    }

    return test_predictions, metrics

def compute_error_rate(predictions, targets):
    """
    Computes the error rate of predictions against the true targets.
    
    Parameters:
        predictions (ndarray): Predicted labels
        targets (ndarray): Actual labels
        
    Returns:
        error_rate (float): Error rate calculated as number of errors divided by number of samples
    """
    errors = np.sum(predictions != targets)
    error_rate = errors / len(targets)
    return error_rate

def save_predictions_to_csv(predictions, output_file="predictions.csv"):
    """
    Saves predictions to a CSV file.
    
    Parameters:
        predictions (ndarray or list): Predicted labels for the test set
        output_file (str): Name of the output CSV file (default: "predictions.csv")
    """
    # Create a DataFrame with predictions
    df = pd.DataFrame(predictions, columns=["Prediction"])
    
    # Save the DataFrame to a CSV file
    df.to_csv(output_file, index=False)
    print(f"Predictions saved to {output_file}")


k = 10
# Classify the test set
predictions, metrics = knn_classify_with_metrics(small_train_X, small_train_y, small_test_X, k, small_test_y)

error_rate = compute_error_rate(predictions, small_test_y)
print(f"Error Rate: {error_rate}")

# Compute confusion matrix
conf_matrix = confusion_matrix(small_test_y, predictions)
print("Confusion Matrix:")
print(conf_matrix)

total_samples = np.sum(conf_matrix)

# Initialize lists for metrics
precisions = []
recalls = []
accuracies = []

# Compute metrics for each class
for i in range(conf_matrix.shape[0]):
    TP = conf_matrix[i, i]
    FP = np.sum(conf_matrix[:, i]) - TP
    FN = np.sum(conf_matrix[i, :]) - TP
    TN = total_samples - (TP + FP + FN)
    
    # Calculate precision, recall, and accuracy
    precision = TP / (TP + FP) if TP + FP > 0 else 0
    recall = TP / (TP + FN) if TP + FN > 0 else 0
    accuracy = (TP + TN) / total_samples if total_samples > 0 else 0
    
    # Append metrics
    precisions.append(precision)
    recalls.append(recall)
    accuracies.append(accuracy)

metrics = {
    "Accuracy": accuracies,
    "Precision": precisions,
    "Recall": recalls,
}
    
# Compute statistics
summary = {metric: {
    "Mean": np.mean(values),
    "Std Dev": np.std(values),
    "25th Percentile": np.percentile(values, 25),
    "75th Percentile": np.percentile(values, 75)
} for metric, values in metrics.items()}

# Print summary
summary_df = pd.DataFrame(summary).T
print(summary_df)

save_predictions_to_csv(predictions, output_file="predictions_minst_1.csv")