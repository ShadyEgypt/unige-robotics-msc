import pandas as pd
from sklearn.preprocessing import LabelEncoder
from sklearn.model_selection import train_test_split
import numpy as np

def label_encode_columns(df, columns):
    """
    Label encodes the specified columns in a pandas DataFrame.

    Parameters:
    df (pd.DataFrame): The DataFrame to encode.
    columns (list): A list of column names to label encode.

    Returns:
    pd.DataFrame: DataFrame with label encoded columns.
    """
    le = LabelEncoder()
    
    for col in columns:
        if df[col].dtype == 'object':  # Ensuring it's a categorical column
            df[col] = le.fit_transform(df[col])
    
    return df

class NaiveBayesClassifier:
    def __init__(self):
        self.prior_probs = {}    # Store prior probabilities for each class
        self.likelihoods = {}    # Store likelihoods for each feature value per class

    def fit(self, X, y):
        # Get unique classes
        self.classes = np.unique(y)

        # Calculate prior probabilities for each class
        print(f"N: number of observations in the train dataset = {len(y)}")
        print(f"n_i: count of values of each class type")
        print("\n")
        for cls in self.classes:
            self.prior_probs[cls] = np.sum(y == cls) / len(y)
            print(f"Prior probability for class {self.classes[cls]}: n_{cls} ({np.sum(y == cls)}) / N ({len(y)}) = {np.sum(y == cls) / len(y)}")
        print("\n")
        # Calculate likelihoods for each feature given the class
        print(f"N: number of feature values given a specific class")
        print(f"n_i: count of values of each feature type given a class")
        print("\n")
        for cls in self.classes:
            self.likelihoods[cls] = {}
            class_data = X[y == cls]  # Subset of data where y == cls
            print(f"Let's calculate likelihoods for all features given class {self.classes[cls]}\n")
            for feature in X.columns:
                self.likelihoods[cls][feature] = {}
                print(f"feature {feature}")
                feature_vals = X[feature].unique()  # Unique values for the feature
                print(f"for this feature, we have {len(feature_vals)} types.")
                
                for val in feature_vals:
                    # P(feature | class) = (count of feature value in class) / (total count of class)
                    count = np.sum(class_data[feature] == val)
                    self.likelihoods[cls][feature][val] = count / len(class_data)
                    print(f"P(feature={val} | class={cls} ) = count of a feature type ({count}) / total count of class ({len(class_data)}) = {count / len(class_data)}")
                print(f"\n")
        print("\n")
    
    def predict(self, X_test):
        predictions = []
        for _, row in X_test.iterrows():
            # For each test instance, calculate P(class | features) for each class
            prior_probs = {}

            for cls in self.classes:
                # Start with prior probability
                prior_probs[cls] = self.prior_probs[cls]

                # Multiply by likelihoods for each feature
                for feature in X_test.columns:
                    feature_val = row[feature]
                    if feature_val in self.likelihoods[cls][feature]:
                        prior_probs[cls] *= self.likelihoods[cls][feature][feature_val]
                    else:
                        # If a feature value wasn't observed, multiply by a small value to avoid zero probability
                        prior_probs[cls] *= 1e-6

            # Choose class with highest probability
            predicted_class = max(prior_probs, key=prior_probs.get)
            predictions.append(predicted_class)

        return np.array(predictions)

class NaiveBayesClassifierWithLaplace:
    def __init__(self, alpha=1):
        """
        Naive Bayes Classifier with Laplace Smoothing.

        Parameters:
        smoothing (float): The Laplace smoothing parameter 'a'.
        """
        self.alpha = alpha
        self.prior_probs = {}  # Prior probabilities P(C)
        self.likelihoods = {}  # Likelihood probabilities P(x|C)

    def fit(self, X, y):
        """
        Fit the Naive Bayes model on the training data.

        Parameters:
        X (pd.DataFrame): Training features.
        y (pd.Series): Training labels.
        """
        self.classes = np.unique(y)
        total_samples = len(y)
        self.class_counts = {}

        # Calculate prior probabilities P(C)
        print(f"N: number of observations in the train dataset = {total_samples}")
        print(f"n_i: count of values of each class type")
        print(f"a: smoothing parameter = {self.alpha}")
        print(f"v: count of possible unique values in classes = {len(self.classes)}")
        print("\n")
        for cls in self.classes:
            class_count = sum(y == cls)
            self.class_counts[cls] = class_count  # Store class count
            self.prior_probs[cls] = (class_count + self.alpha) / (total_samples + len(self.classes) * self.alpha)
            print(f"Prior probability for class {self.classes[cls]} = n_{cls} + a / N + a x v: {(class_count + self.alpha) / (total_samples + len(self.classes) * self.alpha)}")
        print("\n")
        
        # Calculate likelihood probabilities P(x|C) with Laplace smoothing
        print(f"N: number of feature values given a specific class")
        print(f"n_i: count of values of each feature type given a class")
        print(f"a: smoothing parameter = {self.alpha}")
        print(f"v: count of possible unique values in each feature")
        print("\n")
        for cls in self.classes:
            self.likelihoods[cls] = {}
            class_data = X[y == cls]  # Subset of data where y == cls
            print(f"Let's calculate likelihoods for all features given class {self.classes[cls]}\n")
            
            for feature in X.columns:
                self.likelihoods[cls][feature] = {}
                print(f"feature {feature}")
                feature_vals = X[feature].unique()  # Unique values for the feature
                v = len(feature_vals)
                class_feature_values = class_data[feature]
                N = len(class_feature_values)
                print(f"for this feature, we have {len(feature_vals)} types.")
                
                for val in feature_vals:
                    # P(feature | class) = (count of feature value in class) / (total count of class)
                    n_i = np.sum(class_data[feature] == val)
                    self.likelihoods[cls][feature][val] = (n_i + self.alpha) / (N + v * self.alpha)
                    print(f"P(feature {val} | class {cls} ) = count of a feature ({n_i}) x a ({self.alpha})/ N ({N}) = {(n_i + self.alpha) / (N + v * self.alpha)}")
                print(f"")
    
    def predict(self, X):
        """
        Predict the class labels for the provided data.

        Parameters:
        X (pd.DataFrame): Data to predict.

        Returns:
        np.array: Predicted class labels.
        """
        predictions = []

        for _, row in X.iterrows():
            class_scores = {}

            for cls in self.classes:
                # Start with the log of the prior probability
                class_scores[cls] = np.log(self.prior_probs[cls])

                # Add the log likelihoods P(x|C) for each feature
                for feature in X.columns:
                    feature_value = row[feature]

                    if feature_value in self.likelihoods[cls][feature]:  # Corrected access
                        class_scores[cls] += np.log(self.likelihoods[cls][feature][feature_value])
                    else:
                        # If the value hasn't been observed, apply Laplace smoothing
                        unique_values = len(self.likelihoods[cls][feature])
                        class_scores[cls] += np.log(self.alpha / (self.class_counts[cls]) + unique_values * self.alpha)

            # Predict the class with the highest score
            predictions.append(max(class_scores, key=class_scores.get))

        return np.array(predictions)


# Task 1
df = pd.read_csv('data.csv', delim_whitespace=True)
# Convert 'yes'/'no' in 'Play' column to 1/0
df['Play'] = df['Play'].replace({'yes': 1, 'no': 0})
df['Windy'] = df['Windy'].replace({True: 1, False: 0})

encoded_df = label_encode_columns(df, ['Outlook', 'Temperature', 'Humidity'])
# Convert True/False in 'Windy' and other encoded columns to 1/0
df_encoded = encoded_df.astype(int)

print(encoded_df)
print('\n')

# Split the data into training and testing sets (80% train, 20% test)
train_df, test_df = train_test_split(df_encoded, test_size=0.25, random_state=25)

print('train df')
print(train_df)
print('\n')
print('test df')
print(test_df)
print('\n')

# Features (X) and Target (y)
X_train = train_df.drop('Play', axis=1)
y_train = train_df['Play']

X_test = test_df.drop('Play', axis=1)
y_test = test_df['Play']

# Task 2

# Check if the dimension of test_df is equal to or less than train_df
if test_df[0:1].shape[1] == train_df[0:1].shape[1]:
    print(f"Test set dimension {test_df[0:1].shape[1]} is equal to the training set dimension {train_df[0:1].shape[1]}.")
elif test_df[0:1].shape[1] < train_df[0:1].shape[1]:
    print(f"Test set dimension {test_df[0:1].shape[1]} is less than the training set dimension {train_df[0:1].shape[1]}.")

print('\n')

# Train Naive Bayes Classifier
nb_classifier = NaiveBayesClassifier()
nb_classifier.fit(X_train, y_train)

# Predict on test set
y_pred = nb_classifier.predict(X_test)

# Display predictions
print("Predicted labels:", y_pred)
print("Actual labels:", y_test.values)

print('\n')

# Task 3

# Instantiate and fit the classifier
clf = NaiveBayesClassifierWithLaplace(alpha=1)  # Laplace smoothing with a=1
clf.fit(X_train, y_train)

# Predict
predictions = clf.predict(X_test)
print("Predictions:", predictions)
print("True Values:", y_test)