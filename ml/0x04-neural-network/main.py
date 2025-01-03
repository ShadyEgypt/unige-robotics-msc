from tensorflow.keras.datasets import mnist
from sklearn.model_selection import train_test_split
from autoencoder import Autoencoder
import numpy as np
import matplotlib.pyplot as plt

# Load MNIST dataset
(XTrain, YTrain), (XTest, YTest) = mnist.load_data()

# Filter the dataset for classes 1 and 8
train_filter = np.isin(YTrain, [1, 8])
test_filter = np.isin(YTest, [1, 8])

XTrain_subset = XTrain[train_filter]
YTrain_subset = YTrain[train_filter]

# We'll use this for validation
XVal_subset = XTest[test_filter]
YVal_subset = YTest[test_filter]

# Train-test split
XTrain_subset, XTest_subset, YTrain_subset, YTest_subset = train_test_split(
    XTrain_subset, YTrain_subset, test_size=0.2, random_state=42
)

# Verify the shapes of the subsets
print('XTrain_subset shape = ', XTrain_subset.shape)
print('YTrain_subset shape = ', YTrain_subset.shape)
print('XTest_subset shape = ', XTest_subset.shape)
print('YTest_subset shape = ', YTest_subset.shape)

# Check unique classes in the subset
print('Unique classes in YTrain_subset = ', np.unique(YTrain_subset))
print('Unique classes in YTest_subset = ', np.unique(YTest_subset))

# Normalize data
XTrain_subset = XTrain_subset.astype('float32') / 255.0
XTest_subset = XTest_subset.astype('float32') / 255.0
XVal_subset = XVal_subset.astype('float32') / 255.0

# Reshape data
XTrain_subset = XTrain_subset.reshape((len(XTrain_subset), -1))  
XTest_subset = XTest_subset.reshape((len(XTest_subset), -1))
XVal_subset = XVal_subset.reshape((len(XVal_subset), -1)) 

print("XTrain_subset shape:", XTrain_subset.shape)
print("XTest_subset shape:", XTest_subset.shape)
print("XVal_subset shape:", XVal_subset.shape)

autoencoder = Autoencoder(input_dim=(28, 28), encoding_dim=32, activation='relu', output_activation='sigmoid')

autoencoder.fit(XTrain_subset, epochs=70, batch_size=256, validation_data=(XVal_subset, XVal_subset))

decoded_imgs = autoencoder.autoencoder.predict(XTest_subset)

# Plot latent space clusters
autoencoder.plot_clusters(XTest_subset, labels=YTest_subset, apply_pca=True)

# Plot the training history
autoencoder.plot_model_history()

# Compute and plot VAF by class
autoencoder.plot_vaf_by_class(XTest_subset, YTest_subset, decoded_imgs)