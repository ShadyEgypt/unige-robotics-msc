from tensorflow.keras.layers import Input, Dense, BatchNormalization, Dropout
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau
from sklearn.decomposition import PCA
import numpy as np
import matplotlib.pyplot as plt

class Autoencoder:
    def __init__(self, input_dim=(28, 28), encoding_dim=32, activation='relu', output_activation='sigmoid'):
        self.input_dim = input_dim  # Shape of input images
        self.encoding_dim = encoding_dim

        # Encoder
        self.input_layer = Input(shape=(np.prod(input_dim),))  # Flattened input
        self.encoded = Dense(encoding_dim, activation=activation)(self.input_layer)
        self.encoded = BatchNormalization()(self.encoded)
        self.encoded = Dropout(0.2)(self.encoded)

        # Decoder
        self.decoded = Dense(np.prod(input_dim), activation=output_activation)(self.encoded)

        # Autoencoder Model
        self.autoencoder = Model(self.input_layer, self.decoded)
        self.encoder = Model(self.input_layer, self.encoded)

        # Compile Autoencoder
        self.autoencoder.compile(optimizer=Adam(learning_rate=0.001), loss='binary_crossentropy')

    def fit(self, x_train, epochs=10, batch_size=32, validation_data=None):
        """Fit the autoencoder."""
        early_stop = EarlyStopping(monitor='val_loss', patience=5, restore_best_weights=True)
        reduce_lr = ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=3, verbose=1)

        self.history = self.autoencoder.fit(
            x_train, x_train,
            epochs=epochs,
            batch_size=batch_size,
            shuffle=True,
            validation_data=validation_data,
            callbacks=[early_stop, reduce_lr]
        )
        
    def save_model(self, filepath):
        self.autoencoder.save(filepath)
    
    @staticmethod
    def load_model(filepath):
        from tensorflow.keras.models import load_model
        return load_model(filepath)
    
    def visualize_reconstructions(self, x_test, n=10):
        decoded_imgs = self.autoencoder.predict(x_test)

        plt.figure(figsize=(20, 6))
        for i in range(n):
            # Original images
            ax = plt.subplot(2, n, i + 1)
            plt.imshow(x_test[i].reshape(self.input_dim), cmap='gray')
            plt.title("Original")
            plt.axis('off')

            # Reconstructed images
            ax = plt.subplot(2, n, i + 1 + n)
            plt.imshow(decoded_imgs[i].reshape(self.input_dim), cmap='gray')
            plt.title("Reconstructed")
            plt.axis('off')

        plt.show()
        
    def compute_vaf(self, true_data, predicted_data):
        """Compute the Variance Accounted For (VAF)."""
        vaf = 1 - np.var(true_data - predicted_data) / np.var(true_data)
        return vaf * 100

    def plot_model_history(self):
        """Plot training and validation loss history."""
        plt.plot(self.history.history['loss'], label='Train Loss')
        plt.plot(self.history.history['val_loss'], label='Validation Loss')
        plt.ylabel('Loss')
        plt.xlabel('Epoch')
        plt.legend(loc='upper left')
        plt.title('Model Training History')
        plt.show()

    def plot_clusters(self, x_test, labels, coord=slice(0, 2), psize=2, apply_pca=True):
        """
        Plot clusters in 1D, 2D, or 3D latent space.

        Parameters:
        - x_test: Input test data.
        - labels: Class labels for test data.
        - coord: Slice of dimensions to plot.
        - psize: Point size.
        - apply_pca: Whether to apply PCA for dimensionality reduction.
        """
        # Encode the data
        encoded_imgs = self.encoder.predict(x_test)

        # Apply PCA if dimensions > 3
        if apply_pca and encoded_imgs.shape[1] > 3:
            pca = PCA(n_components=3)
            encoded_imgs = pca.fit_transform(encoded_imgs)
            print("Applied PCA to reduce latent space to 3 dimensions.")

        x = encoded_imgs[:, coord]

        unique_labels = np.unique(labels)
        colors = 'rbgcmyk'
        pointstyles = 'o+*xsd^v'

        plt.figure(figsize=(10, 8))

        for i, label in enumerate(unique_labels):
            indices = np.where(labels == label)
            plt.scatter(x[indices, 0], x[indices, 1], c=colors[i % len(colors)], marker=pointstyles[i % len(pointstyles)],
                        label=f"Label {label}", s=psize)

        plt.xlabel("Latent Dimension 1")
        plt.ylabel("Latent Dimension 2")
        plt.title("Latent Space Clusters")
        plt.legend()
        plt.grid()
        plt.show()

    def plot_vaf_by_class(self, x_test, y_test, decoded_imgs):
        """
        Compute and plot VAF (Variance Accounted For) for each class as a bar chart.

        Parameters:
        - x_test: True test data.
        - y_test: Class labels for test data.
        - decoded_imgs: Reconstructed images.
        """
        # Compute VAF for each class
        x = np.unique(y_test)
        vaf_values = [self.compute_vaf(x_test[y_test == digit].flatten(), 
                                      decoded_imgs[y_test == digit].flatten()) for digit in x]

        plt.figure(figsize=(8, 6))
        plt.bar(x, vaf_values, color='skyblue')
        plt.xlabel('Digits')
        plt.ylabel('VAF (%)')
        plt.title('VAF by Digit')
        plt.grid(axis='y', linestyle='--', linewidth=0.5, color='gray', alpha=0.5)

        for i, vaf in enumerate(vaf_values):
            plt.text(x[i], vaf, f'{vaf:.2f}%', ha='center', va='bottom')

        plt.xticks(x)
        plt.ylim(0, 100)
        plt.show()
