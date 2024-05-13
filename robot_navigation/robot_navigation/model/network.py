import tensorflow as tf
from tensorflow.keras import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, BatchNormalization, Flatten, Dense, Dropout
from tensorflow.keras.models import load_model
from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping
from tensorflow import keras
from tensorflow.keras.optimizers import RMSprop

from sklearn.model_selection import train_test_split
import os
import numpy as np

class AlexNetNavigationModel:
    """AlexNet convolutional neural network for robot navigation."""

    def __init__(self, path = "src/robot_navigation/robot_navigation/model/saved_models/model.keras", num_classes=4):
        self.num_classes = num_classes        
        self.model_path = path
        self.model = self._build_model()

    def _build_model(self):
        """Builds and compiles the AlexNet model."""
        if os.path.exists(self.model_path):
            model = load_model(self.model_path)
        else:
            model = Sequential([
                Conv2D(96, (11, 11), strides=(4,4), activation='relu', input_shape=(227, 227, 1)),
                MaxPooling2D((3, 3), strides=(2,2)),
                BatchNormalization(),  
                Conv2D(256, (5, 5), activation='relu', padding='same'),
                MaxPooling2D((3, 3), strides=(2, 2)),
                BatchNormalization(),
                Conv2D(384, (3, 3), activation='relu', padding='same'),
                BatchNormalization(),
                Conv2D(384, (3, 3), activation='relu', padding='same'),
                BatchNormalization(),
                Conv2D(256, (3, 3), activation='relu', padding='same'),
                MaxPooling2D((3, 3), strides=(2, 2)),
                BatchNormalization(),
                Flatten(),
                Dense(4096, activation='relu'),
                Dropout(0.5),
                Dense(4096, activation='relu'),
                Dropout(0.5),
                Dense(self.num_classes, activation='softmax')
            ])

        model.compile(optimizer=RMSprop(learning_rate=0.0001),
                      loss='categorical_crossentropy',
                      metrics=['accuracy'])
        return model

    def prepare_dataset(self, train_images, train_labels):
        """Prepares the dataset for training and validation."""
        train_labels = keras.utils.to_categorical(train_labels, num_classes=self.num_classes)
        self.X_train, X_val, self.y_train, y_val = train_test_split(train_images, train_labels, test_size=0.3, random_state=42)
        X_val, self.X_test, y_val, self.y_test = train_test_split(X_val, y_val, test_size=0.5, random_state=42)
        self.valid_dataset = tf.data.Dataset.from_tensor_slices((X_val, y_val))

    def train(self):
        """Trains the model with early stopping and checkpointing."""
        checkpoint_callback = ModelCheckpoint(filepath=self.model_path, monitor='val_loss', save_best_only=True, mode='min')
        early_stopping_callback = EarlyStopping(monitor='val_loss', mode='min', patience=10)

        self.model.fit(
            x=self.X_train,
            y=self.y_train,
            batch_size=128,
            epochs=100,
            verbose=1,
            callbacks=[checkpoint_callback, early_stopping_callback],
            validation_data=self.valid_dataset.batch(64))
        
    def evaluate(self):
        """Evaluates the model on the test set."""
        loss, acc = self.model.evaluate(self.X_test, self.y_test, verbose=2)

    def predict(self, image):
        """Predicts the movement command based on the input image."""
        out = self.model.predict(np.array([image]))
        return out