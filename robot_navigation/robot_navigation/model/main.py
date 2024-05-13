from robot_navigation.model.network import AlexNetNavigationModel  
from robot_navigation.model.utils import lidar_data_to_image

import numpy as np
import re

DATA_PATH = 'src/robot_navigation/robot_navigation/model/data/'

def prepare_training_data():
    """Loads, preprocesses, and saves training data for the navigation model."""

    def process_label(text):
        """Converts textual movement commands into numerical labels."""
        text = re.sub(r"[^\w]", "", text)
        label_mapping = {'forward': 0, 'stop': 1, 'left': 2, 'right': 3}
        return label_mapping.get(text, -1)

    # Load data from CSV file
    train_data = np.genfromtxt(DATA_PATH + "data.csv", delimiter=',', dtype=str)

    # Extract LiDAR ranges and convert to images
    lidar_ranges = np.array([np.array(data[:-1], dtype=float) for data in train_data])
    train_images = np.apply_along_axis(lidar_data_to_image, 1, lidar_ranges)

    # Extract and process movement commands (labels)
    train_labels = np.array([process_label(data[-1]) for data in train_data])

    return train_images,train_labels


def train_model(train_images,train_labels):
    """Trains the AlexNet navigation model on the prepared dataset."""

    model = AlexNetNavigationModel()  # Instantiate the model
    model.prepare_dataset(train_images, train_labels)
    model.train()
    model.evaluate()


if __name__ == '__main__':
    train_images,train_labels = prepare_training_data()
    train_model(train_images,train_labels)