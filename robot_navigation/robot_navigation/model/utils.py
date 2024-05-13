import math
import numpy as np
import cv2
from robot_navigation.model.network import AlexNetNavigationModel 
import os

model = None

def lidar_data_to_image(lidar_data, img_size=(227, 227), laser_range=2.5):
    """Converts LiDAR data to a grayscale image representation.

    Args:
        lidar_data (np.array): Array of LiDAR ranges.
        img_size (tuple, optional): Size of the output image. Defaults to (227, 227).
        laser_range (float, optional): Maximum range of the LiDAR sensor. Defaults to 2.5.

    Returns:
        np.array: Grayscale image representation of the LiDAR data.
    """

    # Extract angles and distances, filtering out infinite distances
    angles = np.linspace(-math.pi, math.pi, len(lidar_data))  # Create array of angles for each LiDAR reading
    distances = np.array(lidar_data)
    valid_indices = np.where(np.isfinite(distances))[0]  # Find indices of valid (finite) distances
    angles = angles[valid_indices]
    distances = distances[valid_indices]

    # Create an empty image
    image = np.zeros(img_size, dtype=np.uint8)
    image[:] = 255  # Set background to white

    # Convert angles and distances to x, y coordinates and scale to image size
    x = distances * np.cos(angles)  # Calculate x coordinates based on angles and distances
    y = distances * np.sin(angles)  # Calculate y coordinates based on angles and distances
    x = ((x - (-laser_range)) / (2 * laser_range)) * img_size[1] - 1  # Scale x coordinates to image width
    y = ((y - (-laser_range)) / (2 * laser_range)) * img_size[0] - 1  # Scale y coordinates to image height
    x = x.astype(int)  # Convert to integers for indexing
    y = y.astype(int)

    # Set pixel values to black for valid LiDAR points
    image[y, img_size[1] - 1 - x] = 0  # Set pixels to black based on calculated coordinates

    return image

def display_lidar_image(image):
    """Displays a LiDAR image using OpenCV.

    Args:
        image (np.array): LiDAR image to display.
    """
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)  # Convert to 3-channel for display
    cv2.imshow("LiDAR", image)
    cv2.waitKey(1)

def clean_file(data_path = 'src/robot_navigation/robot_navigation/model/data/data.csv'):
    """ Clean file before first usage"""
   
    with open(data_path, "w") as f:  # Open file in write mode 
        pass

def save_data(lidar_data, label, data_path = 'src/robot_navigation/robot_navigation/model/data/data.csv'):
    """Saves LiDAR data and corresponding movement label to a CSV file.

    Args:
        lidar_data (np.array): Array of LiDAR ranges.
        label (str): Movement command label (e.g., "forward", "left").
        data_path (str): Path to the CSV file.
    """

    with open(data_path, "a") as f:  # Open file in append mode 
        data_str = ",".join([str(x) for x in lidar_data])  # Convert data to comma-separated string 
        f.write(f"{data_str},{label}\n")  # Write data and label to file

def initiate_model():
  """
  Initializes the AlexNet navigation model.
  """
  
  global model
  model = AlexNetNavigationModel()

def predict(lidar_data):
  """
  Predicts the robot's next move based on lidar data.

  Processes lidar data, generates a prediction using the AlexNet model, 
  and returns the corresponding movement command.

  Args:
      lidar_data: Lidar scan data.

  Returns:
      str: The predicted movement command (e.g., 'forward', 'stop', 'left', 'right').
  """
  global model
  image = lidar_data_to_image(lidar_data)
  display_lidar_image(image)  
  label_mapping = {0:'forward', 1:'stop', 2:'left', 3:'right'}
  out = model.predict(image)
  predicted_index = np.argmax(out)
  move = label_mapping.get(predicted_index, 'stop')
  return move