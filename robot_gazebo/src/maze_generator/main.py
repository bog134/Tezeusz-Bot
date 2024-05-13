import cv2
import numpy as np
import argparse


def load_image(path):
    """Loads an image from the specified path."""
    global image
    image = cv2.imread(f'src/robot_gazebo/src/maze_generator/{path}.png')


def display_image(img):
    """Displays an image using OpenCV and waits for a key press."""
    cv2.imshow("Image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def extract_line_segments(threshold):
    """Extracts line segments from a grayscale image using Hough transform."""
    grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, binary_image = cv2.threshold(grayscale_image, 127, 255, cv2.THRESH_BINARY_INV)

    lines = cv2.HoughLinesP(
        binary_image, rho=1, theta=np.pi / 180, threshold=threshold, minLineLength=50, maxLineGap=10
    )
    line_segments = [line[0] for line in lines]
    return line_segments


def draw_line_segments(line_segments, color=(0, 0, 255), thickness=2):
    """Draws line segments on an image."""
    for x1, y1, x2, y2 in line_segments:
        cv2.line(image, (x1, y1), (x2, y2), color, thickness)


def draw_line_on_click(event, x, y, flags, param):
    """Mouse callback function to draw lines interactively."""
    global line_points, drawing, line_segments  # Access global variables

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        line_points = [[x, y]]
    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        image_copy = image.copy()  # Create a copy for temporary drawing
        cv2.line(image_copy, line_points[0], (x, y), (0, 255, 0), 2)  # Draw temporary line in green
        display_image(image_copy)  # Display the copy with the temporary line
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        line_points.append([x, y])
        # Ensure horizontal or vertical line
        if abs(line_points[1][0] - line_points[0][0]) < abs(line_points[1][1] - line_points[0][1]):
            line_points[1][0] = line_points[0][0]
        else:
            line_points[1][1] = line_points[0][1]
        line_segments.append([*line_points[0], *line_points[1]])
        cv2.line(image, line_points[0], line_points[1], (0, 0, 255), 2)  # Draw final line in red 


def get_line_parameters(x1, y1, x2, y2):
    """Calculates parameters (position, size, orientation) of a 3D line segment from 2D endpoints."""

    x1 /= 100
    y1 /= 100
    x2 /= 100
    y2 /= 100

    if x1 > x2 or y1 > y2:
        x2, x1 = x1, x2
        y2, y1 = y1, y2

    delta_x, delta_y = x2 - x1, y2 - y1

    start_point = (x1 + delta_x / 2 - 0.1, y1 + delta_y / 2 - 0.1, 0)  # Starting point in 3D

    # Adjust dimensions to avoid overlapping lines 
    if delta_x == 0:
        delta_x = 0.1
        delta_y += 0.08
    if delta_y == 0:
        delta_y = 0.1
        delta_x += 0.08
    dimensions = (delta_x, delta_y, 1)  # Dimensions (dx, dy, dz)    

    return start_point, dimensions


def process(path, threshold):
    """Processes an image and generates an SDF file for the maze."""

    load_image(path)

    global line_segments, drawing
    drawing = False

    # Extract line segments using Hough transform
    line_segments = extract_line_segments(threshold)
    draw_line_segments(line_segments)

    # Set up mouse callback for interactive drawing
    cv2.namedWindow("Image")
    cv2.setMouseCallback("Image", draw_line_on_click)

    while True:
        cv2.imshow("Image", image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):  # Press 'q' to quit
            break

    # Load SDF script template
    with open("src/robot_gazebo/src/maze_generator/script.sdf", "r") as f:
        script_template = f.read()

    # Split the script into parts for easier modification
    script_part1_template = script_template[:script_template.find("<link")]
    script_part2_template = script_template[script_template.find("<link"):script_template.rfind("<static")]
    script_part3 = script_template[script_template.rfind("<static"):]

    new_script_part1 = script_part1_template.replace("<model name='maze'>", f"<model name='{path}'>")

    # Process each line segment and generate corresponding SDF code
    script_part2_modified = ""
    for i, line_segment in enumerate(line_segments):
        start_point, dimensions = get_line_parameters(*line_segment)
        if np.all(np.array(dimensions) >= 0.1): 
            new_script_part2 = (
                script_part2_template.replace("Wall_0", f"Wall_{i}")
                .replace("<pose>0 0 0 0 0 0", f"<pose>{' '.join(map(str, start_point))} 0 0 0")
                .replace("<size>0 0 0", f"<size>{' '.join(map(str, dimensions))}")
            )
            script_part2_modified += new_script_part2

    # Combine modified script parts and save to a new SDF file 
    with open(f'src/robot_gazebo/models/{path}/model.sdf', "w") as f:
        f.write(new_script_part1 + script_part2_modified + script_part3)


def main():
    parser = argparse.ArgumentParser(description='Maze Generator')
    parser.add_argument('path', help='Path to the image file')
    parser.add_argument('-t', '--threshold', type=int, default=80, help='Hough transform threshold')
    args = parser.parse_args()

    process(args.path, args.threshold)


if __name__ == '__main__':
    main()