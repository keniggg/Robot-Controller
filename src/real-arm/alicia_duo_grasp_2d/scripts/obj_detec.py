#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

def select_workspace(image, points=None):
    """
    Crop the workspace based on provided or default points.
    :param image: Input image
    :param points: List of (x,y) points defining workspace corners. If None, use defaults.
    :return: cropped image
    """
    # Use default points if none provided
    if points is None:
        # Default points
        points = [[176, 122], [425, 114], [135, 348], [468, 346]]
    
    ordered_points = np.array(points, dtype=np.float32)

    # Order points clockwise
    def order_points_clockwise(pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]  # Top-left
        rect[2] = pts[np.argmax(s)]  # Bottom-right
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]  # Top-right
        rect[3] = pts[np.argmax(diff)]  # Bottom-left
        return rect

    ordered_points = order_points_clockwise(ordered_points)
    
    (tl, tr, br, bl) = ordered_points

    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    if maxWidth <= 0 or maxHeight <= 0:
        rospy.logwarn("select_workspace: Calculated width or height for perspective transform is zero or negative.")
        return None

    # Define the destination points for the perspective transform
    # This creates a "birds-eye view" of the selected region
    dst_pts = np.array([
        [0, 0],                # Top-left
        [maxWidth - 1, 0],     # Top-right
        [maxWidth - 1, maxHeight - 1], # Bottom-right
        [0, maxHeight - 1]     # Bottom-left
    ], dtype="float32")

    # Compute the perspective transform matrix
    M = cv2.getPerspectiveTransform(ordered_points, dst_pts)

    # Apply the perspective transform to warp the image
    warped_image = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    
    return warped_image

def show_image_and_get_pixel(image_to_show, window_name="Click to get pixel coordinates"):
    """
    Displays an image and prints the coordinates of left mouse clicks.
    Press 'Esc' to close the window.
    Returns a list of clicked points as (x, y) tuples.
    """
    clicked_points = []
    
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # OpenCV coordinates are (x=column, y=row)
            print(f"Pixel coordinates: x={x}, y={y}")
            if y < image_to_show.shape[0] and x < image_to_show.shape[1]:
                # Show the pixel value at the clicked point
                if len(image_to_show.shape) == 3:  # Color image
                    pixel_value = image_to_show[y, x]
                    print(f"RGB value: {pixel_value}")
                else:  # Grayscale/depth image
                    pixel_value = image_to_show[y, x]
                    print(f"Pixel value: {pixel_value}")
                
                clicked_points.append((x, y))  # Store as (x, y)
            else:
                print("Clicked outside valid image area")

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    print(f"Displaying image in window '{window_name}'.")
    print(f"Image dimensions: {image_to_show.shape[1]}x{image_to_show.shape[0]} (width x height)")
    print("Click on points to get coordinates. Press 'Esc' to continue.")
    
    while True:
        # Draw circles on the clicked points
        display_img = image_to_show.copy()
        for i, (px, py) in enumerate(clicked_points):
            cv2.circle(display_img, (px, py), 5, (0, 255, 0), 2)
            cv2.putText(display_img, f"{i+1}: ({px},{py})", (px+10, py), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        cv2.imshow(window_name, display_img)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC key
            break
    
    cv2.destroyWindow(window_name)
    print(f"Selected {len(clicked_points)} points: {clicked_points}")
    return clicked_points  # Return the list of points




class GreenCubeDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/object_detect', Image, queue_size=1)
        self.position_pub = rospy.Publisher('/detected_object_position', Point, queue_size=1)
        
        # Calibration parameters - can be adjusted based on your camera setup
        # These represent the approximate pixels per mm at a typical working distance
        self.pixels_per_mm = 10  # This would need to be calibrated for your specific camera setup
        
    def data_process(self):
        # Get image from camera
        bridge = CvBridge()
        color_topic = "/camera/color/image_raw"  # RGB image topic
        
        print("Waiting for camera data...")
        color_msg = rospy.wait_for_message(color_topic, Image)
        print("Camera data received.")

        # Convert ROS message to OpenCV format
        rgb = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        print("Image shape:", rgb.shape)

        color = rgb.copy()
        
        # Allow user to select workspace region
        clicked_points = show_image_and_get_pixel(color.copy(), "Select workspace corners")
        
        # Crop workspace based on selected corners or use defaults
        if len(clicked_points) >= 4:
            cropped_img = select_workspace(color, clicked_points[:4])
        else:
            cropped_img = select_workspace(color)
        
        cv2.imwrite("cropped_img.png", cropped_img)

        # Detect green cube and get its position
        x_mm, y_mm, detection_img = self.detect_green_cube(cropped_img)
        
        if x_mm is not None and y_mm is not None:
            # Convert to camera coordinates
            camera_position = self.pixel_to_camera_coord(x_mm, y_mm)
            
            # Publish the position
            self.position_pub.publish(camera_position)
            
            # Publish the detection image
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(detection_img, "bgr8"))
            print(f"Green cube detected at position: x={camera_position.x:.3f}, y={camera_position.y:.3f}, z={camera_position.z:.3f} meters")
        else:
            print("Failed to detect green cube")


    def detect_green_cube(self, image):
        if image is None:
            return None, None, None
            
        # Convert to HSV for better color segmentation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define green color range for the cube
        # Adjust these values based on your specific green cube
        hsv_min = np.array([35, 50, 50])   # Lower bound for green
        hsv_max = np.array([85, 255, 255]) # Upper bound for green
        
        # Create mask for green objects
        mask = cv2.inRange(hsv, hsv_min, hsv_max)
        
        # Apply morphological operations to remove noise
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(mask, kernel)
        mask = cv2.erode(mask, kernel)
        
        # Find contours of green objects
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by size and shape - looking for 1.5cm cube
        valid_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 100:  # Filter out very small contours
                continue
                
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(c)
            
            # Check aspect ratio - a cube should be roughly square
            aspect_ratio = float(w) / h
            if 0.7 <= aspect_ratio <= 1.3:  # Allow some deviation from perfect square
                valid_contours.append(c)
        
        if not valid_contours:
            rospy.logwarn("No valid green cube detected")
            return None, None, image
                
        # Get the most square-like contour (closest to 1:1 aspect ratio)
        best_contour = min(valid_contours, 
                        key=lambda c: abs(cv2.boundingRect(c)[2]/cv2.boundingRect(c)[3] - 1))
        
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(best_contour)
        center_x, center_y = x + w // 2, y + h // 2
        
        # Draw detection on image
        detection_img = image.copy()
        cv2.rectangle(detection_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(detection_img, (center_x, center_y), 4, (0, 0, 255), -1)
        cv2.putText(detection_img, f"1.5cm Cube ({w}x{h}px)", (x, y-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Use the known cube size (1.5cm) to calculate pixels per mm
        # This dynamically adjusts based on distance from camera
        pixels_per_mm_x = w / 15.0  # 15mm = 1.5cm
        pixels_per_mm_y = h / 15.0
        
        # Store this for debugging
        cv2.putText(detection_img, f"Scale: {pixels_per_mm_x:.2f}px/mm", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        cv2.imwrite("detected_green_cube.png", detection_img)
        
        # Convert pixel coordinates to mm (relative to image center)
        img_height, img_width = image.shape[:2]
        img_center_x, img_center_y = img_width // 2, img_height // 2
        
        # Convert to mm using the calibration from cube size
        x_mm = (center_x - img_center_x) / pixels_per_mm_x
        y_mm = (center_y - img_center_y) / pixels_per_mm_y
        
        return x_mm, y_mm, detection_img


    def pixel_to_camera_coord(self, x_mm, y_mm, z_height=0.45):
        """
        Convert 2D image coordinates (in mm) to 3D camera coordinates.
        
        Args:
            x_mm: X position in mm relative to image center
            y_mm: Y position in mm relative to image center
            z_height: Z height in meters (default is estimated height of workspace)
        
        Returns:
            Point: 3D position in camera frame (meters)
        """
        if x_mm is None or y_mm is None:
            return None
        
        # Create a ROS point message
        point_msg = Point()
        
        # Convert mm to meters and apply any necessary transforms
        # Note: In camera coordinates, typically:
        # X: right positive
        # Y: down positive
        # Z: forward positive
        point_msg.x = x_mm / 1000.0  # mm to meters
        point_msg.y = y_mm / 1000.0  # mm to meters
        point_msg.z = z_height      # meters
        
        return point_msg


if __name__ == '__main__':
    try:
        rospy.init_node('green_cube_detector')
        rospy.loginfo("Starting Green Cube Detector...")

        detector = GreenCubeDetector()
        detector.data_process()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass