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



class VisionManager:
    def __init__(self, length, breadth):
        self.table_length = length
        self.table_breadth = breadth
        self.bridge = CvBridge()

        # self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.imageCb)
        self.image1_pub = rospy.Publisher('/table_detect', Image, queue_size=1)
        self.image2_pub = rospy.Publisher('/object_detect', Image, queue_size=1)
        self.position_pub = rospy.Publisher('/detected_object_position', Point, queue_size=1)
        self.workspace_mask = None
        self.mask_initialized = False

        self.pixels_permm_x = None
        self.pixels_permm_y = None
        self.failure_count = 0

    def data_process(self):    
        # ROS 桥接器，用于将 ROS 图像消息转换为 OpenCV 格式
        bridge = CvBridge()
        # rospy.init_node('moveit_control_server', anonymous=False)
        # 订阅 RealSense 相机的相关话题depth_camera_info_topic
        color_topic = "/camera/color/image_raw"  # 彩色图像话题

        color_camera_info_topic = "/camera/color/camera_info"  # 彩色相机信息话题
        print("等待相机数据...")
        # 将 ROS 消息转换为 OpenCV 格式
        color_msg = rospy.wait_for_message(color_topic, Image)
        print("获取到相机数据。")

        rgb = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")  # 转换彩色图像

        print("rgb.shape:", rgb.shape)

        color = rgb.copy()  
        # Use the function to get clicked points
        clicked_points = show_image_and_get_pixel(color.copy(), "Select workspace corners")
        
        # If points were selected, use them for workspace mask
        if len(clicked_points) >= 4:
            # Use only the first 4 points
            points = clicked_points[:4]
            cropped_img = select_workspace(color, points)
        else:
            # Use default points
            cropped_img = select_workspace(color)


        color = cropped_img #[workspace_mask == 0] = 0
        cv2.imwrite("cropped_img.png", color)


        x, y = self.get2DLocation(color)
        if x is None or y is None:
            return

        position_msg = Point()
        position_msg.x = x
        position_msg.y = y
        position_msg.z = 0.4287306796454262  # Set from calibration
        self.position_pub.publish(position_msg)



    def get2DLocation(self, current_image): # Accepts cv2 image directly
        bbox = self.detectTable(current_image) # Pass cv2 image
        if bbox is None:
            return self._handle_failure("Table not detected")

        pixel_x, pixel_y = self.detect2DObject(current_image, bbox) # Pass cv2 image
        if pixel_x == 0 and pixel_y == 0: # Assuming 0,0 is an error code
            return self._handle_failure("Object not detected")

        self.failure_count = 0 # Reset on success
        img_h, img_w = current_image.shape[:2]
        return self.convertToMM(pixel_x, pixel_y, img_w, img_h)
    

    def detectTable(self, image):
        if image is None:
            return None

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)

        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # cv2.imshow("Binary Image", binary)
        # cv2.waitKey(1)

        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            rospy.logwarn("No contours found for table.")
            return None
        """
        x: The x-coordinate of the top-left corner of the bounding rectangle.
        y: The y-coordinate of the top-left corner of the bounding rectangle.
        w: The width of the bounding rectangle.
        h: The height of the bounding rectangle.
        """
        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        center = (x + w // 2, y + h // 2)
        
        # Draw overlay and compute scale
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(image, center, 4, (0, 0, 255), -1)
        cv2.imwrite("detected_table.png", image)
        self.image1_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

        self.pixels_permm_x = w / self.table_breadth
        self.pixels_permm_y = h / self.table_length

        return (x, y, w, h)

    def detect2DObject(self, image, table_bbox):
        # image = self._convert_to_cv(msg)
        if image is None:
            return 0, 0

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_min = np.array([35, 50, 50])
        hsv_max = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, hsv_min, hsv_max)

        x, y, w, h = table_bbox
        region_mask = np.zeros_like(mask)
        region_mask[y+3:y+h-3, x+3:x+w-3] = 255
        mask = cv2.bitwise_and(mask, region_mask)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(mask, kernel)
        mask = cv2.erode(mask, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) > 10]

        if not valid:
            rospy.logwarn("No valid contours for object.")
            return 0, 0

        x, y, w, h = cv2.boundingRect(valid[0])
        cx, cy = x + w // 2, y + h // 2
        cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
        cv2.drawContours(image, valid, -1, (255, 0, 0), 2)
        cv2.imwrite("detected_object.png", image)
        self.image2_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

        return cx, cy

    def convertToMM(self, x, y, width, height):
        if self.pixels_permm_x is None or self.pixels_permm_y is None:
            rospy.logerr("Conversion scale not available.")
            return 0, 0

        center_x = width // 2
        center_y = height // 2
        return (x - center_x) / self.pixels_permm_x, (y - center_y) / self.pixels_permm_y

    def _convert_to_cv(self, msg):
        try:
            return self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", str(e))
            return None

    def _handle_failure(self, reason):
        self.failure_count += 1
        if self.failure_count >= 5:
            rospy.logwarn(f"{reason} for 5 consecutive frames!")
        return None, None


if __name__ == '__main__':
    try:
        rospy.init_node('vision_manager_node')
        rospy.loginfo("Starting Vision Manager...")

        length = rospy.get_param("~table_length", 0.281)
        breadth = rospy.get_param("~table_breadth", 0.190)

        vm = VisionManager(length, breadth)
        vm.data_process()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
