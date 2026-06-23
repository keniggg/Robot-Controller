import cv2

image = './masked_image.png'
def detectTable(image):
        # image = self._convert_to_cv(msg)
        if image is None:
            return None

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)

        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        cv2.imshow("Binary Image", binary)
        cv2.waitKey(1)

        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print("No contours found for table.")
            return None

        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        center = (x + w // 2, y + h // 2)

        # Draw overlay and compute scale
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(image, center, 4, (0, 0, 255), -1)
        # save the ploted image
        cv2.imwrite('output.png', image)
        # self.pixels_permm_x = w / table_breadth
        # self.pixels_permm_y = h / self.table_length

image = cv2.imread(image)
detectTable(image)