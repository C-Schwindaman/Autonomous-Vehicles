import cv2 as cv
import numpy as np
import sys
import os

def detect_taillights(image_path):
    """
    Detects red taillights in an image using color thresholding and morphological operations.
    
    Args:
        image_path (str): The full path to the input image file.

    Returns:
        None. Displays the mask and the final image with bounding boxes.
    """
    # --- 1. Load the Image ---
    if not os.path.exists(image_path):
        print(f"Error: Image not found at '{image_path}'")
        return

    img = cv.imread(image_path)
    if img is None:
        print(f"Error: Could not read image from '{image_path}'")
        return

    # --- 2. Color Thresholding in HSV Space ---
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    # Define ranges for red in HSV
    lower_red_1 = np.array([0, 100, 100])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([160, 100, 100])
    upper_red_2 = np.array([179, 255, 255])
    
    mask1 = cv.inRange(hsv_img, lower_red_1, upper_red_1)
    mask2 = cv.inRange(hsv_img, lower_red_2, upper_red_2)
    mask = cv.bitwise_or(mask1, mask2)

    # --- 3. (NEW) Clean up the Mask with Morphology ---
    # A "closing" operation (dilation followed by erosion) will connect nearby
    # white pixels and fill small holes, making the detected regions more solid.
    kernel = np.ones((5,5), np.uint8)
    mask_closed = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    # --- 4. Find Bounding Boxes from the Cleaned Mask ---
    num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(mask_closed, connectivity=8)
    
    output_img = img.copy()

    for i in range(1, num_labels):
        x = stats[i, cv.CC_STAT_LEFT]
        y = stats[i, cv.CC_STAT_TOP]
        w = stats[i, cv.CC_STAT_WIDTH]
        h = stats[i, cv.CC_STAT_HEIGHT]
        area = stats[i, cv.CC_STAT_AREA]
        
        # --- (IMPROVED) Filter based on area and aspect ratio ---
        # Lower the minimum area threshold and check that the blob isn't just a long, thin line.
        min_area = 20
        aspect_ratio = w / float(h)
        
        if area > min_area and 0.3 < aspect_ratio < 4.0:
            cv.rectangle(output_img, (x, y), (x + w, y + h), (0, 255, 0), 2) # Changed to green for visibility

    # --- 5. Display Results ---
    cv.imshow('Original Mask', mask) 
    cv.imshow('Cleaned Mask (Morphology)', mask_closed)
    cv.imshow('Detected Lights', output_img)
    
    print("Showing results for:", image_path)
    print("Press any key to close the windows.")
    
    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python light_detect.py <path_to_image>")
        print("Example: python src/light_detect.py data/img1.jpg")
        sys.exit(1)
    
    image_path_arg = sys.argv[1]
    detect_taillights(image_path_arg)

