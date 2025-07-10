import cv2

# Initialize variables
drawing = False  # True if mouse is pressed
ix, iy = -1, -1  # Initial position

# List to store coordinates
box_coordinates = []

# Mouse callback function
def draw_rectangle(event, x, y, flags, param):
    global ix, iy, drawing, box_coordinates

    if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse button pressed
        drawing = True
        ix, iy = x, y
        box_coordinates = [(ix, iy)]  # Start point

    elif event == cv2.EVENT_MOUSEMOVE:  # Mouse movement
        if drawing:
            temp_img = img.copy()
            cv2.rectangle(temp_img, (ix, iy), (x, y), (0, 255, 0), 2)
            cv2.imshow('Image', temp_img)

    elif event == cv2.EVENT_LBUTTONUP:  # Left mouse button released
        drawing = False
        cv2.rectangle(img, (ix, iy), (x, y), (0, 255, 0), 2)
        box_coordinates.append((x, y))  # End point
        cv2.imshow('Image', img)

        # Display box coordinates
        xmin, ymin = box_coordinates[0]
        xmax, ymax = box_coordinates[1]
        print(f"xmin: {xmin}, ymin: {ymin}, xmax: {xmax}, ymax: {ymax}")


# Load the image
img = cv2.imread('first.png')  # Replace with your image file path
cv2.imshow('Image', img)

# Set mouse callback
cv2.setMouseCallback('Image', draw_rectangle)

# Wait until a key is pressed
cv2.waitKey(0)
cv2.destroyAllWindows()
