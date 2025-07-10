Header header
int16 class_id          # ID of the object class (e.g., 1 for "person")
string class_name       # Name of the object class (e.g., "person")
float32 centroid_x      # X-coordinate of the centroid in the image
float32 centroid_y      # Y-coordinate of the centroid in the image
sensor_msgs/Image mask_image  # The segmented mask as a grayscale image