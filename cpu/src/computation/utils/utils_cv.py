from cv_bridge import CvBridge

from sensor_msgs.msg import (
    Image as ROSImage,
)

#Constants
validFormats = [
        "rgb8", "rgba8", "rgb16", "rgba16", 
        "bgr8", "bgra8", "bgr16", "bgra16", 
        "mono8", "mono16","passthrough"]


def rosImage2cv2image(rosImage: ROSImage , imageFormat: str):
    """
    Given an image in ROS message format, return it in OpenCV format

    In the repo, it is said that it's in format BGRA
    TODO This will return BGRA as the original image is BGRA, but we can change it http://docs.ros.org/en/jade/api/sensor_msgs/html/image__encodings_8h_source.html
    """
    
    if imageFormat not in validFormats:
        raise ValueError(f"Invalid image format: {imageFormat}. Must be one of {validFormats}")

    cv_image = CvBridge().imgmsg_to_cv2(rosImage, desired_encoding=imageFormat)   # "passthrough" como default
    return cv_image

def cv2image2rosImage(cvImageMask, imageFormat:str):
    """
    Given an image in OpenCV format return it into ROSImage format.
    """
    if imageFormat not in validFormats:
        raise ValueError(f"Invalid image format: {imageFormat}. Must be one of {validFormats}")

    rosImage = CvBridge().cv2_to_imgmsg(cvImageMask, encoding=imageFormat)
    return rosImage

def rosDepth2cv2(ros_depth: ROSImage):
    """
    Given a depth map in ROS message format, return it in OpenCV format as Matlike
    """
    cv_image = CvBridge().imgmsg_to_cv2(ros_depth, desired_encoding="passthrough")   # "passthrough" como default
    return cv_image

