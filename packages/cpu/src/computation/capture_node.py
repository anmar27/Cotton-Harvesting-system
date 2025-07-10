#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image as ROSImage
from message_filters import Subscriber, ApproximateTimeSynchronizer
from computation.srv import retrieveSensorInfo, getDetectedInfo, getDetectedInfoResponse
from utils.vision import Detection
from computation.msg import BoundingBoxes, BoundingBox

class capturign_Node:
    def __init__(self):
        rospy.init_node('capture_node')

        #Based on camera used -> Select different subscriber
        if rospy.get_param('/detection_parameters/camera_used') == 'kinova':
            # Subscribers for PointCloud2 and Image (30hz -> 33.33ms) (Kinova)
            self.pointcloud_sub = Subscriber('/camera/depth_registered/points', PointCloud2)
            self.image_sub = Subscriber('/camera/color/image_rect_color', ROSImage)
        else:
            self.image_sub = Subscriber('rgb/image_raw',ROSImage)
            self.pointcloud_sub = Subscriber('points2', PointCloud2)

        #Initialize BBoxes variable
        self.BBoxes = None

        #Object instance for detection model (YOLO)
        self.detection_model = Detection()
        self.bb_array = []

        #Create publisher for yolo detected info
        self.detected_pub = rospy.Publisher('/yolo/bounding_boxes', BoundingBoxes, queue_size=10)
        
        # Synchronizer to force having both messages same timestamp
        self.sync = ApproximateTimeSynchronizer([self.pointcloud_sub, self.image_sub], queue_size=10, slop=0.1) #Max difference 0.1 s
        self.sync.registerCallback(self.inferance_callback)

        self.last_pointcloud = None
        self.last_image = None

        #Services to retrieve info
        #self.plainInfo_service = rospy.Service('/capture_node/get_latest_messages', retrieveSensorInfo, self.handle_get_latest_messages)
        self.bb_service = rospy.Service('/capture_node/get_info', getDetectedInfo, self.handle_get_last_bb)

    def inferance_callback(self, pointcloud_msg, image_msg):
        # Store the last received messages
        self.last_pointcloud = pointcloud_msg
        self.last_image = image_msg
        #rospy.loginfo("Received new synchronized messages.")

        succesful_detection = self.detection_model.get_predictions(self.last_image)
        if succesful_detection:
            self.bb_array = []  #Initialize only once
            boxes, classes, confidences = self.detection_model.getDetectedInfo()

            for i in range(len(classes)):
                bb = BoundingBox()
                bb.probability = confidences[i]
                bb.xmin = boxes[i][0]
                bb.ymin = boxes[i][1]
                bb.xmax = boxes[i][2]
                bb.ymax = boxes[i][3]
                bb.id = i
                bb.Class = classes[i]

                self.bb_array.append(bb)
            
            self.BBoxes = BoundingBoxes()
            self.BBoxes.header.stamp = rospy.Time.now()
            self.BBoxes.bounding_boxes = self.bb_array
            self.detected_pub.publish(self.BBoxes)

    def handle_get_latest_messages(self):
        # Return the last messages as a service response
        return retrieveSensorInfoResponse(self.last_pointcloud, self.last_image)

    def handle_get_last_bb(self,req):
        if self.BBoxes != None: 
            #Make use of the stamp of the last detected boxes to discard past detections
            last_bb_stamp = self.BBoxes.header.stamp.secs + self.BBoxes.header.stamp.nsecs * 1e-9
            requested_time = req.request_timestamp.data.secs + req.request_timestamp.data.nsecs * 1e-9
            if (requested_time - last_bb_stamp) <= 0.1: #Only accept recent request
                print(self.BBoxes)
                return getDetectedInfoResponse(self.BBoxes,self.last_pointcloud,self.last_image)
            else:
                rospy.logwarn("Last Bounding Box detected was detected to much in the past")
                foo_BBoxes = BoundingBoxes()
                foo_BBoxes.header.stamp = rospy.Time.now()
                foo_BBoxes.bounding_boxes = []
                return getDetectedInfoResponse(foo_BBoxes, self.last_pointcloud, self.last_image)
        else:
            return getDetectedInfoResponse(self.BBoxes,self.last_pointcloud,self.last_image)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        capture_node = capturign_Node()
        capture_node.run()
    except rospy.ROSInterruptException:
        pass
