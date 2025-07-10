"""
Util in charge of inferancing YOLO model over Image stream and obtain state
of cotton bolls and the         
"""

import rospy
import numpy as np
from ultralytics import YOLO
import time
import cv2
import torch

class utilPerceptionModel():
    def __init__(self,arch):
        print("UtilPerceptionModel CONSTRUCTOR Called")
        #self.model = self.__initalization_model() #TODO used to be the func in charge of initialize model
        #self.predictor = SamPredictor(sam_model_registry["vit_h"](checkpoint=SAM_CHECKPOINT).to(device))
        
        #Initialize class variables
        self.boxesInfo_ = []
        self.classesInfo = []
        self.confidencesInfo_ = []

        #TODO change configuration of model to configuration/set up state
        #TODO remove assigment of model in initalization (after testing it takes around 50 ms)
        if arch == "yolo":
            self.arch_yolo = True
            self.yolo_model = YOLO("/iri_lab/iri_ws/src/cpu/Models/YOLO_Model/best_0.7.7m_e500.pt")
        else: 
            self.arch_yolo = False

    def __del__(self):
        print("UtilPerceptionModel DESTRUCTOR called")

    def __get_approximate_centroids(self,boxes: torch.Tensor):
        centroids = []
        #Box -> [center_x,center_y,w,h]
        for box in boxes:
            x_centerPoint = box[0]
            y_centerPoint = box[1]
            center = [x_centerPoint.item(),y_centerPoint.item()]
            centroids.append(center)
        return centroids

    def __setInfoDetections(self,boxes,classes,confidences):
        self.boxesInfo_ = boxes
        self.classesInfo_ = classes
        self.confidencesInfo_ = confidences

    def obtainStatesAndApproxCentroids(self,cv_image):

        start_time = time.time()
        if self.arch_yolo == True:
            
            #Inferance of Yolov8 model
            print("Entering YOLO Modified Image step")
            results = self.yolo_model(cv_image)
            boxes = results[0].boxes.xywh
            boxes_xyxy = results[0].boxes.xyxy.tolist()
            classes = results[0].boxes.cls.tolist()
            confidences = results[0].boxes.conf.tolist()
            names = results[0].names
            if boxes == []:
                reception_flag = 0
                YOLO_computation_time = time.time() - start_time
                return boxes, YOLO_computation_time,cv_image,reception_flag
            else:
                reception_flag = 1
                print("Boxes obtained" + str(boxes))
                aprox_centroids = self.__get_approximate_centroids(boxes)
                for box,cls,conf in zip(boxes_xyxy, classes,confidences):

                        #TODO add index number of each localization in mod_image
                        x_min, y_min, x_max, y_max = box
                        cls = cls
                        conf = conf
                        cv2.rectangle(cv_image, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
                        # Create text with class and confidence
                        text = f"{names[int(cls)]}: {conf:.2f}" 
                        text_width, text_height = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 1)[0]
                        text_x = x_min + 5 
                        text_y = y_min - text_height  
                        # Draw the text with a black background for better visibility
                        cv2.rectangle(cv_image, (int(text_x), int(text_y - 5)), (int(text_x + text_width + 10), int(text_y + 5)), (0, 255, 0), cv2.FILLED) 
                        cv2.putText(cv_image, text, (int(text_x), int(text_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)  
    
                YOLO_computation_time = time.time() - start_time
                aprox_centroids = [[int(elem) for elem in sublist] for sublist in aprox_centroids]
                self.__setInfoDetections(boxes_xyxy,classes,confidences)
                return aprox_centroids, YOLO_computation_time,cv_image,reception_flag
            
        else:
            rospy.loginfo("Model selected different from YOLO.")

    def getInfoDetections(self):            
        return self.boxesInfo_,self.classesInfo_,self.confidencesInfo_