"""
Testing project to obtain a centroid of an object, given an image and a mask over it,
obtain the centroid of the object detected.

The part of z dimension and Azure Kinect DK will come after

-TODO make a package system for all files in this repository?
"""

import os
import cv2
from argparse import ArgumentParser
import torch
import sys
import time
import matplotlib.pyplot as plt
import numpy as np
from groundingdino.util.inference import annotate, predict
from ultralytics import YOLO
#from kornia.morphology import erosion #Temporary commented for testign purposes

IS_A_DOCKER_IMAGE = 1

if IS_A_DOCKER_IMAGE == 0:
    sys.path.append(f"../Models/Grounded-Segment-Anything")
    sys.path.append(os.path.join(f"../Models/Grounded-Segment-Anything", "GroundingDINO"))
    sys.path.append(os.path.join(f"../Models/Grounded-Segment-Anything", "segment_anything"))
else:
    sys.path.append(f"/iri_lab/iri_ws/src/cpu/Models/Grounded-Segment-Anything")
    sys.path.append(os.path.join(f"/iri_lab/iri_ws/src/cpu/Models/Grounded-Segment-Anything", "GroundingDINO"))
    sys.path.append(os.path.join(f"/iri_lab/iri_ws/src/cpu/Models/Grounded-Segment-Anything", "segment_anything"))

# segment anything
from segment_anything import (
    sam_model_registry,
    SamPredictor
)

# Testing utils
"""
from testingUtils import ( 
    load_image,
    load_model,
    get_grounding_output, 
    show_mask, 
    show_box, 
    save_mask_data
)
"""

BASE_PATH = os.getcwd() + "/.."
BOX_TRESHOLD = 0.35
TEXT_TRESHOLD = 0.25
#device = torch.device('cuda:0')
device = torch.device('cpu')


#TEMPORARY global variables for ROS testin -> Later on put them in constructor object
CONFIG_FILE = '/iri_lab/iri_ws/src/cpu/Models/Grounded-Segment-Anything/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py'
GROUNDED_CHECKPOINT = '/iri_lab/iri_ws/src/cpu/Models/Grounded-Segment-Anything/groundingdino_swint_ogc.pth'
SAM_CHECKPOINT = '/iri_lab/iri_ws/src/cpu/Models/Grounded-Segment-Anything/sam_vit_h_4b8939.pth'
#Relative paths may give problems when being called from ROS node (cpu)
TEXT_PROMPT = 'cotton'



## Utils

def settings() -> ArgumentParser:
    """
    General setting before script execution.
    """
    parser = ArgumentParser("Centroid computing program", add_help=True)
    parser.add_argument("--image_path", "-i", type=str, required=True, help="path to image file")
    parser.add_argument("--output_path", "-o", type=str, required=True, help="path to output directory")
    parser.add_argument("--config_file", "-c", type=str, required=True, help="path to SAM config file")
    parser.add_argument("--grounded_checkpoint", "-g", type=str, required=True, help="path to grounded SAM checkpoint")
    parser.add_argument("--device", "-d", type=str, default="cuda:0", help="Device to use when performing tensorial operations")
    parser.add_argument("--text_prompt", "-t", type=str, required=True, help="Text prompt to compute prediction")
    parser.add_argument("--sam_checkpoint", "-s", type=str, required=True, help="Path to SAM checkpoint")

    return parser.parse_args()

def print_image_with_centroids(image, centroids: list, output_dir: str, image_file: str):
    """
    Saves the given image with centroids printed on top, so they can be visualized.
    """
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    for (x,y) in centroids:
        image = cv2.circle(image, (x,y), radius=10, color=(0, 0, 255), thickness=-1)
    
    os.makedirs(output_dir, exist_ok=True)
    os.chdir(output_dir)
    image_filename = image_file.split(".")[0]
    cv2.imwrite(image_filename+"_centroids.jpg", image)

    return

def erode(mask: torch.Tensor) -> torch.Tensor:
    """
    Apply erosion to a boolean matrix, just like MatLab does. It reduces the boundaries, eroding away part of these boundaries
    """

    copy_mask = mask.detach().clone().to(device)
    copy_mask = copy_mask.int()
    copy_mask = copy_mask[None,:,:,:]
    kernel_mask = torch.ones((5,5),device=device)

    eroded_mask = erosion(copy_mask,kernel_mask)

    return eroded_mask

## Functionalities

def compute_centroid(pixels: list):
    """
    Compute the centroid of the given pixels in image

    pixels = [(pixel_w, pixel_h)]
    """
    n_pixels = len(pixels)
    
    # Median:
    x_pixels = [pixel[0] for pixel in pixels]
    y_pixels = [pixel[1] for pixel in pixels]
    x_pixels.sort()
    y_pixels.sort()

    return (x_pixels[n_pixels // 2], y_pixels[n_pixels // 2])

def get_centroids(masks: torch.Tensor):
    """
    Returns the centroids (x,y) as positions in image.

    image.(w,h) = mask.(w,h)
    centroid: list[tuple[int,int]]
    """
    # Algorithm
    # IDEA: To compute the centroid is the same as getting the median of the sorted pixels located
    # inside de area highlighted by the mask

    centroids = []
    for mask in masks:
        crit = (mask[0] == True)
        idx = crit.nonzero()
        pixels = []
        for (x,y) in idx:
            pixels.append((x.item(),y.item()))
        centroids.append(compute_centroid(pixels))
    return centroids

def get_approximate_centroids(boxes: torch.Tensor):
    centroids = []
    #Box -> [center_x,center_y,w,h]
    for box in boxes:
        x_centerPoint = box[0]
        y_centerPoint = box[1]
        center = [x_centerPoint.item(),y_centerPoint.item()]
        centroids.append(center)
    return centroids

def process_all_images():
    """
    Compute all centroids in images, given a file in directory

    -TODO Maybe we can classify in types of image (raw, SAM, centroids, masks) 
    -TODO this funciton will be updated to get_xz_coords() in eye space, and will have to be restructured 
    """
    # Settings
    args = settings()

    image_path: str = args.image_path
    output_dir: str = args.output_path
    config_file: str = args.config_file
    grounded_checkpoint: str = args.grounded_checkpoint
    device: str = args.device
    text_prompt: str = args.text_prompt
    sam_checkpoint: str = args.sam_checkpoint

    # Absolute paths
    if not os.path.isabs(image_path):
        image_path = os.path.abspath(image_path)
    if not os.path.isabs(output_dir):
        output_dir = os.path.abspath(output_dir)

    # make dir
    os.makedirs(output_dir, exist_ok=True)

    # Model loading
    model = load_model(config_file, grounded_checkpoint, device=device)
    # initialize SAM
    # predictor = SamPredictor(sam_hq_model_registry["vit_h"](checkpoint=sam_hq_checkpoint).to(device))
    predictor = SamPredictor(sam_model_registry["vit_h"](checkpoint=sam_checkpoint).to(device))

    images = []
    if os.path.isfile(image_path):
        images = [image_path.split("/")[-1]]
        image_path = image_path.split("/")[:-1]
        image_path = "/".join(image_path)
    elif os.path.isdir(image_path):
        images = os.listdir(image_path)

    for image_filename in images:
        # load image
        extension = image_filename.split(".")[1]
        image_name = image_filename.split(".")[0]

        image_pil, image = load_image(f"{image_path}/{image_filename}")  # Fails
        
        # visualize raw image
        image_pil.save(os.path.join(output_dir, image_name+"_raw_image.jpg"))
        start_time = time.time()

        # run grounding dino model
        boxes_filt, pred_phrases = get_grounding_output(
            model, image, text_prompt, BOX_TRESHOLD, TEXT_TRESHOLD, device=device
        )
        
        print('Boxes:'+ str(boxes_filt))
        image = cv2.imread(f"{image_path}/{image_name}.{extension}")
        print(f"{image_path}/{image_name}.{extension}")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        predictor.set_image(image)

        size = image_pil.size
        H, W = size[1], size[0]
        for i in range(boxes_filt.size(0)):
            boxes_filt[i] = boxes_filt[i] * torch.Tensor([W, H, W, H])
            boxes_filt[i][:2] -= boxes_filt[i][2:] / 2
            boxes_filt[i][2:] += boxes_filt[i][:2]

        boxes_filt = boxes_filt.cpu()
        transformed_boxes = predictor.transform.apply_boxes_torch(boxes_filt, image.shape[:2]).to(device)

        # Given (image, mask) return the centroid of image
        H, W, _ = image.shape

        masks = torch.zeros(0, 1, H, W).to(device)
        if len(pred_phrases) != 0:
            masks, _, _ = predictor.predict_torch(
                point_coords = None,
                point_labels = None,
                boxes = transformed_boxes.to(device),
                multimask_output = False,
            )

        # draw output image
        plt.figure(figsize=(10, 10))
        plt.imshow(image)
        for mask in masks:
            show_mask(mask.cpu().numpy(), plt.gca(), random_color=True)
        for box, label in zip(boxes_filt, pred_phrases):
            show_box(box.numpy(), plt.gca(), label)

        plt.axis('off')
        plt.savefig(
            os.path.join(output_dir, image_name+"_gsam.jpg"),
            bbox_inches="tight", dpi=300, pad_inches=0.0
        )

        print("Image size: ", size)
        print("Mask size: ", masks.size())

        # Erosion treatment to masks
        '''
        for i in range(masks.shape[0]):
            masks[i] = erode(masks[i])
            pass
        '''

        # Centroid obtaining
        centroids = []
        centroids: list = get_centroids(masks=masks)    # TODO we have to relate the centroids to every prediction (cotton, centroid)
        # Change the mask dimensions from HxW to WxH
        centroids = [(centroid[1],centroid[0]) for centroid in centroids]
        print("Centroid for", f"{image_name}.{extension}", "are pixels", centroids)

        # Centroid printing in original image
        print_image_with_centroids(image, centroids, output_dir, f"{image_name}.{extension}")

        # Local save
        text_prompt = text_prompt.replace(" ","_")
        save_mask_data(img_file_name_prompt=f"mask_{image_name}_{text_prompt}",output_dir=output_dir, 
                           mask_list=masks, box_list=boxes_filt, label_list=pred_phrases)
        print(time.time() - start_time, "seconds")

        plt.close("all")
    pass

#Creation of class for better code encapsulating (also not initialize model whole time)
#,tensor_image,cv_image,windows_or_centroids
class Centroid:
    def __init__(self,arch) -> None :
        print("Inside Initalization Centroid")
        self.model = self.initalization_model()
        self.predictor = SamPredictor(sam_model_registry["vit_h"](checkpoint=SAM_CHECKPOINT).to(device))
        if arch == "yolo":
            self.arch_yolo = True
            self.yolo_model = YOLO("/iri_lab/iri_ws/src/cpu/Models/YOLO_Model/best.pt")
        else: 
            self.arch_yolo = False

        
        
        #self.centroids_process_single_imageROS(tensor_image,cv_image,windows_or_centroids)
    
    def initalization_model(self):
        model = load_model(CONFIG_FILE, GROUNDED_CHECKPOINT, device=device)
        return model
    
    def centroids_process_single_imageROS(self,tensor_image,cv_image,windows_or_centroids,segmentation):

        # Model loading  MOVE LATER ON TO INITALIZATION PROCESS ¿Create __init__ class?
        """
        model = load_model(CONFIG_FILE, GROUNDED_CHECKPOINT, device=device)
        """
        print("IN CENTROIDS PROCESS SINGLE IMAGE")
        start_time = time.time()
        if self.arch_yolo == True:
            
            #Inferance of Yolov8 model
            print("Entering yolo model conditional")
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
                print("Boxes obtained"+ str(boxes))
                aprox_centroids = get_approximate_centroids(boxes)

                for box,cls,conf in zip(boxes_xyxy, classes,confidences):
                    
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

                return aprox_centroids, YOLO_computation_time,cv_image,reception_flag
            

        else:

            if windows_or_centroids:  #If TRUE windows / if FALSE only centroids
                #For testing reasons to print actual boxes

                boxes_filt, logits, pred_phrases = predict(
                model=self.model, 
                image=tensor_image, 
                caption=TEXT_PROMPT, 
                box_threshold=BOX_TRESHOLD, 
                text_threshold=TEXT_TRESHOLD
                )

                cv_image = annotate(image_source=cv_image,boxes=boxes_filt,logits=logits,phrases=pred_phrases)
                print("Score: " + str(logits))
            else: 
            
                #Run grounding dino model
                boxes_filt, pred_phrases = get_grounding_output(
                    self.model, tensor_image, TEXT_PROMPT, BOX_TRESHOLD, TEXT_TRESHOLD, device=device
                )

            print("Finding: " + str(pred_phrases))
            aprox_centroids = get_approximate_centroids(boxes_filt) #List of centers
            print(aprox_centroids)

            if aprox_centroids == []:
                reception_flag = 0
                DINO_computation_time = time.time() - start_time
                return aprox_centroids, DINO_computation_time,cv_image,reception_flag
            
            elif segmentation == False: #Obtain only centroids 
                aprox_centroids = np.array(aprox_centroids) * np.array(cv_image.shape[:2][::-1]) # scale by image dimensions
                aprox_centroids = aprox_centroids.astype(np.int32)
                    # Draw circles for each point on the image
                for point in aprox_centroids:
                    print("Our aprox point is " + str(point))
                    cv2.circle(cv_image, tuple(point), radius=5, color=(0, 0, 255), thickness=-1)

                reception_flag = 1
                DINO_computation_time = time.time() - start_time
                #print("Inputs: + str(aprox_centroids) +  " + str(DINO_computation_time) + " " +str(cv_image) + " " + str(reception_flag))
                return aprox_centroids,DINO_computation_time,cv_image,reception_flag

            else: #Obtain SAM

                self.predictor.set_image(cv_image)

                size = cv_image.size
                #H, W = size[1], size[0]
                H, W, _ = cv_image.shape
                for i in range(boxes_filt.size(0)):
                    boxes_filt[i] = boxes_filt[i] * torch.Tensor([W, H, W, H])
                    boxes_filt[i][:2] -= boxes_filt[i][2:] / 2
                    boxes_filt[i][2:] += boxes_filt[i][:2]

                boxes_filt = boxes_filt.cpu()

                transformed_boxes = self.predictor.transform.apply_boxes_torch(boxes_filt, cv_image.shape[:2]).to(device)
                # Given (image, mask) return the centroid of image
                
                #

                masks = torch.zeros(0, 1, H, W).to(device)
                if len(pred_phrases) != 0:
                    masks, _, _ = self.predictor.predict_torch(
                        point_coords = None,
                        point_labels = None,
                        boxes = transformed_boxes.to(device),
                        multimask_output = False,)
                """    
                for mask in masks:
                    show_mask(mask.cpu().numpy(), plt.gca(), random_color=True)
                for box, label in zip(boxes_filt, pred_phrases):
                    show_box(box.numpy(), plt.gca(), label)
                """ 

                reception_flag = 1
                SAM_computation_time = time.time() - start_time
                return masks,SAM_computation_time,cv_image,reception_flag
    
