"""!
@file LegoDetect.py
@author Federico Buzzini (federico.buzzini@studenti.unitn.it)
@brief Defines the class Lego and LegoDetect.
@date 2024-01-28
"""
# ---------------------- IMPORT ----------------------
from pathlib import Path
import sys
import os
#import torch
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv
from IPython.display import display
from PIL import Image

from ultralytics import YOLO
from PIL import Image
import ultralytics

ultralytics.checks()

# ---------------------- GLOBAL CONSTANTS ----------------------
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
VISION_PATH = os.path.abspath(os.path.join(ROOT, ".."))
WEIGHTS_PATH = os.path.join(VISION_PATH, "weights/best.pt")
IMG_ROI = os.path.abspath(os.path.join(ROOT, "log/img_ROI.png"))
MODEL = YOLO('../weights/best.pt')
CONFIDENCE = 0.7


LEGO_NAMES = [  'X1-Y1-Z2',
                'X1-Y2-Z2',
                'X1-Y3-Z2',
                'X1-Y4-Z2'  ]

# ---------------------- CLASS ----------------------

class LegoDetect:
    """
    @brief This class use custom trained weights and detect lego blocks with YOLOv8
    """

    def __init__(self, img_path):
        """ @brief Class constructor
            @param img_path (String): path of input image
        """

        MODEL.conf = CONFIDENCE
        MODEL.multi_label = False

        # MODEL.iou = 0.5

        self.block_list = []
        self.detect(img_path)

        # Let user choose detect method
        choice = '0'
        while True:
            while (choice != '1' and choice != '2' and choice != ''):
                ask =  ('\nContinue   (ENTER)'+
                        '\nDetect again (1)'+
                        '\nTYPE (ENTER or 1+ENTER):')
                choice = input(ask)

            # Continue
            if choice == '':
                break

            # Detect again using original image
            if choice == '1':
                print('Detecting again...')
                self.detect(img_path)
                choice = '0'



    def detect(self, img_path):
        """ @brief This function pass the image path to the model and calculate bounding boxes for each object
            @param img_path (String): path of input image
        """
        self.block_list.clear()

        # Detection model
        self.results = MODEL(img_path)
        #print(self.results)
        #self.results.show



        img = Image.open(img_path)
        print(img_path)
        print('img size:', img.width, 'x', img.height)

        # Bounding boxes
        #bboxes = self.results[0].boxes #.pandas().xyxy[0].to_dict(orient="records")
        for r in self.results:


            for box in r.boxes:

                for c in box.cls:
                    name = int(c)

                conf = box.conf

                b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format

                x1=int(b[0])
                y1=int(b[1])
                x2=int(b[2])
                y2=int(b[3])


                # Add lego to list
                self.block_list.append(Lego(name, conf, x1, y1, x2, y2, img_path))


        # Info
        print('Detected', len(self.block_list), 'object(s)\n')
        for l in self.block_list:
            l.show()


    def show(self):
        """ @brief This function show infos of detected legos
        """
        for index, lego in enumerate(self.block_list, start=1):
            print(index)
            lego.show()

# ---------------------- CLASS ----------------------

class Lego:
    """
    @brief This class represents info of detected lego
    """

    def __init__(self, name, conf, x1, y1, x2, y2, img_source_path):
        """ @brief Class constructor
            @param name (String): lego name
            @param conf (float): confidence
            @param x1 (float): xmin of bounding box
            @param y1 (float): ymin of bounding box
            @param x2 (float): xmax of bounding box
            @param y2 (float): ymax of bounding box
            @param img_source_path (String): path to image
        """

        self.name = LEGO_NAMES[name]
        self.class_id = name
        self.confidence = conf
        self.xmin = x1
        self.ymin = y1
        self.xmax = x2
        self.ymax = y2
        self.img_source_path = img_source_path
        self.img_source = Image.open(self.img_source_path)
        self.center_point = (int((x1+x2)/2), int((y1+y2)/2))
        self.center_point_uv = (self.img_source.width - self.center_point[0], self.center_point[1])
        self.point_cloud = ()
        self.point_world = ()


    def show(self):
        """ @brief Show lego info
        """

        self.img = self.img_source.crop((self.xmin, self.ymin, self.xmax, self.ymax))

        # Resize detected obj
        # Not neccessary. Useful when the obj is too small to see
        aspect_ratio = self.img.size[1] / self.img.size[0]
        new_width = 70  # resize width (pixel) for detected object to show
        new_size = (new_width, int(new_width * aspect_ratio))
        #elf.img = self.img.resize(new_size, Image.LANCZOS)

        # Lego details
        display(self.img)
        print('class =', self.name)
        print('id =', self.class_id)
        print('confidence =', '%.2f' %self.confidence)
        print('center_point =', self.center_point)
        print('center_point_uv =', self.center_point_uv)
        print('--> point cloud =', self.point_cloud)
        print('--> point world =', self.point_world)
        print()


# ---------------------- MAIN ----------------------
# To use in command:
# python3 LegoDetect.py /path/to/img...

if __name__ == '__main__':
    legoDetect = LegoDetect(img_origin_path=sys.argv[1])

