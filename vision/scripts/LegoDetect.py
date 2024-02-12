"""!
@file LegoDetect.py
@author Federico Buzzini (federico.buzzini@studenti.unitn.it)
@brief Defines the class Lego and LegoDetect.
@date 2024-01-28
"""
# ------------------------ IMPORT ------------------------

import os
from pathlib import Path
import sys

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from IPython.display import display
from PIL import Image
import ultralytics
from ultralytics import YOLO


ultralytics.checks()

# ------------------------ GLOBAL CONSTANTS ------------------------
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # add relative PATH
VISION_PATH = os.path.abspath(os.path.join(ROOT, ".."))
WEIGHTS_PATH = os.path.join(VISION_PATH, "weights/best.pt")
IMG_ROI = os.path.abspath(os.path.join(ROOT, "log/img_ROI.png"))
MODEL = YOLO('../weights/best.pt')
CONFIDENCE = 0.4


LEGO_CLASSES = [  'X1-Y1-Z2',
                'X1-Y2-Z2',
                'X1-Y3-Z2',
                'X1-Y4-Z2'  ]

# ------------------------ CLASS -------------------------

class LegoDetect:

    #This class detects Lego using Yolov8


    def __init__(self, img_path):

        #class constructor


        MODEL.conf = CONFIDENCE
        MODEL.multi_label = False

        self.block_list = []
        self.detect(img_path)

        # User choose to stop and read, and go or to detect again
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

            # Detect again with same image
            if choice == '1':
                print('Detecting again...')
                self.detect(img_path)
                choice = '0'



    def detect(self, img_path):

        #This function detects Lego and store it to the variable

        self.block_list.clear()

        # Detection model
        self.results = MODEL(img_path)


        img = Image.open(img_path)
        print(img_path)
        print('img size:', img.width, 'x', img.height)

        # Bounding boxes

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

        for index, lego in enumerate(self.block_list, start=1):
            print(index)
            lego.show()

# ---------------------- CLASS ----------------------

class Lego:


    def __init__(self, name, conf, x1, y1, x2, y2, img_source_path):



        self.name = LEGO_CLASSES[name]
        self.class_id = name
        self.confidence = conf
        self.xmin = x1
        self.ymin = y1
        self.xmax = x2
        self.ymax = y2
        self.img_source_path = img_source_path
        self.img_source = Image.open(self.img_source_path)
        self.center_point = (int((x1+x2)/2), int((y1+y2)/2))
        self.point_cloud = ()
        self.point_world = ()



    def show(self):

        # Crop the image to the bounding box of the detected object
        self.img = self.img_source.crop((self.xmin, self.ymin, self.xmax, self.ymax))

        # Resize detected object
        # This step is not necessary. It is useful when the object is too small to see clearly.
        aspect_ratio = self.img.size[1] / self.img.size[0]
        new_width = 70
        new_size = (new_width, int(new_width * aspect_ratio))

        # Print Lego class
        display(self.img)
        print('class =', self.name)
        print('id =', self.class_id)
        print('confidence =', '%.2f' %self.confidence)
        print('center_point =', self.center_point)
        print('--> point cloud =', self.point_cloud)
        print('--> point world =', self.point_world)
        print()



# ------------------------ MAIN -----------------------
# To use this file in command line type:
# python3 LegoDetect.py /path/to/imageName


if __name__ == '__main__':
    legoDetect = LegoDetect(img_origin_path=sys.argv[1])

