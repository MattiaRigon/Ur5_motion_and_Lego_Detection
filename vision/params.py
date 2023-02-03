

import os

YOLO_PATH = os.system("find / -name 'yolov5'")
VISION_PATH = os.path.dirname(os.path.abspath(__file__))
LAST_PHOTO_PATH = VISION_PATH + "/last_photo.png"
BEST_PATH = VISION_PATH +"/best_v2.40.pt"

