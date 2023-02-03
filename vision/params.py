sono prova

uiu

dd

import os
import subprocess
 
# Set up find command
findCMD = 'find / -name "yolov5"'
out = subprocess.Popen(findCMD,shell=True,stdin=subprocess.PIPE, 
                        stdout=subprocess.PIPE,stderr=subprocess.PIPE)
# Get standard out and error
(stdout, stderr) = out.communicate()
 
# commento di prova finale

# Save found files to list
filelist = stdout.decode().split()

if(len(filelist)>0):
    YOLO_PATH =  filelist[0]
else:
    print("ERROR DON'T FIND YOLOV5 PATH")
    YOLO_PATH = ""
VISION_PATH = os.path.dirname(os.path.abspath(__file__))
LAST_PHOTO_PATH = VISION_PATH + "/last_photo.png"
BEST_PATH = VISION_PATH +"/best_v2.40.pt"

