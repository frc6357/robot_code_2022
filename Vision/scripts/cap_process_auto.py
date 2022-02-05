'''
Usage:
    cap_process.py (--camera-windows=<int> | --camera-linux=<string>)
    cap_process.py [-h | -help]
Options:
    -h -help                Shows this screen
    --camera-windows=<int>  Video Capture Device number given to OpenCV, used to represent the camera [default: 1]
    --camera-linux=<string> Video Capture Device path in /dev/ directory, the script will parse this to give the Video capture Device number to OpenCV [default: "v4l2-ctl --list-devices | tail -n +`v4l2-ctl --list-devices | egrep \"Video Capture\" -n -m1 | cut -f1 -d:` | head -2 | tail -1 | cut -f2"]
    
    
'''
import cv2
import itertools
import numpy as np
from create_bound import *
import hough_transform
import visionFunctions
import math
import multiprocessing
import time
import imutils
from datetime import datetime
from docopt import docopt
import os
from udp_send import sendPacket
import curses

def sendFloat(input):
    string = str(input)
    decIndex = string.index('.')
    digPreDecimal = len(string[:decIndex])
    adjVal = 4-digPreDecimal
    val = input * 10**adjVal
    return {"int":int("{:.0f}".format(val)), "adjVal":adjVal}
    
def ResizeWithAspectRatio(image, width=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]
    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h*r))
    return cv2.resize(image, dim, interpolation=inter)

def process_frame(frame):
    #frame = ResizeWithAspectRatio(frame, width=1280)
    start_time = time.time()
    h, w, d = frame.shape
    #print(frame.shape)
    #print("height: {h}, width: {w}".format(h=h, w=w))

    #cv2.circle(frame, (320, 240), 5, (248, 26, 225))

    #cv2.imshow("frame", frame)

    im = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    """
    cv2.cvtColor() takes in a variable storing a read image
    and then another method in cv2 that will tell cv2.cvtColor()
    which color space it is in and which color space it wants to convert to
    Ex: 
    
    cv2.cvtColor(im, cv2.BGR2HLS)
    this will read the image stored in im that is currently in the BGR color space
    and convert all pixels to HLS
    """
    #im3 = im
    # test 1 HSV color value: (173 deg, 48%, 76%)
    # upper bound +5%:
    # lower bound -5%:

    """
    bound_percent_cv2 method takes Hue(h), Saturation(s), Brightness(v), Threshold Percent(in decimal form so 100% = 1.0
    Ex: input HSV = [120, 100, 97] and threshold image by +5%
    image_upper_bound = bound_percent_cv2(120, 100, 97, 1.05)
    """

    image_lower_bound = bound_percent_cv2(165, 86, 60, 0.9)
    image_upper_bound = bound_percent_cv2(165, 86, 60, 1.2)

    """
    cv2.inRange() takes in a variable storing a read image, lower bound, and upper bound
    It then checks each each pixel to see if it lies within the lower and upper bound
    It will return a true(1) or false(0) and if the pixel returns 0 then this method
    turns that pixel black, if the pixel returns 1 then this method does nothing to it
    """
    imageFilter = cv2.inRange(im, image_lower_bound, image_upper_bound)
    imageFiltered = cv2.bitwise_and(im, im, mask=imageFilter)
    cv2.imshow("Filtered Image", imageFiltered)

    """
    OpenCV displays all images in the BGR color space in order to look properly
    so after filtering the image in HSV color space the script needs to convert
    back to BGR in order to display interpretable images
    """
    imageFiltered = cv2.cvtColor(imageFiltered, cv2.COLOR_HSV2BGR)
    # cv2.imshow("Filtered Image", imageFiltered)

    # Converts BGR to Grayscale image in preparation for thresholding by making a high contrast image
    grayscale_im = cv2.cvtColor(imageFiltered, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("Grayscale Image", grayscale_im)

    # uses OTSU and Binary Thresholding methods to maximize contrast in filtered image
    ret2, th2 = cv2.threshold(grayscale_im, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    #cv2.imshow("Grayscale Otsu Thresholded Image", th2)

    # uses canny edge detection to find the edges of segmented image
    #edges = cv2.Canny(th2, 50, 200)
    # cv2.imshow("Canny Edge Detection", edges)

    # finds each individual "contour" or OTSU thresholded rectangluar region
    contours = cv2.findContours(th2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    contourAreas = []

    # iterates through list
    for contour in contours:
        # finds pixel count for each target
        A = cv2.contourArea(contour)
        contourAreas.append(A)
            #cv2.circle(frame, (cX,cY), 3, (0,0,255))
            #centroids.append((Cx, Cy))
    distance = 0
    vertAngle = 0
    horiAngle = 0
    adjVal = 255
    valid = False
    for contour in contours:
        M = cv2.moments(contour)
        if int(M["m00"]) != 0:
            # chooses the target with the largest pixel count
            if (cv2.contourArea(contour) == max(contourAreas)):
                roiBox = cv2.minAreaRect(contour)
                box = cv2.boxPoints(roiBox)
                box = np.int0(box)
                
                points = []
                for point in box:
                    points.append(point[0])
                    cv2.circle(frame, (point[0], point[1]), 1, (0,0,255))
                pixelWidth = max(points)-min(points) 
                stdscr.addstr(3, 0, "ROI width: {}".format(pixelWidth))
                #print("ROI width: {}".format(pixelWidth))
                #cv2.imshow("roi points", frame)
                #cv2.waitKey(0)
                Cx = int(M["m10"]/M["m00"])
                Cy = int(M["m01"]/M["m00"])
                stdscr.addstr(4, 0, "Cy: {}".format(Cy))
                #print("Center Value x: {}".format(Cx))
                #print("Center Value y: {}".format(Cy))
                #print("Y distance from center: {}".format(Cy-))
                cv2.circle(frame, (Cx, Cy), 3, (255,255,255))
                #cv2.putText(frame, "center: ({x_val}, {y_val})".format(x_val=Cx, y_val=Cy), (20, 20),
                            #cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 1)
                # mounting height
                vertOffset = 45

                # degrees per pixel, constant measure before hand
                degPixHFOV = 70.42/w

                horiAngle = (degPixHFOV * ((w/2)-Cx))
                stdscr.addstr(5, 0, "Hori Angle: {}".format(horiAngle))
                #print(horiAngle)
                # if vertical angle is zero then this breaks, practically in a match we will never be level with the
                # target
                #print(math.atan(horiAngle))
                distanceRaw = (0.5*5)/math.atan(horiAngle)
                distanceCalib = (1.0526*distanceRaw)-27.711
                stdscr.addstr(0,0, "Calib: " + str(distanceCalib))
                #print("Raw: " + str(distanceRaw))
                #print("Calib: " + str(distanceCalib), end = "\r")
                #print("\n")
                #print((h/2) - Cy)
                #print((43.3/h) * ((h/2)-Cy) + 40)
                degPixVFOV = 43.3/h
                vertAngle  = (degPixVFOV * ((h/2)-Cy)) + 40
                #print("vertical angle: {}".format(vertAngle))
                
                distance2 = (102-46.25) / math.tan(vertAngle * (math.pi/180))
                distance2Calib = 1.0862*distance2-1.09967
                stdscr.addstr(1,0,"distance2 calib: {}".format(distance2Calib))
                #print("\n")
                #print(distanceRaw)
                #print("Avg Distance: " + str((distance2+distanceCalib)/2), end = "\r")
                stdscr.addstr(2,0,"distance2: " + str(distance2))
                floatVals = sendFloat(distance2)
                distance = floatVals["int"]
                adjVal = floatVals["adjVal"]               
                valid = False
                #print("horizontal angle: " + str(horiAngle))
                #print("vertical angle: " + str(vertAngle))
                #print("distance: " + str(distance/(10**adjVal)))
                #print("\n")
                
                #cv2.putText(frame, "horizontal angle: {theta}".format(theta=horiAngle), (20, 80),
                            #cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 1)
                #cv2.putText(frame, "vertical angle: {theta}".format(theta=vertAngle), (20, 200),
                             #cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 1)
                #cv2.putText(frame, "distance: {dist}".format(dist=distance), (20, 140),
                            #cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 1)
    #sendUDP(distance, horiAngle, vertAngle, frameID)
    end_time = time.time()
    stdscr.refresh()
    #print("Processing Time for 1 Frame: " + str(end_time-start_time))
    cv2.imshow("frame", frame)
    if cv2.waitKey(0) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        #curses.endwin()
    #elif cv2.waitKey(0) & 0xFF == ord('s'):
                #cv2.imwrite("/home/odroid/Desktop/frame.png", frame)
                #cv2.destroyAllWindows()
    return {"frame": frame, "dist": distance, "adjVal":adjVal, "hAngle": float(horiAngle), "vAngle": float(vertAngle), "valid":valid}
    
        




def cap_frame(args):
    frameID = 0
    running = True
    """if (bool(args['--camera-windows'])):
        cap = cv2.VideoCapture(int(args['--camera-windows']))
        cap.set(cv2.CAP_PROP_EXPOSURE, -11)
    if (bool(args['--camera-linux'])):
        device_path = os.popen(str(args['--camera-linux'])).read()
        cap = cv2.VideoCapture(device_path[len(device_path)-1])
        cap.set(cv2.CAP_PROP_EXPOSURE, -11)"""
        
    # for linux specify video capture driver (V4L2)
    cap = cv2.VideoCapture(8, cv2.CAP_V4L2)
    # For V4L2 and Logitech c920 1 is manual exposure, 3 is auto exposure
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    # cap_prop_exposure is in terms of milliseconds
    cap.set(cv2.CAP_PROP_EXPOSURE, 10)

    while running:
        ret, frame = cap.read()
        if ret == False:
            running == False
            print("Camera ret val false, qutting")
            exit()
        frameOutputs = process_frame(frame)
        if (frameOutputs["valid"]):
            distance = frameOutputs["dist"]
            adjVal = frameOutputs["adjVal"]
            vertAngle = int(float("{:.2f}".format(frameOutputs["vAngle"]))* 100)
            horiAngle = int(float("{:.2f}".format(frameOutputs["hAngle"]))* 100)
            frameID +=1
            sendPacket(distance, adjVal, horiAngle, vertAngle, frameID)
        #else:
            #print("Target Not Found")
            

if __name__ == "__main__":
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    frameID = 0
    args = docopt(__doc__)
    cap_frame(args)
