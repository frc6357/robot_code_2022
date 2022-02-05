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

def test_process_frame(frame):
    #print(frame[0])
    h, w, d = frame.shape
    cv2.circle(frame, (int(w/2), int(h/2)), 3, (255, 0, 0))
    cv2.circle(frame, (int(w/4), int(h/4)), 3, (255, 0, 0))
    cv2.circle(frame, (int(w/6), int(h/6)), 3, (255, 0, 0))
    cv2.circle(frame, (int(w/8), int(h/8)), 3, (255, 0, 0))
    cv2.circle(frame, (int(w/10), int(h/10)), 3, (255, 0, 0))
    cv2.circle(frame, (int(w/12), int(h/12)), 3, (255, 0, 0))
    cv2.circle(frame, (int(w/14), int(h/14)), 3, (255, 0, 0))
    cv2.circle(frame, (int(w/16), int(h/16)), 3, (255, 0, 0))

    return frame

def rectCentr(input):
    Xs = [a[0] for a in input]
    Ys = [a[1] for a in input]
    bot_left = [min(Xs), min(Ys)]
    top_right = [max(Xs), max(Ys)]
    mid_point = (int((bot_left[1] + top_right[1])/2), int((bot_left[0] + top_right[0])/2))
    return mid_point

def extremeItems(a, str_input):
    Xs = [i[0] for i in a]
    Ys = [i[1] for i in a]
    if str_input == 'minXmaxY':
        bucket = []
        b = min(Xs)
        for i, j in enumerate(a):
            if b in j:
                bucket.append(a[i])
        return [b, max(Ys)]
    elif str_input == 'maxXmaxY':
        bucket = []
        b = max(Xs)
        for i, j in enumerate(a):
            if b in j:
                bucket.append(a[i])
        return [b, max(Ys)]

def findHoriDist(dist, w):
    hori_angle = dist * 0.04499
    # bottom side length of rectangle is 4 inches total
    # return distance in inches
    return (104-45) / math.tan(hori_angle)

def process_frame(frame):
    #frame = ResizeWithAspectRatio(frame, width=1280)
    start_time = time.time()
    h, w, d = frame.shape
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

    image_lower_bound = bound_percent_cv2(166, 85, 64, 0.5)
    image_upper_bound = bound_percent_cv2(166, 85, 64, 1.7)

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
    edges = cv2.Canny(th2, 50, 200)
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

    for contour in contours:
        M = cv2.moments(contour)
        if int(M["m00"]) != 0:
            # chooses the target with the largest pixel count
            if (cv2.contourArea(contour) == max(contourAreas)):
                Cx = int(M["m10"]/M["m00"])
                Cy = int(M["m01"]/M["m00"])
                #cv2.circle(frame, (Cx, Cy), 3, (0,0,255))
                #cv2.putText(frame, "center: ({x_val}, {y_val})".format(x_val=Cx, y_val=Cy), (20, 20),
                            #cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 1)
                # mounting height
                vertOffset = 45

                # degrees per pixel, constant measure before hand
                degPix = 0.1722487

                horiAngle = degPix * (w/2-Cx)
                vertAngle = degPix * (h/2-Cy)
                # if vertical angle is zero then this breaks, practically in a match we will never be level with the
                # target
                distance = (104-vertOffset) / (math.tan(math.pi/180 * vertAngle))

                #cv2.putText(frame, "horizontal angle: {theta}".format(theta=horiAngle), (20, 80),
                            #cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 1)
                #cv2.putText(frame, "vertical angle: {theta}".format(theta=vertAngle), (20, 200),
                             #cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 1)
                #cv2.putText(frame, "distance: {dist}".format(dist=distance), (20, 140),
                            #cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 1)
    end_time = time.time()
    print("Processing Time for 1 Frame: " + str(end_time-start_time))
    if cv2.waitKey(0) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
    return frame
    
        




def cap_frame(args):

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
    key_values = {
        49 : 1,
        50 : 2,
        51 : 3,
        52 : 4,
        53 : 5,
        54 : 6,
        55 : 7,
        56 : 8,
        57 : 9,
        113 : "q",
        114 : "r",
        115 : "s"
    }

    while cap.isOpened():
        x = ord(input("How many frames: "))

        if x in key_values:

            if key_values[x] == "q":
                cap.release()
                return
            if key_values[x] == "r":
                cap.release()
                print("released")
                if (bool(args['--camera-windows'])):
                    cap = cv2.VideoCapture(int(args['--device-number']))
                if (bool(args['--camera-linux'])):
                    device_path = str(args[--camera-linux])
                    cap = cv2.VideoCapture(device_path[len(device_path)-1])
                cap.set(cv2.CAP_PROP_EXPOSURE, -11)
                continue

            frames = []
            for i in range(0, key_values[x]):

                ret, frame = cap.read()
                # frame = cv2.imread("C:/Users/vivek/Desktop/stuffff/multiple-targets.png")
                if ret == False:
                    print("broke -> " + str(cap.isOpened()))
                    cap.release()
                    print("released " + str(cap.isOpened()))
                    quit()

                frames.append(frame)



            #print(str(frames) + "\n")
            with multiprocessing.Pool( processes=5) as pool:
                processed_frames = pool.map(process_frame, frames)
                #processed_frames = pool.map(test_process_frame, frames)


            a = 1
            for frame in processed_frames:
                #print(frame)
                #print("\n")
                frame = ResizeWithAspectRatio(frame, width=640)
                cv2.imshow("Processed frames " + str(a), frame)
                a += 1
            if cv2.waitKey(0) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                continue

if __name__ == "__main__":
    args = docopt(__doc__)
    cap_frame(args)
