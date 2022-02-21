import cv2
import numpy as np
import imutils
from create_bound import *
import math
import Constants
import os

def initialFrameProcessing(frame):
    im = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    image_lower_bound = bound_percent_cv2(165, 86, 60, 0.9)
    image_upper_bound = bound_percent_cv2(165, 86, 60, 1.2)

    imageFilter = cv2.inRange(im, image_lower_bound, image_upper_bound)
    imageFiltered = cv2.bitwise_and(im, im, mask=imageFilter)
    imageFiltered = cv2.cvtColor(imageFiltered, cv2.COLOR_HSV2BGR)

    grayscale_im = cv2.cvtColor(imageFiltered, cv2.COLOR_BGR2GRAY)
    ret2, th2 = cv2.threshold(grayscale_im, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    contours = cv2.findContours(th2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    contoursList = imutils.grab_contours(contours)

    return contoursList

def getContourPoints(contour, frame):
    pointsX = []
    pointsY = []

    roiBox = cv2.minAreaRect(contour)
    box = cv2.boxPoints(roiBox)
    box = np.int0(box)
    #cv2.drawContours(frame, [box], 0, (0,0,255), 2)
    for point in box:
        pointsX.append(point[0])
        pointsY.append(point[1])
    return {"xPoints":pointsX, "yPoints":pointsY}

def contourDimensions(contour, frame):
    points = getContourPoints(contour, frame)
    pointsX = points["xPoints"]
    pointsY = points["yPoints"]
    #cv2.circle(frame, (max(pointsX), max(pointsY)), 3, (236, 181, 0))
    #cv2.circle(frame, (min(pointsX), min(pointsY)), 3, (236, 181, 0))
    roiWidth = max(pointsX)-min(pointsX)
    roiHeight = max(pointsY)-min(pointsY)

    return {"roiWidth":roiWidth, "roiHeight":roiHeight}

def getContours(contours, center, frame):
    returnContours = []
    Cx, Cy = center
    smallestDistCenter = 2 * Cx
    for contour in contours:
        contourDim = contourDimensions(contour, frame)
        contourRatio = contourDim["roiWidth"]/contourDim["roiHeight"]

        contourArea = cv2.contourArea(contour)

        contourMoments = cv2.moments(contour)
        if (contourMoments["m00"] != 0):
            contourCy = int(contourMoments["m01"]/contourMoments["m00"])
            contourCx = int(contourMoments["m10"]/contourMoments["m00"])
            contourDistanceCenter = math.sqrt( (contourCx-Cx)**2 + (contourCy-Cy)**2 )
        else:
            continue

        print(f"Contour Aspect Ratio: {contourRatio}")
        print(f"Contour Area: {contourArea}")

        if ( (contourArea >= Constants.TARGET_MIN_AREA) and (contourArea <= Constants.TARGET_MAX_AREA) ):
            if ( (contourRatio >= 0.9*Constants.TARGET_ASPECT_RATIO) and (contourRatio <= Constants.TARGET_MAX_AREA) ):
                if (contourDistanceCenter <= smallestDistCenter):
                    smallestDistCenter = contourDistanceCenter
                    returnCx, returnCy = contourCx, contourCy
                    returnContours = []
                    returnContours.append(contour)
    if not returnContours:
        return None, None
    return returnContours, (returnCx, returnCy)

def getBoundingBoxCenter(contours):
    pointsX = []
    pointsY = []

    for contour in contours:
        pointsX.append(getContourPoints(contour)["xPoints"])
        pointsY.append(getContourPoints(contour)["yPoints"])

    pointsX = [point for contourPoint in pointsX for point in contourPoint]
    pointsY = [point for contourPoint in pointsY for point in contourPoint]

    Cx = ((max(pointsX) + min(pointsX))/2)
    Cy = ((max(pointsY) + min(pointsY))/2)

    return (int(Cx), int(Cy))
def getHoriAngle(xValue, imageWidth):
    # the horizontal FOV of the c920e is 70.42 degrees
    cameraHorizontalFOV = 70.42
    degPixHFOV = cameraHorizontalFOV/imageWidth
    horiAngle = (degPixHFOV * ((imageWidth/2)-xValue))
    return horiAngle

def getVertAngle(yValue, imageHeight, angleOffset):
    # the vertical FOV of the c920e is 43.3 degrees
    cameraVerticalFOV = 43.3
    degPixVFOV = cameraVerticalFOV/imageHeight
    vertAngle  = (degPixVFOV * ((imageHeight/2)-yValue)) + angleOffset
    return vertAngle

def getDistanceHori(horiAngle, imageWidthInches):
    return (0.5*imageWidthInches)/math.atan(horiAngle)

def getDistanceVert(targetHeight, mountingHeight, vertAngle):
    return (targetHeight-mountingHeight) / math.tan(vertAngle * (math.pi/180))

def twoPointCal(m, b, val):
    return (m*val)+b

def main(frame):
    h, w, d = frame.shape

    contours = initialFrameProcessing(frame)


    validContours, validCenter = getContours(contours, (w/2, h/2), frame)
    if (validContours == None or validCenter == None):
        cv2.imshow("No Target Found", frame)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
    else:


        #cv2.drawContours(frame, validContours, -1, (0,255,0), 1)
        """cv2.imshow("valid contours", frame)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            cv2.destroyAllWindows()"""

        Cx, Cy = validCenter

        cv2.circle(frame, (Cx, Cy), 3, (255, 255, 255))

        horiAngle = getHoriAngle(Cx, w)
        vertAngle = getVertAngle(Cy, h, Constants.ANGLE_OFFSET)

        distanceHori = getDistanceHori(horiAngle, Constants.TARGET_WIDTH)
        distanceVert = getDistanceVert(Constants.TARGET_HEIGHT, Constants.MOUNTING_HEIGHT, vertAngle)

        distanceHoriCalib = twoPointCal(1.0526, -27.711, distanceHori)
        distanceVertCalib = twoPointCal(1.0862, -1.09967, distanceVert)

        print(f"Horizontal Angle: {horiAngle} \nVertical Angle: {vertAngle} \n"
              f"distance using vertical angle: {distanceVert} \ndistance using horizontal angle: {distanceHori}")

        cv2.imshow("frame", frame)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

if __name__ == "__main__":
    successfulImages = "C:/Users/vivek/Desktop/Vision Test Images/Successful/"
    testImages = "C:/Users/vivek/Desktop/Vision Test Images/Test/"
    dir = successfulImages
    imgDir = os.listdir(dir)
    for img in imgDir:
        print(img)
        file = dir + img
        #print(file
        frame = cv2.imread(file)
        main(frame)

