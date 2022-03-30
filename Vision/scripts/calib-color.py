import PIL.Image
import cv2
import os
import glob
import numpy

def cameraCalib(patternDim):
    # vector to store 3D points within the real world for checkerboard image
    objpoints = []
    # vector to store 2D points within the image for checkerboard image
    imgpoints = []
    
    # termination criteria for iterative method, cornerSubPix
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # defining real world conditions for 3D points
    objp = = np.zeros((1, patternDim[0] * patternDim[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:patternDim[0], 0:patternDim[1]].T.reshape(-1,2)
    prev_img_shape = None
    
    # Extractingpath of each image in a directory
    images = glob.glob('./camera-calib-images/*.jpg')
    for image in images:
        img = cv2.imread(image)
        gray = cv2.cvtColor(cv2.COLOR_BGR2GRAY)
        
        # find chess board corners; ret = True if desired number of corners 
        # are found 
        ret, corners = cv2.findChessboardCorners(gray, patternDim, 
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + 
        cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        # if desired number of corners are found
        if ret == True:
            objpoints.append(objp)
            
            # using cv2 functions to increase precision of pixel corners selected
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1,-1), criteria)
            
            imgpoints.append(corners2)
            
            # draw and display corners
            img = cv2.drawChessboardCorners(img, CheckerBoard, corners2, ret)
            
        cv2.imshow('frame', img)
        cv2.waitKey(0)
    cv2.destroyAll Windows()
    h,w = img.shape[:2]
    
    # this returns in the following order:
    # retval (True or False)
    # mtx (contains the intrinsic parameters of the camera -> *should be* 3 x 3)
    # dist (contains the lens distortion coefficients)
    # rvecs (contains the rotation vector -> 3 x 1)
    # tvecs (contains the translation vector -> 3 x 1)
    return cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

def undistort(imgParam, mtx, dist):
    img = cv2.imread(imgParam)
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    
    # undistort image
    dst = cv2.undistort(img, mtx, dist, None, newcamermtx)
    
    # crop the image
    x, y, w ,h = roi
    dst = dst[y:y+h, x:x+w]
    
    # returns the undistorted image
    return dst
    
def rgb_to_hsv(input):
    r, g, b = input
    r, g, b = r/255.0, g/255.0, b/255.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    df = mx-mn
    if mx == mn:
        h = 0
    elif mx == r:
        h = (60 * ((g-b)/df) + 360) % 360
    elif mx == g:
        h = (60 * ((b-r)/df) + 120) % 360
    elif mx == b:
        h = (60 * ((r-g)/df) + 240) % 360
    if mx == 0:
        s = 0
    else:
        s = (df/mx)*100
    v = mx*100
    return h, s, v

# for linux specify video capture driver (V4L2)
cap = cv2.VideoCapture(8, cv2.CAP_V4L2)
# For V4L2 and Logitech c920 1 is manual exposure, 3 is auto exposure
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
# cap_prop_exposure is in terms of milliseconds
cap.set(cv2.CAP_PROP_EXPOSURE, 10)

ret, img = cap.read()
imgGrayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, imgOTSU = cv2.threshold(imgGrayscale, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
imgPIL = PIL.Image.open(img_path)
imgPILRGB = imgPIL.convert("RGB")

M = cv2.moments(imgOTSU)
Cx = int(M["m10"]/M["m00"])
Cy = int(M["m01"]/M["m00"])
cv2.circle(img, (Cx, Cy), 3, (255,255,255))
rgbVal = imgPILRGB.getpixel((Cx,Cy))
print(rgb_to_hsv(rgbVal))

cv2.imshow("Input Frame", img)

if cv2.waitKey(0) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                
if __name__ == "__main__":
    

