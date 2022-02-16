import PIL.Image
import cv2

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

