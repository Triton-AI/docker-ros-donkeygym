import cv2
import numpy as np

frame = cv2.imread('Image_screenshot_05.04.2021.png')

height, width, _ = frame.shape

rgb = frame[int(height/3):height, 0:width]
rgbsidefilter = {
            "lowR": 216,
            "highR": 255,
            "lowG": 222,
            "highG": 255,
            "lowB": 198,
            "highB": 255
}



lowR = rgbsidefilter.get("lowR")
highR = rgbsidefilter.get("highR")
lowG = rgbsidefilter.get("lowG")
highG = rgbsidefilter.get("highG")
lowB = rgbsidefilter.get("lowB")
highB = rgbsidefilter.get("highB")

lower = np.array([lowR, lowG, lowB])
higher = np.array([highR, highG, highB])
mask = cv2.inRange(rgb, lower, higher)

# epsilon = 0.1*cv2.arcLength(cnt,True)
# approx = cv2.approxPolyDP(cnt,epsilon,True)

cv2.imshow("1", mask)
cv2.waitKey()
