import numpy as np
import cv2
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
args = vars(ap.parse_args())

img = cv2.imread(args["image"])
cv2.imshow("Original", img)


copy = img.copy()

img = cv2.imread(args["image"], cv2.IMREAD_GRAYSCALE)

cv2.imshow("Grayscale", img)

height, width = img.shape
mask = np.zeros((height,width), np.uint8)
circle_img = cv2.circle(mask,(width//2, height//2), width*7//18,(255,255,255),thickness=-1)
img = cv2.bitwise_and(img, img, mask=circle_img)
cv2.imshow("Circle", img)


circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 20, param1=130, param2=30, minRadius=0, maxRadius=width//6)
circles = np.uint16(np.around(circles))

for i in circles[0,:]:
    cv2.circle(copy, (i[0],i[1]), i[2],(0,255,0), 2)

cv2.imshow("circles", copy)
cv2.waitKey(0)