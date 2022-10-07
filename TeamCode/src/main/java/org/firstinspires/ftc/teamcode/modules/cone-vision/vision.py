import numpy as np
import cv2
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
ap.add_argument("-c", "--color", required=True, help="Color")
args = vars(ap.parse_args())

img = cv2.imread(args["image"])
cv2.imshow("Original", img)

print(img.shape)
height, width, c = img.shape
mask = np.zeros((height,width), np.uint8)
circle_img = cv2.circle(mask,(width//2, height//2), width*7//18,(255,255,255),thickness=-1)
circle_crop = cv2.bitwise_and(img, img, mask=circle_img)
cv2.imshow("Circle", circle_crop)


img =  cv2.GaussianBlur(circle_crop, (3, 3), 0)
cv2.imshow("Blurred", img)

img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


if args["color"] == "red":
    mask1 = cv2.inRange(img_hsv, (0,50,20), (5,255,255))
    mask2 = cv2.inRange(img_hsv, (175,50,20), (180,255,255))
    mask = cv2.bitwise_or(mask1, mask2 )
elif args["color"] == "blue":
    mask = cv2.inRange(img_hsv, (100,150,0), (140,255,255))
elif args["color"] == "silver":
    mask = cv2.inRange(img_hsv, (100,150,60), (140,255,255))

cropped = cv2.bitwise_or(img, img, mask=mask)
cv2.imshow("Filtered", cropped)

gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
cv2.imshow("Gray", gray)
edged = cv2.Canny(gray, 30, 150)
cv2.imshow("Edges", edged)

contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
img_copy = img.copy()
cv2.drawContours(img_copy, contours, -1, (0, 255, 0), 2)
cv2.imshow("Contours", img_copy)

cv2.waitKey(0)