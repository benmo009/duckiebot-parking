import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image
img = cv2.imread('practice_image.png')
# plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
# plt.show()

# Cut the image
img = img[120::, :]
# plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
# plt.show()

# Warp the image to get an aerial view
image_H = (img.shape)[0]
image_W = (img.shape)[1]

new_Left = 270
new_Right = 350
src = np.float32([[0, image_H], [image_W, image_H], [0,0], [image_W, 0]])
dst = np.float32([[new_Left,image_H], [new_Right, image_H], [0,0], [image_W, 0]])
M = cv2.getPerspectiveTransform(src, dst)
Minv = cv2.getPerspectiveTransform(dst, src)  # Use to unwarp the image

img_warped = cv2.warpPerspective(img, M, (image_W, image_H))
# plt.imshow(cv2.cvtColor(img_warped, cv2.COLOR_BGR2RGB))
# plt.show()

# Find bounds on the lanes

# Transform image to HSV color space
img_hsv = cv2.cvtColor(img_warped, cv2.COLOR_BGR2HSV)
# plt.imshow(img_hsv)
# plt.show()

# Filter out everything but yellow (dotted lines) and white (side lines)
yellow_lower = np.array([20, 75, 100])
yellow_upper = np.array([30, 255, 255])
white_lower = np.array([0,0,1])
white_upper = np.array([120,5,255])

img_yellow = cv2.inRange(img_hsv, yellow_lower, yellow_upper)
img_white = cv2.inRange(img_hsv, white_lower, white_upper)
combined_filtered = img_yellow + img_white

# Show all of the images
cv2.imshow('Aerial View', img_warped)
cv2.imshow('HSV Image', img_hsv)
cv2.imshow('Combined Mask', combined_filtered)

img_filtered = cv2.bitwise_and(img_warped, img_warped, mask=combined_filtered)
cv2.imshow('Filtered - Combined', img_filtered)

cv2.waitKey(0)

cv2.imwrite('aerial.png', img_warped)
cv2.imwrite('filtered.png', img_filtered)

# Use edge detection?

# Fid the points on the lane lines to a polynomial

# Draw the polynomial over the lines

