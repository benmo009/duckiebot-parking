import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image
img = cv2.imread('practice_image_3.png')
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.show()

# Cut the image
img = img[120::, :]
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.show()

# Warp the image to get an aerial view
image_H = (img.shape)[0]
image_W = (img.shape)[1]

new_Left = 270
new_Right = 350
src = np.float32([[0, image_H], [image_W, image_H], [0,0], [image_W, 0]])
dst = np.float32([[new_Left,image_H], [new_Right, image_H], [0,0], [image_W, 0]])
M = cv2.getPerspectiveTransform(src, dst)
Minv = cv2.getPerspectiveTransform(dst, src)  # Use to unwarp the image

warped_img = cv2.warpPerspective(img, M, (image_W, image_H))
plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB))
plt.show()

# Find bounds on the lanes
gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
cv2.imshow("Grayscale Image", gray)
cv2.waitKey(0)