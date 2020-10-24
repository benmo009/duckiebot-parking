import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image
img = cv2.imread('practice_image.png')
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.show()

# Use edge detection to remove the sky and just have road

# For now, manually cut the image
img = img[150::, :]
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.show()

image_H = (img.shape)[0]
image_W = (img.shape)[1]

new_Left = 250
new_Right = 480
src = np.float32([[0, image_H], [image_W, image_H], [0,0], [image_W, 0]])
dst = np.float32([[new_Left,image_H], [new_Right, image_H], [0,0], [image_W, 0]])
M = cv2.getPerspectiveTransform(src, dst)

warped_img = cv2.warpPerspective(img, M, (image_W, image_H))
plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB))
plt.show()
