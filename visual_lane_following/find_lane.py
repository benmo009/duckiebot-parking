import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Load the image
img = cv2.imread('practice_image.png')
# plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
# plt.show()

# Cut the image
img = img[120:180, :]
# plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
# plt.show()

# Warp the image to get an aerial view
image_H = (img.shape)[0]
image_W = (img.shape)[1]

new_Left = 190
new_Right = 450
car_pos = new_Right - new_Left / 2
car_pos = np.array([car_pos, image_H-1], dtype=int)
src = np.float32([[0, image_H], [image_W, image_H], [0,0], [image_W, 0]])
dst = np.float32([[new_Left,image_H], [new_Right, image_H], [0,0], [image_W, 0]])
M = cv2.getPerspectiveTransform(src, dst)
Minv = cv2.getPerspectiveTransform(dst, src)  # Use to unwarp the image

img_warped = cv2.warpPerspective(img, M, (image_W, image_H))
#plt.imshow(cv2.cvtColor(img_warped, cv2.COLOR_BGR2RGB))
#plt.scatter(car_pos[0], car_pos[1])
#plt.show()

# # Sharpen the warped image
# gb = cv2.GaussianBlur(img_warped, (5,5), 20.0)
# img_warped = cv2.addWeighted(img_warped, 2, gb, -1, 0)
# plt.imshow(cv2.cvtColor(img_warped, cv2.COLOR_BGR2RGB))
# plt.show()

# # Increase the contrast of the image
# img2 = cv2.multiply(img_warped, np.array([1.0]))
# img_warped = img2
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
#cv2.imshow('Aerial View', img_warped)
#cv2.imshow('HSV Image', img_hsv)
#cv2.imshow('Combined Mask', combined_filtered)

img_filtered = cv2.bitwise_and(img_warped, img_warped, mask=combined_filtered)
#cv2.#('Filtered - Combined', img_filtered)

#cv2.waitKey(0)

#cv2.imwrite('aerial.png', img_warped)
#cv2.imwrite('filtered.png', img_filtered)

# Fid the points on the lane lines to a polynomial
# Convert to binary image
img_yellow_bin = img_yellow / 255
#cv2.imshow('Binary Image', img_yellow_bin)
#cv2.waitKey(0)

# Try using polyfit
data = np.argwhere(img_yellow_bin == 1)
# plt.scatter(data[:,1], data[:,0])
# plt.scatter(car_pos[0],car_pos[1])
# plt.show()


# Draw the polynomial over the lines

# Find points to the left of car_pos
window_H = 10
window_W = 10
# Go down until finding row with ones
points = []
for i in range(window_H):
    row = img_yellow_bin[car_pos[1]-i, 0:car_pos[0]]
    row = np.where(row)
    if row[0].size != 0:
        middle = int(np.mean(row))
        points.append([car_pos[1]-i, middle])

# Convert to numpy array
points = np.asarray(points)

# Find a 1-degree fit to the points
p = np.polyfit(points[:, 1], points[:, 0], 1)
p = np.poly1d(p)
print("Fitted polynomial:")
print(p)

# Parameters for drawing a box
window_center = points[0,1]
window_center = np.array([window_center, car_pos[1]])
x = int(window_center[0] - 0.5*window_W)
y = int(window_center[1] - window_H)

# Calculate distance from center of lane
distance_from_center = car_pos[0] - window_center[0]

# Check what direction the angle should be
# If the slope of the polynomial is negative, turn left
# If the slope of the polynomial is positive, turn right
angle_from_straight = math.atan2(1, abs(p[1]))

print("Distance from Center = {:d}".format(distance_from_center))
print("Angle from Straight = {:.2f}".format(angle_from_straight))


# Plot
fig, ax = plt.subplots(1)
ax.scatter(data[:, 1], data[:, 0])
ax.scatter(car_pos[0], car_pos[1])
rect = patches.Rectangle((x, y), window_W, window_H,
                         linewidth=1, edgecolor='r', facecolor='none')
ax.add_patch(rect)
x_axis = np.linspace(x, x+window_W, 100)
ax.plot(x_axis, p(x_axis), color='red')
plt.show()
