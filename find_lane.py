import cv2
import math
import numpy as np
import matplotlib.pyplot as plt

# Function to solve quadratic function
def quadratic_formula(p):
    [a, b, c] = p

    d = b**2 - 4*a*c
    if d < 0:
        return -1
    
    x1 = ( -b + np.sqrt(d) ) / (2*a)
    x2 = ( -b - np.sqrt(d) ) / (2*a)

    return (x1, x2)


# Function for cropping and then warping the image
def warp_image(img, left, right):
    # Get the dimensions of the image
    (image_h, image_w, _) = img.shape

    # Set the car position to the center of the window
    car_pos = ((right - left) / 2) + left
    car_pos = np.array([car_pos, image_h-1], dtype=int)

    # Warp the image to get an areal view
    src = np.float32([[0, image_h], [image_w, image_h], [0,0], [image_w, 0]])
    dst = np.float32([[left,image_h], [right, image_h], [0,0], [image_w, 0]])
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)  # Use to unwarp the image

    img_warped = cv2.warpPerspective(img, M, (image_w, image_h))

    return img_warped, car_pos, Minv


# Takes an image and returns a filtered binary image
def get_binary_image(img, lower, upper):
    # Convert to HSV color space
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Filter out the specified color
    img_filtered = cv2.inRange(img_hsv, lower, upper)

    # Divide by 255 to convert to binary
    img_bin = img_filtered / 255

    return img_bin


# Takes binary image and estimates the center lane line
def estimate_lane(img, window_h=70, window_w=50):
    data = np.argwhere(img == 1)

    start_pos = np.max(data[:,0])

    # Get the middle point for each row
    # Only look at points within the window dimensions
    # For each row, shift the window to be centered at the previous middle point
    points = []
    middle = 0
    left = 0
    for i in range(window_h):
        y = start_pos - i
        if i == 0:
            # For the first row, look in the entire row
            row = img[y,::]
            
        else:
            # Compute left and right bounds to look in
            left = middle - window_w // 2
            right = middle + window_w // 2
            row = img[y, left:right]

        # Find all the points in the row thats a 1
        ones = np.where(row)

        # Recordthe middle point 
        if ones[0].size != 0:
            middle = int(np.mean(ones)) + left
            points.append([y, middle])

    # Convert to numpy array
    points = np.asarray(points)

    # Fit the points to a 1-degree polynomial
    p_lin = np.polyfit(points[:,1], points[:,0], 1)
    p_lin = np.poly1d(p_lin)
    # Fit the points to a 2-degree polynomial
    p_quad = np.polyfit(points[:,1], points[:,0], 2)
    p_quad = np.poly1d(p_quad)

    return p_lin, p_quad, data


# Compute the angle from vertical to the estimated line
def compute_theta(p, x1, x2):
    # Compute the two points
    p1 = [ x1, p(x1) ]
    p2 = [ x2, p(x2) ]

    # Shift p2 such that p1 is at (0,0).
    p2[1] = abs(p2[1] - p1[1])
    p2[0] = p2[0] - p1[0]

    theta = math.atan2(p2[0], p2[1])

    return theta


# Compute the distance of the car from the estimated line
def compute_dist(quad, car_pos, points, n=10):
    y = car_pos[1]  # car's y position
    x = quadratic_formula(quad - y)

    # Take the solution that is closer to the points in binary image
    avg_distance = np.array([0, 0])
    # Compute each solution's distance to the n points at the hightes y value
    for point in points[-n:-1,:]:
        avg_distance[0] += np.linalg.norm(point - [y, x[0]])
        avg_distance[1] += np.linalg.norm(point - [y, x[1]])

    avg_distance = avg_distance / n

    # Take the point with smaller average distance
    x = x[np.argmin(avg_distance)]

    # Compute the distance
    dist = car_pos[0] - x

    return dist


# Function that takes in an image and returns the estimate distance from the center line and angle
def estimate_position(img, crop_val=120, new_left=270, new_right=350):
    # Crop the image with specified value
    img = img[crop_val::, :]

    # Warp the image
    img_warped, car_pos, Minv = warp_image(img, new_left, new_right)

    # Filter out the yellow dotted lines
    yellow_lower = np.array([20, 75, 100])
    yellow_upper = np.array([30, 255, 255])

    img_bin = get_binary_image(img_warped, yellow_lower, yellow_upper)

    # Estimate the lane as a 1 dimensional polynomial
    p_lin, p_quad, points = estimate_lane(img_bin)
    print(p_lin)
    print(p_quad)

    # Compute the angle from the estimated line
    # positive angle -> turn right
    # negative angle -> turn left
    theta = compute_theta(p_lin, new_left, new_right)
    
    # Calculate distance from center of lane
    dist = compute_dist(p_quad, car_pos, points)

    return dist, theta



if __name__ == '__main__':
    image_file = 'duckietown_images/7.png'
    img = cv2.imread(image_file)
    dist, theta = estimate_position(img)
    print('Distance from Center Line = {:.2f}'.format(dist))
    print('Angle from Straight = {:.2f} rad'.format(theta))
