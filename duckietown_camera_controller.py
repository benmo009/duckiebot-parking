import rospy
import numpy as np 
import cv2
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import math


class DuckiebotCamera:
    def __init__(self, show_cam=False, crop_val=120, warp_param=(270,350), window_param=(70,50)):
        # Initialize ROS Node
        self.node_name = rospy.get_name()
        rospy.loginfo("Initializing Node: [%s]" %(self.node_name))

        # Initialize subscriber Node
        self.sub = rospy.Subscriber("/image_raw", Image, self.cam_callback)

        # Initialize Publisher Node
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=0)

        # Parameters
        
        self.show_cam = show_cam
        self.crop_val = 120
        self.warp_param= warp_param
        self.window_param = window_param

        # Filter for getting the yellow dotted lines
        self.filter_lower = np.array([20, 100, 150])
        self.filter_upper = np.array([30, 255, 255])

        # Initialize rate


    # Function to solve quadratic function
    def quadratic_formula(self, p):
        [a, b, c] = p

        d = b**2 - 4*a*c
        if d < 0:
            return -1
        
        x1 = ( -b + np.sqrt(d) ) / (2*a)
        x2 = ( -b - np.sqrt(d) ) / (2*a)

        return (x1, x2)


    # Function for cropping and then warping the image
    def warp_image(self):
        # Get the dimensions of the image
        (image_h, image_w, _) = self.img.shape
        (left, right) = self.warp_param

        # Set the car position to the center of the window
        self.car_pos = ((right - left) / 2) + left
        self.car_pos = np.array([self.car_pos, image_h-1], dtype=int)

        # Warp the image to get an areal view
        src = np.float32([[0, image_h], [image_w, image_h], [0,0], [image_w, 0]])
        dst = np.float32([[left,image_h], [right, image_h], [0,0], [image_w, 0]])
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)  # Use to unwarp the image

        self.img_warped = cv2.warpPerspective(self.img, self.M, (image_w, image_h))



    # Takes an image and returns a filtered binary image
    def get_binary_image(self):
        # Convert to HSV color space
        img_hsv = cv2.cvtColor(self.img_warped, cv2.COLOR_BGR2HSV)

        # Filter out the specified color
        img_filtered = cv2.inRange(img_hsv, self.filter_lower, self.filter_upper)

        # Divide by 255 to convert to binary
        self.img_bin = img_filtered / 255



    # Takes binary image and estimates the center lane line
    def estimate_lane(self):
        (window_h, window_w) = self.window_param
        
        self.bin_points = np.argwhere(self.img_bin == 1)

        #start_pos = self.bin_points[-1,0]
        start_pos = np.max(self.bin_points[:,0])

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
                row = self.img_bin[y,::]
                
            else:
                # Compute left and right bounds to look in
                left = middle - window_w // 2
                right = middle + window_w // 2
                row = self.img_bin[y, left:right]

            # Find all the points in the row thats a 1
            ones = np.where(row)

            # Recordthe middle point 
            if ones[0].size != 0:
                middle = int(np.mean(ones)) + left
                points.append([y, middle])

        # Convert to numpy array
        points = np.asarray(points)

        # Fit the points to a 1-degree polynomial
        self.p_lin = np.polyfit(points[:,1], points[:,0], 1)
        self.p_lin = np.poly1d(self.p_lin)
        # Fit the points to a 2-degree polynomial
        self.p_quad = np.polyfit(points[:,1], points[:,0], 2)
        self.p_quad = np.poly1d(self.p_quad)



    # Compute the angle from vertical to the estimated line
    def compute_theta(self):
        # Compute the two points
        (x1, x2) = self.warp_param
        p1 = [ x1, self.p_lin(x1) ]
        p2 = [ x2, self.p_lin(x2) ]

        # Shift p2 such that p1 is at (0,0).
        p2[1] = abs(p2[1] - p1[1])
        p2[0] = p2[0] - p1[0]

        self.theta = math.atan2(p2[0], p2[1])


    # Compute the distance of the car from the estimated line
    def compute_dist(self, n=10):
        y = self.car_pos[1]  # car's y position
        x = self.quadratic_formula(self.p_quad - y)

        if x != -1:
            # Take the solution that is closer to the points in binary image
            avg_distance = np.array([0, 0])
            # Compute each solution's distance to the n points at the hightes y value
            for point in self.bin_points[-n:-1,:]:
                avg_distance[0] += np.linalg.norm(point - [y, x[0]])
                avg_distance[1] += np.linalg.norm(point - [y, x[1]])

            avg_distance = avg_distance / n

            # Take the point with smaller average distance
            x = x[np.argmin(avg_distance)]        # Take the solution that is closer to the points in binary image

        else:
            # The solution to the quadratic is imaginary, so use the linear estimate
            x = (y - self.p_lin[0]) / self.p_lin[1]

        # Compute the distance
        self.dist = self.car_pos[0] - x



    # Function that takes in an image and returns the estimate distance from the center line and angle
    def estimate_position(self):
        # Crop the image with specified value
        self.img = self.img[self.crop_val::, :]

        # Warp the image
        self.warp_image()


        self.get_binary_image()

        # Estimate the lane as a 1 dimensional polynomial
        self.estimate_lane()


        # Compute the angle from the estimated line
        # positive angle -> turn right
        # negative angle -> turn left
        self.compute_theta()
        
        # Calculate distance from center of lane
        self.compute_dist()


    def draw_estimate(self):
        cv2.namedWindow("Quadratic Estimate")
        cv2.namedWindow("Linear Estimate")

        (left, right) = self.warp_param

        img_lin = np.copy(self.img_warped)
        p1 = ( left, int(self.p_lin(left)) )
        p2 = ( right, int(self.p_lin(right)) )
        cv2.line(img_lin, p1, p2, (0, 0, 255), 5)

        img_quad = np.copy(self.img_warped)
        for x in range(left, right):
            p = ( x, int(self.p_quad(x)) )
            cv2.circle(img_quad, p, radius=4, color=(0,255,0), thickness=-1)
        
        (img_h, img_w, _) = self.img_warped.shape
        img_lin = cv2.warpPerspective(img_lin, self.Minv, (img_w, img_h))
        img_quad = cv2.warpPerspective(img_quad, self.Minv, (img_w, img_h))
        cv2.imshow("Quadratic Estimate", img_quad)
        cv2.imshow("Linear Estimate", img_lin)

        if cv2.waitKey(1)!=-1:     
            cv2.destroyAllWindows() 


    def cam_callback(self, data):
        # Convert images to OpenCV image
        bridge=CvBridge()  # CV bridge to convert image to OpenCV image
        self.img = bridge.imgmsg_to_cv2(data, "8UC3")

        # if self.show_cam:
        cv2.namedWindow("Image")

        if (self.img is not None):    
            # Use the image to estimate car pos
            self.estimate_position()            
            rospy.loginfo('Estimated dist = {:.2f}'.format(self.dist))
            rospy.loginfo('Estimated theta = {:.2f}'.format(self.theta))

            action = Twist()
            k = 0.25

            v = 0.2

            w = -k * (self.theta)

            # Use the estimates to compute steering 

            # Publish to \cmd_vel
            vel_msg = Twist()
            vel_msg.linear.x = v
            vel_msg.angular.z = w
            self.pub.publish(vel_msg)

            # Show image
            if self.show_cam:
                self.draw_estimate()
            
        if cv2.waitKey(1)!=-1:     #Burak, needs to modify this line to work on your computer, THANKS!
            cv2.destroyAllWindows() 

if __name__ == "__main__":
    rospy.init_node('duckietown_camera_node', anonymous=False)
    duckiecam = DuckiebotCamera(show_cam=True, window_param=(120,100))
    rospy.spin()