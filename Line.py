import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from red_stop import RedStopController
from apriltag_stop import AprilTagStopController

class MoorebotLaneFollower(Node):
    def __init__(self):
        super().__init__('moorebot_lane_follower')
        self.red_stop = RedStopController(stop_time=3.0) #for red logic
        self.waiting_for_red_clear = False
        self.bridge = CvBridge()
        self.latest_image = None
        #april tag
        self.apriltag_stop = AprilTagStopController(stop_tag_id=162, stop_time=3.0)
        self.subscription = self.create_subscription(
            Image,
            '/duckiescrooge/camera/image',
            self.image_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/duckiescrooge/cmd_vel', 10)

        # (0.02 = approx 50 hz)
        self.timer = self.create_timer(0.02, self.process_image)

        # Motion params
        self.linear_speed = 0.2
        self.angular_gain = 0.015# gain for steering( how hard should it turn if off center)
        self.min_angular = 0.1  # minimum torque for turning

        # Lane smoothing
        self.lane_center_history = []

        self.get_logger().info("Moorebot lane follower active")

    def image_callback(self, msg):
        self.latest_image = msg

    def process_image(self):
        if self.latest_image is None:
            return
        # Ros image to cv2 image format
        frame = self.bridge.imgmsg_to_cv2(self.latest_image, 'rgb8')
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.resize(frame, (320, 240))
        h, w, _ = frame.shape
        roi = frame[int(h * 0.70):h, :]  # bottom 45% of the image is used in the program

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV) #apprently rgb sucks and hsv is better for minimizing lighting changes
        tag_speed = self.apriltag_stop.update(frame)

        # yellow
        lower_yellow = np.array([15, 60, 60])#hsv format (hue,saturation,value)
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)



        # white
        lower_white = np.array([0, 0, 160])
        upper_white = np.array([180, 40, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        #red
        lower_red1 = np.array([0, 150, 150])
        upper_red1 = np.array([10, 255, 255])
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        lower_red2 = np.array([170, 150, 150])
        upper_red2 = np.array([180, 255, 255])
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        red_mask = cv2.bitwise_or(red_mask1, red_mask2)



        #places gray dot on what it think the edge is.
        white_cx = self.get_largest_white_centroid(white_mask)
        if white_cx is not None:
            cv2.circle(white_mask, (white_cx, 20), 5, (128, 128, 128), -1)  # gray dot

        #essensially makes the white lines solid with morphology ex for hole filling

        kernel = np.ones((7, 7), np.uint8)#any gap smaller than 7 pixels gets filled
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

        #yellow_cx = self.get_centroid(yellow_mask)#finds biggest yellow contour


        yellow_cx, yellow_cy = self.get_centroid(yellow_mask)
        # Ignore yellow too high in ROI
        if yellow_cy is not None and yellow_cy < roi.shape[0] * .1:
            yellow_cx = None

        white_cx = self.get_largest_white_centroid(white_mask)#chooses biggest white contour

        #print("RED PIXELS:", cv2.countNonZero(red_mask))
        speed = self.red_stop.update(red_mask)



        #offset
        if white_cx is not None:
            white_cx = white_cx + 50
        #debug = roi.copy()
        full_debug = frame.copy()
        debug = roi.copy()

        img_center = w // 2#img_center is where the moorebot wants to go


        # Draw reference center
        cv2.line(debug, (img_center, 0), (img_center, roi.shape[0]), (0, 0, 255), 2)

        # Draw detected centroid            cv2.circle(white_mask, (white_cx, 20), 5, (128, 128, 128), -1)  # gray dot
        if yellow_cx is not None:
            cv2.circle(debug, (yellow_cx, 40), 6, (0, 255, 255), -1)
        if white_cx is not None:
            cv2.circle(debug, (white_cx, 40), 6, (255, 255, 255), -1)
        # ---------------- Compute lane center ----------------
        twist = Twist()

        if yellow_cx is not None and white_cx is not None:
            lane_center = (yellow_cx + white_cx) // 2
        elif yellow_cx is not None:
            lane_center = yellow_cx + 145  #60 needs to be tuned, bias toward inside
        elif white_cx is not None:
            lane_center = white_cx - 145  # bias toward outside
        else:
            lane_center = None

        if lane_center is not None:
            # moving average to smooth zig-zag
            self.lane_center_history.append(lane_center)
            if len(self.lane_center_history) > 5:
                self.lane_center_history.pop(0)
            lane_center_smoothed = int(np.mean(self.lane_center_history))

            # Draw lane center

            cv2.arrowedLine(debug, (img_center, roi.shape[0] - 5),
                            (lane_center_smoothed, roi.shape[0] - 5),
                            (255, 0, 0), 2)

            #Draws its percieved middle in blue
            cv2.circle(debug, (lane_center_smoothed, 40), 6, (255, 0, 0), -1)

            # Steering
            error = img_center - lane_center_smoothed #if lane center is left, its positive and angular z is positive (P Controller for reference)
            angular_cmd = self.angular_gain * error

            # minimum torque
            if 0 < abs(angular_cmd) < self.min_angular:
                angular_cmd = np.sign(angular_cmd) * self.min_angular

            # clamp max angular speed
            angular_cmd = np.clip(angular_cmd, -1.2, 1.2)#abs limit of angular z, needs to be tuned

            twist.angular.z = float(angular_cmd)
            twist.linear.x = self.linear_speed * speed * tag_speed

            if speed == 0.0:
                twist.angular.z = 0.0
            else:
                twist.angular.z = float(angular_cmd)

        else:
            # lost lane, searches for the lines instead of stopping
            twist.linear.x = 0.0
            twist.angular.z = 0.3
            cv2.putText(debug, "NO LANES", (90, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Publish command
        self.cmd_pub.publish(twist)

        # Debug windows
        cv2.namedWindow("Lane Debug", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Lane Debug", 600, 600)

        cv2.namedWindow("White Mask", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("White Mask", 600, 600)

        cv2.namedWindow("Yellow Mask", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Yellow Mask", 600, 600)

       # cv2.imshow("Lane Debug", debug)
        cv2.imshow("Full Frame (Tags)", full_debug)
        cv2.imshow("Lane ROI", debug)

        cv2.imshow("White Mask", white_mask)
        cv2.imshow("Yellow Mask", yellow_mask)
        cv2.waitKey(1)

    # ---------------- HELPERS ----------------

    #this version returns y coordinate
    def get_centroid(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None, None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy

    """
    def get_centroid(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:#area of contour is m00 and avoids dividing by zero
            return None
        return int(M["m10"] / M["m00"])#sum of x coordinates is m10
    """

    def get_largest_white_centroid(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        max_area = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area > max_area:
                best = c
                max_area = area
        if best is None:
            return None
        M = cv2.moments(best)
        if M["m00"] == 0:
            return None
        return int(M["m10"] / M["m00"])

def main():
    rclpy.init()
    node = MoorebotLaneFollower()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
