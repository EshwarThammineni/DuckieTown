import rospy
from roller_eye.msg import frame
import numpy as np
import cv2

def callback(msg):
    np_arr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if img is not None:
        cv2.imshow("Moorebot Camera", img)
        cv2.waitKey(1)

def cleanup():
    cv2.destroyAllWindows()
    print("OpenCV windows closed.")

rospy.init_node("moorebot_camera_viewer")
rospy.Subscriber("/CoreNode/jpg", frame, callback)
rospy.on_shutdown(cleanup)  # <-- ensures cleanup runs on Ctrl+C
rospy.loginfo("Subscribed to /CoreNode/jpg... displaying video")
rospy.spin()
