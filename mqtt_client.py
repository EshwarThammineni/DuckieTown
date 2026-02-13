import os, rospy, socket, sys
import paho.mqtt.client as mqtt

# running on scout as a service

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Range, Imu, Image, Illuminance
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from roller_eye.msg import frame, status

class MqttClient:
    def __init__(self, host, port, ip, name):
        rospy.init_node('mqtt_client', anonymous=True)
        self._name            = name

        self._pose_pub        = rospy.Publisher('positions', String, queue_size=1)
        self._pix_pub         = rospy.Publisher('pixel_pos', String, queue_size=1)
        self._cmd_vel_pub     = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self._image_sub       = rospy.Subscriber('CoreNode/jpg', frame, self.image_callback)
        self._baselink_odom_sub = rospy.Subscriber('MotorNode/baselink_odom_relative', Odometry, None)
        self._vio_odom_sub     = rospy.Subscriber('MotorNode/vio_odom_relative', Odometry, None)
        self._battery_sub     = rospy.Subscriber('SensorNode/simple_battery_status', status, None)
        self._imu_sub         = rospy.Subscriber('SensorNode/imu', Imu, self.imu_callback)
        self._light_sub       = rospy.Subscriber('SensorNode/light', Illuminance, self.light_callback)
        self._tof_sub         = rospy.Subscriber('SensorNode/tof', Range, self.tof_callback)

        self._mqtt_client_obj = mqtt.Client()
        self._mqtt_client_obj.connect(host, port)
        self._mqtt_client_obj.on_connect = self.on_connect
        self._mqtt_client_obj.on_message = self.on_message
        self._mqtt_client_obj.loop_start()

    def on_connect(self, client, userdata, flags, reason_code, properties=None):
        self._mqtt_client_obj.subscribe("lobby")
        self._mqtt_client_obj.subscribe(self._name+"/cmd_vel")
        self._mqtt_client_obj.publish("lobby", self._name + "::connected")
        print("Connected2 with result code "+str(reason_code))

    def on_message(self, client, userdata, message):
        topic, msg = message.topic, message.payload.decode('utf-8')
        #print("Message received on topic ", topic, "with payload", msg)
        if   'lobby'      in topic: self.lobby_handler(msg)
        elif 'cmd_vel' and self._name in topic:
            self.cmd_vel_channel_handler(msg)

    def lobby_handler(self, msg):
        #print("Lobby message: ", msg)
        if "check-alive" in msg: # check for heartbeat in msg
            self._mqtt_client_obj.publish("lobby", self._name+"::alive")

    def cmd_vel_channel_handler(self, msg):
        #print("Cmd_vel message: ", msg)
        vals = msg.split(" ")
        ly = float(vals[0].strip()) * 1
        lx = float(vals[1].strip()) * 1
        az = float(vals[2].strip()) * 1.5
        self._cmd_vel_pub.publish(Twist(Vector3(lx, ly, 0), Vector3(0, 0, az)))

    def image_callback(self, data):
        # Convert the ROS Image message to a byte array
        jpg_bytes = data.data
        # Publish the byte array to the MQTT topic
        self._mqtt_client_obj.publish(self._name+"/camera/image", jpg_bytes)
        #print("Published image to MQTT topic")

    def baselink_odom_callback(self, data):
        print("Baselink odom data received.")

    def vio_odom_callback(self, data):
        print("VIO odom data received.")

    def battery_callback(self, data):
        print("Battery data received.")

    def imu_callback(self, data):
        # print(data.header)
        # print(data.angular_velocity)
        # print(data.linear_acceleration)

        data_string = str(data.angular_velocity.x) + " "
        data_string += str(data.angular_velocity.y) + " "
        data_string += str(data.angular_velocity.z) + "\n"
        data_string += str(data.linear_acceleration.x) + " "
        data_string += str(data.linear_acceleration.y) + " "
        data_string += str(data.linear_acceleration.z)

        self._mqtt_client_obj.publish(self._name+"/sensor/imu", data_string)

    def light_callback(self, data):
        #print(data)
        #print(data.illuminance)
        self._mqtt_client_obj.publish(self._name+"/sensor/light", str(data.illuminance))

    def tof_callback(self, data):
        # print(data.range)
        self._mqtt_client_obj.publish(self._name+"/sensor/tof", str(data.range))

    def start(self):
        rospy.spin()

broker = xxx.xxx.x.xx
moorebot = xxx.xxx.x.xx
name = "DuckieDonald
if __name__ == '__main__':
    client = MqttClient(broker, 1883, moorebot, name)
    client.start()
