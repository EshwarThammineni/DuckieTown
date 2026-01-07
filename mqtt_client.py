#!/usr/bin/env python3 
import os, rospy, socket, sys
import paho.mqtt.client as mqtt 

# running on scout as a service 

from geometry_msgs.msg import Twist, Vector3 
from sensor_msgs.msg import Range, Imu, Image 
from std_msgs.msg import String 

from roller_eye.msg import frame

class MqttClient: 
  #constructor
    def __init__(self, host, port, ip, name): 
        rospy.init_node('mqtt_client', anonymous=True) 
        self._name            = name
        self._pose_pub        = rospy.Publisher('positions', String, queue_size=1)
        self._pix_pub         = rospy.Publisher('pixel_pos', String, queue_size=1) 
        self._cmd_vel_pub     = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        self._image_sub       = rospy.Subscriber('CoreNode/jpg', frame, self.image_callback)
        self._mqtt_client_obj = mqtt.Client()
        self._mqtt_client_obj.connect(host, port) 
        self._mqtt_client_obj.on_connect = self.on_connect 
        self._mqtt_client_obj.on_message = self.on_message 
        self._mqtt_client_obj.loop_start()

    def on_connect(self, client, userdata, flags, reason_code, properties=None): 
        self._mqtt_client_obj.subscribe("lobby")
        self._mqtt_client_obj.subscribe("command_velocities")
        print("Connected with result code "+str(reason_code))

    def on_message(self, client, userdata, message): 
        topic, msg = message.topic, message.payload.decode('utf-8') 
        print("Message received on topic ", topic, "with payload", msg)    
        if   'lobby'      in topic: self.lobby_handler(msg) 
        elif 'positions'  in topic: self.pose_handler(msg) 
        elif 'pixel_pos'  in topic: self.pixel_handler(msg) 
        elif 'transport'  in topic: self.transport_handler(msg) 
        elif 'velocities' in topic: self.velocities_handler(msg) 

    def lobby_handler(self, msg): 
        pass 

    def pose_handler(self, msg): 
        pass 

    def pixel_handler(self, msg): 
        pass

    def transport_handler(self, msg): 
        pass

    def velocities_handler(self, msg):
        try:
            msg_client_id, cmd_vel = msg.split('::')
            if self._name in msg_client_id:
                lx, ly, az = cmd_vel.split('\n')
                lx = float(lx.strip()) / 2
                ly = float(ly.strip())
                az = float(az.strip())
                twist_msg = Twist(Vector3(ly, lx, 0), Vector3(0, 0, az))
                self._cmd_vel_pub.publish(twist_msg)
                print("[VEL] Sent cmd_vel: lx={}, ly={}, az={}".format(lx, ly, az))
        except Exception as e:
            print("Error" + str(e))

    def image_callback(self, data):
        # Convert the ROS Image message to a byte array
        jpg_bytes = data.data
        # Publish the byte array to the MQTT topic
        self._mqtt_client_obj.publish(self._name+"/camera/image", jpg_bytes)
        print("Published image to MQTT topic")

    def start(self): 
        rospy.spin() 

device_ip = "your ip"
moorebot_ip = "moorebot ip"
if __name__ == '__main__': 
    client = MqttClient(device_ip, 1883, moorebot_ip, 'placeholder')
    client.start() 
