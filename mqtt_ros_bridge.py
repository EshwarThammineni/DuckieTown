import rclpy 
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu, Illuminance, Range
from nav_msgs.msg import Odometry

import paho.mqtt.client as mqtt
import enum 

import numpy as np
import cv2
from cv_bridge import CvBridge

class NodeState(enum.Enum):
  ALIVE = 1
  AWAIT = 2
  PENDING = 3
  OFFLINE = 4

class MqttRosBridge(Node): 

  def __init__(self, mqtt_broker_ip, mqtt_broker_port):
    super().__init__('mqtt_ros_bridge') 
    self.mqtt_client = mqtt.Client()
    self.mqtt_client.connect(mqtt_broker_ip, mqtt_broker_port)
    self.mqtt_client.on_connect = self.on_connect
    self.mqtt_client.on_message = self.on_message
    # establish a periodic heartbeat publisher to the 'lobby' channel 
    hb_timer_period = 1 # emit every second 
    self._hb_timer = self.create_timer(hb_timer_period, self.hb_timer_callback) 
    self._detected_nodes = dict() # node name -> state
    self._sub_dict = dict() # node name -> subscriber object
    self._pub_dict = dict() # node name -> publisher objects{}
    self._cv_bridge = CvBridge()
    self.mqtt_client.loop_start()

  def on_connect(self, client, userdata, flags, reason_code, properties=None):
    self.get_logger().info("Connected to MQTT broker with result code "+str(reason_code))
    self.mqtt_client.subscribe("lobby")
    self.mqtt_client.loop_start()

  def on_message(self, client, userdata, message):
    topic, msg = message.topic, message.payload
    #print("Message received on topic ", topic)
    if 'lobby' in topic:
      self.lobby_handler(message)
    if 'sensor/imu' in topic: 
      node_name = topic.split('/')[0]
      if node_name in self._pub_dict:
        imu_msg = Imu()
        angular_vals = msg.decode('utf-8').split('\n')[0].split(' ')
        linear_vals = msg.decode('utf-8').split('\n')[1].split(' ')
        if len(angular_vals) != 3 or len(linear_vals) != 3:
          self.get_logger().error("Invalid IMU data format.") 
          return
        imu_msg.angular_velocity.x = float(angular_vals[0])
        imu_msg.angular_velocity.y = float(angular_vals[1])
        imu_msg.angular_velocity.z = float(angular_vals[2])
        imu_msg.linear_acceleration.x = float(linear_vals[0])
        imu_msg.linear_acceleration.y = float(linear_vals[1])
        imu_msg.linear_acceleration.z = float(linear_vals[2])
        self._pub_dict[node_name]["sensor/imu"].publish(imu_msg)

    if 'camera/image' in topic:

      node_name = topic.split('/')[0]
      
      if node_name in self._pub_dict:

        np_arr = np.frombuffer(message.payload, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
        
        if cv_image is None:
          self.get_logger().error("Failed to decode image.") 
          return
        
        image_msg = self._cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub_dict[node_name]["camera/image"].publish(image_msg)
    
    if 'baselink/odom' in topic:
      node_name = topic.split('/')[0]
      if node_name in self._pub_dict:
        odom_msg = Odometry()
        # Here you would parse message.payload to fill odom_msg appropriately
        self._pub_dict[node_name]["baselink/odom"].publish(odom_msg)
    if 'vio/odom' in topic:
      node_name = topic.split('/')[0]
      if node_name in self._pub_dict:
        odom_msg = Odometry()
        # Here you would parse message.payload to fill odom_msg appropriately
        self._pub_dict[node_name]["vio/odom"].publish(odom_msg)

    if 'sensor/battery' in topic:
      node_name = topic.split('/')[0]
      if node_name in self._pub_dict:
        battery_msg = String()
        # Here you would parse message.payload to fill battery_msg appropriately
        self._pub_dict[node_name]["battery"].publish(battery_msg)

    if 'sensor/light' in topic:
      node_name = topic.split('/')[0]
      if node_name in self._pub_dict:
        light_msg = Illuminance()
        light_msg.illuminance = float(msg.decode('utf-8'))
        self._pub_dict[node_name]["sensor/light"].publish(light_msg)

    if 'sensor/tof' in topic:
      node_name = topic.split('/')[0]
      if node_name in self._pub_dict:
        tof_msg = Range()
        tof_msg.range = float(msg.decode('utf-8'))
        self._pub_dict[node_name]["sensor/tof"].publish(tof_msg)

    #print(repr(message.payload))

  def lobby_handler(self, message):
    msg = message.payload.decode('utf-8')
    if "::alive" in msg: 
      self._detected_nodes[msg.split("::")[0]] = NodeState.ALIVE  # get the name of the node sending the alive signal
      if msg.split("::")[0] not in self._sub_dict: 
        node_name = msg.split("::")[0]
        self.register_moorebot_node(node_name)

  def register_moorebot_node(self, node_name):
    # create subscriber for this node's positions 
    self._pub_dict[node_name] = dict()
    self._sub_dict[node_name] = dict()

    self._sub_dict[node_name]["cmd_vel"] = self.create_subscription(
      Twist,
      f"{node_name}/cmd_vel",
      lambda msg, nn=node_name: self.cmd_vel_callback(msg, nn),
      1
    )
    
    self._pub_dict[node_name]["camera/image"] = self.create_publisher(
      Image, f"{node_name}/camera/image", 1)
    
    self._pub_dict[node_name]["baselink/odom"] = self.create_publisher(
      Odometry, f"{node_name}/baselink/odom", 1)
    
    self._pub_dict[node_name]["vio/odom"] = self.create_publisher(
      Odometry, f"{node_name}/vio/odom", 1)
    
    #self._pub_dict[node_name]["battery"] = self.create_publisher(
    #  String, f"{node_name}/battery", 1)

    self._pub_dict[node_name]["sensor/imu"] = self.create_publisher(
      Imu, f"{node_name}/sensor/imu", 1)
    
    self._pub_dict[node_name]["sensor/light"] = self.create_publisher(
      Illuminance, f"{node_name}/sensor/light", 1)
    
    self._pub_dict[node_name]["sensor/tof"] = self.create_publisher(
      Range, f"{node_name}/sensor/tof", 1)
    # set mqtt_client to listen to this node's topics
    self.mqtt_client.subscribe(f"{node_name}/camera/image")
    self.mqtt_client.subscribe(f"{node_name}/baselink/odom")
    self.mqtt_client.subscribe(f"{node_name}/vio/odom")
    self.mqtt_client.subscribe(f"{node_name}/sensor/battery")
    self.mqtt_client.subscribe(f"{node_name}/sensor/imu")
    self.mqtt_client.subscribe(f"{node_name}/sensor/light")
    self.mqtt_client.subscribe(f"{node_name}/sensor/tof")


  def hb_timer_callback(self):
    self.mqtt_client.publish("lobby", "check-alive")
    # print("emit heartbeat")
    # adjust states
    for node in self._detected_nodes:
      if self._detected_nodes[node] == NodeState.ALIVE:
        self._detected_nodes[node] = NodeState.AWAIT
      elif self._detected_nodes[node] == NodeState.AWAIT:
        self._detected_nodes[node] = NodeState.PENDING
      elif self._detected_nodes[node] == NodeState.PENDING:
        self._detected_nodes[node] = NodeState.OFFLINE
        self.get_logger().info(f"Node {node} is now OFFLINE.")
        # can incorporate cleanup logic here -- 
        # node crashes when 'dictionary changed size during iteration'

  def cmd_vel_callback(self, msg, node_name):
    # log 
    self.get_logger().info(f"Cmd_vel received for {node_name}: linear_x={msg.linear.x}, linear_y={msg.linear.y}, angular_z={msg.angular.z}")
    data_string = str(msg.linear.x) + " "
    data_string += str(msg.linear.y) + " "
    data_string += str(msg.angular.z)
    self.mqtt_client.publish(f"{node_name}/cmd_vel", data_string) 

def main(args=None): 
  rclpy.init(args=args) 
  
  mqtt_ros_bridge = MqttRosBridge('localhost', 1883)

  rclpy.spin(mqtt_ros_bridge) 

  mqtt_ros_bridge.destroy_node() 
  rclpy.shutdown()
