import paho.mqtt.client as mqtt

broker_ip = "xxx.xxx.x.xxx"   # your ip
broker_port = 1883  # standard port for unencrypted mqtt communication but secure connections with TLS or SSL are 8883. 

client = mqtt.Client()
client.connect(broker_ip, broker_port)

# velocity values
lx = 0.5  # linear x
ly = 0.5  # linear y
az = 1.2  # angular z

# newline format required by robot code
payload = f"DuckieDonald::{lx}\n{ly}\n{az}"

client.publish("velocities", payload)

print("Command sent!")
client.disconnect()
