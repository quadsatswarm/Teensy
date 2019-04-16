import serial
from digi.xbee.devices import XBeeDevice

BAUD_RATE = 9600  # assumption of BAUD_RATE is same for both teensy and xbee (9600)
TEENSY_PORT = "/dev/ttyACM1"  # TODO: this will need to change to actual teensy port
XBEE_PORT = "/dev/ttyUSB1"  # TODO: This will need to change most likely depending on what XBee is recognized as
GROUND_STATION = "GROUNDSTATION"  # TODO: find actual ground station node id

# Serial Port for Xbee
s1 = serial.Serial(TEENSY_PORT, BAUD_RATE)
s1.flushInput()

xbee = XBeeDevice(XBEE_PORT, BAUD_RATE)
xbee.open()

# Find ground station
xbee_network = xbee.get_network()
ground_station_xbee = xbee_network.discover_device(GROUND_STATION)
if ground_station_xbee is None:
    print("Could not find the ground station")
    exit(1)


# Define a callback function for receives
def update_agent_loc(xbee_message):
    """
    Simple callback function to update current swarm signal iteration
    :param xbee_message:
    :return:
    """
    swarm_signal = xbee_message.data.decode() # additional data processing can occur on this line


xbee.add_data_received_callback(update_agent_loc)

coordinates = s1.readline()  # DATA received from teensy
coordinates = "0 1 2 3"  # dummy
swarm_signal = "0 0 0"
print(coordinates)

while True:
    xbee.send_data_async(ground_station_xbee, coordinates)
    s1.write(swarm_signal.encode())
