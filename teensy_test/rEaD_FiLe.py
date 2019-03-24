"""

Basic reading and printing file for xbee ground stattion. Will read data from Teensy

Need to work on timing between sending and receiving

"""

from digi.xbee.devices import XBeeDevice

import array
import struct

PORT = "COM9"
# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 9600

xbee = XBeeDevice(PORT, BAUD_RATE)

xbee.open()

while True:
    try:
       # print(xbee.read_data_from(0x13A200409F5BA2,5)) #specific xbee on teensy 64bit addr
       xbee_message = xbee.read_data(10) #Timeout value
       data_output = xbee_message.data
       #print(data_output)
       fmt = "<%dd" % (len(data_output) // 8)
       #print(fmt)
       print(list(struct.unpack(fmt, data_output)))
       #print(array.array('d', data_output))
       #print(data_output.decode("utf-8"))
       
    except KeyboardInterrupt:
        break
    

xbee.close()
