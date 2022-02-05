import socket
import struct
import ctypes
import zlib
import binascii
import time
# RoboRIO ip address: 10.TE.AM.2: '10.63.57.2'
udp_ip = '10.63.57.2'

# Available bi-directionl UDP/TCP ports on RoboRIO: 5800-5810
udp_port = 5800

# need to find open ports on odroid
# not sure if this port is open for UDP use on the Odroid
local_port = 49153

# binds to local port meaning that sock_s.sendto()
# will always send from RPi UDP @ local_port
sock_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_s.bind(('0.0.0.0', local_port))

# datagram format
# Little Endian
# 2 bytes Version ID | Length
# 4 Bytes Frame ID
# 4 Bytes Timestamp
# 2 Bytes Distance in cm, divide int by 100 for decimal value 
# 2 Bytes Horizontal Angle in degrees, divide int by 100 for decimal value
# 2 Bytes Vertical Angle in degrees, divide int by 100 for decimal value
# 8 Bytes Reserved
# 2 Bytes Checksum
struct_format = '<BBIIHhhQ'

frameID = 0
def sendPacket(distance, horiAngle, vertAngle, frameID):
    versionID = 1
    packetLength = 28
    timestamp = 0
    
    # create a buffer that is 28 bytes long based off the above format
    bufChecksum = bytearray(28)
    
    # pack into buffer the values represented as bytes
    struct.pack_into(struct_format, bufChecksum, 0, versionID, packetLength, frameID, timestamp, distance, horiAngle, vertAngle, 0)

    # creates datagram in the above format
    # Test Value:, 
    # Version ID: 1, Length: 26, Frame ID: 1, Timestamp: 00:00:00, 
    # Distance: 509.524 cm (200.6 in), Horizontal Angle: 20.0 deg,
    # Vertical Angle, 63.0 deg, 0 out unsigned long long, Checksum: 1 initially

    # Calculate Checksum
    checksum = zlib.crc32(bufChecksum[0:24])
    #print("checksum: " + "{:x}".format(checksum))

    struct.pack_into("L", bufChecksum, 24, checksum)
    
    #print("post checksum: " + str(binascii.hexlify(bytes(bufChecksum))))

    
    #sends bytes in datagram to device with udp_ip at udp_port
    sock_s.sendto(bytes(bufChecksum), (udp_ip, udp_port))
        
    
    #data, addr = sock_s.recvfrom(1024)
    # retrives data from the port sock_s is bound to -> local_port
    
    #print("received message: %s" % data.decode('utf-8'))
    # before printing message the bytes must be decoded to string format so it can be printed
    # equaivalent to Java .toString() from bytes
