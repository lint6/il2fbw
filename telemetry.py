import socket
import struct

UDP_IP = "127.0.0.1"
UDP_PORT = 20888  

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"Listening for IL-2 telemetry on {UDP_IP}:{UDP_PORT}\n")

def reciveData():
    global sock
    data, addr = sock.recvfrom(4096)  # receive UDP packet
    if len(data) == 44:  # motion packet
        values = struct.unpack('<11f', data)  # 11 floats
        return values
