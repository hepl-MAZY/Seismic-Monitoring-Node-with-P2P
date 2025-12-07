import socket
import json

NUCLEO_IP = "192.168.1.222"   
NUCLEO_PORT = 12345       

data_request = {
    "type": "presence",
    "id": "pc-client",
    "ip": "192.168.1.223",   
    "timestamp": "2025-01-01T00:00:00Z"
}

message = json.dumps(data_request).encode("utf-8")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(message, (NUCLEO_IP, NUCLEO_PORT))
print("Sent:", message.decode())
sock.close()