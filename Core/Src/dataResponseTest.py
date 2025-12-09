import socket
import json

NUCLEO_IP = "192.168.1.226"   
NUCLEO_PORT = 12345       

data_request = {
    "type": "data_response",
    "id": "pc-client",
    "timestamp": "2025-01-01T00:00:00Z",
    "acceleration": {
        "x": 10.1,
        "y": 10.2,
        "z": 10.3
    }
}

message = json.dumps(data_request).encode("utf-8")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(message, (NUCLEO_IP, NUCLEO_PORT))
print("Sent:", message.decode())
sock.close()