import socket, json

BROADCAST_ADDR = "192.168.129.255"    
NUCLEO_PORT    = 12345
PC_IP          = "192.168.129.71"    

data_request = {
    "type": "presence",
    "id": "pc-client",
    "ip": PC_IP,
    "timestamp": "2025-01-01T00:00:00Z"
}

message = json.dumps(data_request).encode("utf-8")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

sock.bind((PC_IP, 0))   

sock.sendto(message, (BROADCAST_ADDR, NUCLEO_PORT))
print("Sent:", message.decode())
sock.close()

