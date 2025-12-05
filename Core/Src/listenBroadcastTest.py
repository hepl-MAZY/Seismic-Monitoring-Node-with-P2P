import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(('', 12345))

print("Listening on port 12345...")
while True:
    data, addr = sock.recvfrom(1024)
    print("From:", addr, "->", data.decode())
