import socket
import json

NUCLEO_IP = "192.168.1.226"
NUCLEO_PORT = 12345

def main():
    data_request = {
        "type": "data_request",
        "from": "pc-client",
        "to": "nucleo-Kate",
        "timestamp": "2025-01-01T00:00:00Z"
    }

    message = json.dumps(data_request).encode("utf-8")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(3.0)  # seconds

        # 1) Connect to Nucleo (TCP handshake happens here)
        print(f"Connecting to {NUCLEO_IP}:{NUCLEO_PORT} ...")
        sock.connect((NUCLEO_IP, NUCLEO_PORT))
        print("Connected.")

        # 2) Send JSON request
        sock.sendall(message)
        print("Sent:", message.decode())

        # 3) Receive response (until server closes the connection)
        chunks = []
        try:
            while True:
                chunk = sock.recv(1024)
                if not chunk:
                    break  # server closed connection
                chunks.append(chunk)
        except socket.timeout:
            print("Timeout waiting for response")

        if chunks:
            resp_bytes = b"".join(chunks)
            print("Raw response:", resp_bytes.decode("utf-8", errors="replace"))

            try:
                resp_json = json.loads(resp_bytes.decode("utf-8"))
                print("Parsed JSON response:", resp_json)
            except json.JSONDecodeError:
                print("Could not parse response as JSON.")

if __name__ == "__main__":
    main()
