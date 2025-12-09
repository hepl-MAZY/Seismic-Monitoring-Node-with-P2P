# tcpServerDataResponse.py
import socket
import json
import time
import random

SERVER_IP   = "192.168.1.227"       
SERVER_PORT = 12345

NODE_ID = "pc-client"    


def build_data_response():
    """
    Build a JSON data_response message.
    Here I just generate some dummy acceleration values.
    Replace with whatever you want.
    """
    # Example: random small noise around 0 on X/Y, ~1g on Z
    ax = round(random.uniform(-0.05, 0.05), 4)
    ay = round(random.uniform(-0.05, 0.05), 4)
    az = round(random.uniform(0.95, 1.05), 4)

    timestamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    msg = {
        "type": "data_response",
        "id": NODE_ID,
        "timestamp": timestamp,
        "acceleration": {
            "x": ax,
            "y": ay,
            "z": az
        },
        "status": "normal"
    }
    return msg


def handle_client(conn, addr):
    print(f"[+] New TCP connection from {addr}")

    conn.settimeout(3.0)

    try:
        # 1) Receive JSON request from Nucleo
        data = conn.recv(1024)
        if not data:
            print("[-] No data received, closing connection.")
            return

        try:
            txt = data.decode("utf-8", errors="replace")
            print("[>] Raw request:", txt)
            req = json.loads(txt)
            print("[>] Parsed JSON request:", req)
        except json.JSONDecodeError:
            print("[-] Request is not valid JSON, closing.")
            return

        # 2) Check type is data_request
        if req.get("type") != "data_request":
            print("[-] JSON type is not 'data_request', ignoring.")
            return

        # 3) Build and send data_response
        resp = build_data_response()
        resp_bytes = json.dumps(resp).encode("utf-8")

        conn.sendall(resp_bytes)
        print("[<] Sent data_response:", resp)

    except socket.timeout:
        print("[-] Timeout while waiting/reading client.")
    finally:
        conn.close()
        print("[*] Connection closed.")


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((SERVER_IP, SERVER_PORT))
        server.listen(5)

        print(f"[LISTEN] TCP server on {SERVER_IP or '0.0.0.0'}:{SERVER_PORT}")

        while True:
            conn, addr = server.accept()
            handle_client(conn, addr)


if __name__ == "__main__":
    main()
