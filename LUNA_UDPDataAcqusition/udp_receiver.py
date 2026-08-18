import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("========================================")
print("       ESP32 UDP SENSOR RECEIVER")
print("========================================")
print(f"Listening on UDP port {UDP_PORT}...")
print("Waiting for ESP32 data...\n")

while True:
    data, address = sock.recvfrom(4096)

    message = data.decode("utf-8")

    print(message)