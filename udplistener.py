import socket

# Configuration
UDP_IP = "0.0.0.0"   # Listen on all interfaces
UDP_PORT = 5005      # Change to match your sender's port

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP messages on {UDP_IP}:{UDP_PORT}...")

try:
    while True:
        data, addr = sock.recvfrom(1024)  # Buffer size = 1024 bytes
        print(f"Received from {addr}: {data.decode(errors='replace')}")
except KeyboardInterrupt:
    print("\nStopped listening.")
finally:
    sock.close()
