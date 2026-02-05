#!/usr/bin/env python3
"""
MAVLink bridge - forwards messages from PX4 UDP to TCP for external access.
Run this inside the Docker container.
"""
import socket
import threading
import time

# PX4 connection (inside container)
PX4_HOST = "127.0.0.1"
PX4_PORT = 14580  # PX4 offboard local port

# External TCP server
TCP_PORT = 5761

def udp_to_tcp():
    """Bridge UDP from PX4 to TCP clients."""
    # UDP socket to talk to PX4
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.settimeout(1.0)
    
    # TCP server for external clients
    tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_server.bind(("0.0.0.0", TCP_PORT))
    tcp_server.listen(5)
    tcp_server.settimeout(1.0)
    
    print(f"MAVLink bridge started")
    print(f"  PX4: udp://{PX4_HOST}:{PX4_PORT}")
    print(f"  TCP: tcp://0.0.0.0:{TCP_PORT}")
    
    clients = []
    
    # Send initial packet to PX4 to start receiving
    udp_sock.sendto(b"\xfe\x09\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x06\x08\x00\x00\x00\x00\xa6\xd0", 
                    (PX4_HOST, PX4_PORT))
    
    def accept_clients():
        while True:
            try:
                client, addr = tcp_server.accept()
                print(f"TCP client connected: {addr}")
                clients.append(client)
            except socket.timeout:
                pass
            except Exception as e:
                print(f"Accept error: {e}")
    
    # Start client acceptor thread
    threading.Thread(target=accept_clients, daemon=True).start()
    
    while True:
        try:
            # Receive from PX4
            data, addr = udp_sock.recvfrom(4096)
            
            # Forward to all TCP clients
            for client in clients[:]:
                try:
                    client.sendall(data)
                except:
                    clients.remove(client)
            
            # Also receive from TCP clients and forward to PX4
            for client in clients[:]:
                try:
                    client.setblocking(False)
                    tcp_data = client.recv(4096)
                    if tcp_data:
                        udp_sock.sendto(tcp_data, (PX4_HOST, PX4_PORT))
                except BlockingIOError:
                    pass
                except:
                    clients.remove(client)
                    
        except socket.timeout:
            # Send keepalive heartbeat
            udp_sock.sendto(b"\xfe\x09\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x06\x08\x00\x00\x00\x00\xa6\xd0",
                           (PX4_HOST, PX4_PORT))
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.1)

if __name__ == "__main__":
    udp_to_tcp()
