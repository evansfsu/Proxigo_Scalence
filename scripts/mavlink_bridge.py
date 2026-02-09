#!/usr/bin/env python3
"""
MAVLink bridge - forwards messages from PX4 UDP to TCP for external access.
Run this inside the Docker container.
"""
import socket
import threading
import time
import sys

# Try to use pymavlink for proper MAVLink communication
try:
    from pymavlink import mavutil
    HAVE_PYMAVLINK = True
except ImportError:
    HAVE_PYMAVLINK = False
    print("WARNING: pymavlink not available, using raw socket forwarding", flush=True)

# PX4 connection (from bridge container to px4-sitl container)
# Use service name from docker-compose (with hyphen, not underscore)
PX4_HOST = "px4-sitl"  # Service name (Docker Compose DNS)
PX4_PORT = 18570  # PX4 GCS port (listening)

# External ports
UDP_PORT = 14550  # QGC standard port
TCP_PORT = 5760   # QGC TCP port

def udp_to_tcp():
    """Bridge UDP from PX4 to UDP/TCP for QGC."""
    # UDP socket to communicate with PX4
    # PX4 listens on 18570, so we send TO 18570 and receive FROM 18570
    px4_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    px4_udp.settimeout(1.0)
    # Don't use connect() - we need to use sendto/recvfrom for proper UDP communication
    
    # UDP server for QGC (14550)
    try:
        qgc_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        qgc_udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        qgc_udp.bind(("0.0.0.0", UDP_PORT))
        print(f"UDP server bound to port {UDP_PORT}")
    except Exception as e:
        print(f"ERROR: Failed to bind UDP port {UDP_PORT}: {e}")
        return
    
    # TCP server for QGC (5760)
    try:
        tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_server.bind(("0.0.0.0", TCP_PORT))
        tcp_server.listen(5)
        tcp_server.settimeout(1.0)
        print(f"TCP server bound to port {TCP_PORT}")
    except Exception as e:
        print(f"ERROR: Failed to bind TCP port {TCP_PORT}: {e}")
        return
    
    print(f"MAVLink bridge started", flush=True)
    print(f"  PX4 GCS: udp://{PX4_HOST}:{PX4_PORT}", flush=True)
    print(f"  QGC UDP: udp://0.0.0.0:{UDP_PORT}", flush=True)
    print(f"  QGC TCP: tcp://0.0.0.0:{TCP_PORT}", flush=True)
    
    # Test PX4 connectivity
    try:
        import socket as test_socket
        test_sock = test_socket.socket(test_socket.AF_INET, test_socket.SOCK_DGRAM)
        test_sock.sendto(b"test", (PX4_HOST, PX4_PORT))
        print(f"  Test packet sent to {PX4_HOST}:{PX4_PORT}", flush=True)
    except Exception as e:
        print(f"  WARNING: Cannot reach {PX4_HOST}:{PX4_PORT}: {e}", flush=True)
    
    clients = []
    qgc_clients = {}  # Track QGC UDP clients
    
    # Note: PX4 will send heartbeats automatically, we just need to forward them
    print(f"  Ready to forward MAVLink messages", flush=True)
    
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
    
    # Thread to receive from QGC UDP and forward to PX4
    def handle_qgc_udp():
        while True:
            try:
                data, addr = qgc_udp.recvfrom(4096)
                if len(data) > 0:
                    qgc_clients[addr] = True
                    print(f"  [QGC->PX4] Received {len(data)} bytes from {addr}, forwarding to PX4", flush=True)
                    # Forward to PX4
                    px4_udp.sendto(data, (PX4_HOST, PX4_PORT))
            except Exception as e:
                pass
    
    threading.Thread(target=handle_qgc_udp, daemon=True).start()
    
    # Thread to receive from PX4 and forward to QGC
    def receive_from_px4():
        while True:
            try:
                data, addr = px4_udp.recvfrom(4096)
                if len(data) > 0:
                    print(f"  [PX4->QGC] Received {len(data)} bytes from PX4, forwarding to {len(qgc_clients)} UDP clients and {len(clients)} TCP clients", flush=True)
                    # Forward to QGC UDP clients
                    for client_addr in list(qgc_clients.keys()):
                        try:
                            qgc_udp.sendto(data, client_addr)
                        except:
                            del qgc_clients[client_addr]
                    # Forward to TCP clients
                    for client in clients[:]:
                        try:
                            client.sendall(data)
                        except:
                            clients.remove(client)
            except socket.timeout:
                pass
            except Exception as e:
                pass
    
    threading.Thread(target=receive_from_px4, daemon=True).start()
    
    # Main loop - handle QGC -> PX4 forwarding
    while True:
        try:
            time.sleep(0.1)
            
            # Receive from TCP clients and forward to PX4
            for client in clients[:]:
                try:
                    client.settimeout(0.1)
                    tcp_data = client.recv(4096)
                    if tcp_data and len(tcp_data) > 0:
                        px4_udp.sendto(tcp_data, (PX4_HOST, PX4_PORT))
                except (socket.timeout, BlockingIOError):
                    pass
                except:
                    clients.remove(client)
                    
        except Exception as e:
            print(f"Error in main loop: {e}", flush=True)
            time.sleep(0.1)

if __name__ == "__main__":
    udp_to_tcp()
