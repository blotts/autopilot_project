# Telemetry streaming utils for the controlllers
import socket
import logging

class TelemetryManager:
    def __init__(self, dest_addr: str, dest_port: int):
        self.dest_addr = dest_addr
        self.dest_port = dest_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.connect((self.dest_addr, self.dest_port))
        except Exception as e:
            raise ConnectionError(f"Failed to connect to telemetry destination {self.dest_addr}:{self.dest_port}:\n{e}")

    def __del__(self):
        try:
            self.sock.close()
        except Exception as e:
            logging.warning(f"Failed to close telemetry socket:\n{e}")    

    # maybe use flatbuffers or some actual schema. idk if
    # xplane plugins actually support that
    async def send_data(self, data: str):
        try:
            self.sock.sendall(data.encode('utf-8'))
        except:
            logging.warning(f"Could not send telemetry data")
