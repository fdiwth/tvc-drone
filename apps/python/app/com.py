import asyncio
import random
import serial.tools.list_ports
import serial_asyncio
import time

STATUS_TSDN = 0b10000000
STATUS_TSDS = 0b11000000
STATUS_TCDS = 0b11100000
STATUS_TCDC = 0b11110000
STATUS_NULL = 0b00000000

DELAY = 2
MAX_LATENCY = 1


class SerialProtocol(asyncio.Protocol):
    def __init__(self, app):
        super().__init__()
        self.transport = None
        self.com_status = None
        self.com_signature = None
        self.handshake_task = None
        self.timelog = None
        self.app = app
        self.rx_buffer = bytearray()
        self.handshake_complete = False
        random.seed()

    def connection_made(self, transport):
        self.transport = transport
        
        # Give the app a reference to this protocol instance so it can send commands
        if hasattr(self.app, "set_protocol"):
            self.app.set_protocol(self)
            
        print("requesting connection...")
        self.handshake_task = asyncio.create_task(self.syn_req())

    async def syn_req(self):
        try:
            while self.com_status != STATUS_TCDC:
                self.com_signature = random.randint(0, 15)
                self.com_status = STATUS_TSDN
                header = self.com_status | self.com_signature
                self.transport.write(bytes([header]))
                self.timelog = time.time()
                print(f"req: {header:08b}")
                await asyncio.sleep(DELAY)
        except asyncio.CancelledError:
            pass

    def data_received(self, data):
        if self.handshake_complete:
            self.listen_stream(data)
            return

        cmd = data[0]
        self.com_status = cmd & 0xF0
        recieved_signature = cmd & 0x0F
        print(f"received: {cmd:08b}")

        if self.com_status == STATUS_TSDS and recieved_signature == self.com_signature:
            if (time.time() - self.timelog) < MAX_LATENCY:
                self.com_status = STATUS_TCDS
            else:
                self.com_status = STATUS_NULL
            header = self.com_status | self.com_signature
            self.transport.write(bytes([header]))
            print(f"confirm: {header:08b}")

        if self.com_status == STATUS_TCDC and recieved_signature == self.com_signature:
            print("communication handshake completed")
            if self.handshake_task:
                self.handshake_task.cancel()

            self.handshake_complete = True
            print("waiting for initialization...")

            if len(data) > 1:
                self.listen_stream(data[1:])

    def listen_stream(self, data):
        self.rx_buffer.extend(data)

        while True:
            if len(self.rx_buffer) < 3:
                break

            enc = self.rx_buffer[1]
            payload_len = self.rx_buffer[2]
            total_packet_len = 3 + payload_len

            if len(self.rx_buffer) < total_packet_len:
                break

            packet = self.rx_buffer[:total_packet_len]
            del self.rx_buffer[:total_packet_len]

            try:
                payload = packet[3:]

                if enc == 0b00000000:
                    utf8_stream = payload.decode("utf-8", errors="ignore").strip()
                    print(f"received: {utf8_stream}")
                else:
                    self.app.receive_callback(enc, payload)
            except Exception as e:
                print(f"Error parsing packet: {e}")

    def connection_lost(self, exc):
        print("communication port closed")


def list_ports():
    ports = serial.tools.list_ports.comports()
    print("Serial ports found: ")
    for port in ports:
        print(port.device)
