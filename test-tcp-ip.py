import socket
from gurux_dlms import GXDLMSClient, GXReplyData
from gurux_dlms.enums import InterfaceType, Authentication
from gurux_dlms.objects import GXDLMSRegister
from gurux_common import GXCommon

# -------------------------------
# TCP connection helper
# -------------------------------
class DLMSConnectionTCP:
    def __init__(self, host, port=4059, timeout=5):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect((host, port))

    def read_dlms_packet(self, data, reply):
        # Send request
        self.sock.sendall(data)

        # Read until a complete DLMS packet is received
        while True:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise Exception("No response from meter")

            reply.Data += chunk

            if GXCommon.isCompleteDLMSPacket(reply.Data):
                break

        return reply

    def close(self):
        self.sock.close()


# -------------------------------
# Create DLMS client
# -------------------------------
client = GXDLMSClient(
    True,                 # Use Logical Name referencing
    InterfaceType.WRAPPER,  # TCP/IP uses wrapper
    16,                   # Client address
    1                     # Server address (meter)
)

# LOW level authentication
client.authentication = Authentication.LOW
client.password = b"00000000"  # Password must be bytes

# -------------------------------
# Open TCP connection
# -------------------------------
conn = DLMSConnectionTCP("192.168.1.100", 4059)  # replace with meter IP
reply = GXReplyData()

# -------------------------------
# AARQ request (application association)
# -------------------------------
for aarq in client.aarqRequest():
    reply.clear()
    conn.read_dlms_packet(aarq, reply)

client.parseAAREResponse(reply.Data)
print("Connected using LOW authentication over TCP/IP")

# -------------------------------
# Read an example object
# OBIS: 1.0.1.8.0.255 (Active energy import)
# -------------------------------
energy = GXDLMSRegister("1.0.1.8.0.255")

request = client.read(energy, 2)  # Attribute 2 = value
reply.clear()
conn.read_dlms_packet(request, reply)

client.updateValue(energy, 2, reply.Data)
print("Energy value:", energy.value)

# -------------------------------
# Release association
# -------------------------------
for rel in client.releaseRequest():
    reply.clear()
    conn.read_dlms_packet(rel, reply)

conn.close()
print("Disconnected cleanly")
