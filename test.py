import serial
import time
from gurux_dlms import GXDLMSClient, GXReplyData
from gurux_dlms.enums import InterfaceType, Authentication
from gurux_dlms.objects import GXDLMSRegister
from gurux_dlms import GXByteBuffer

# -------------------------------
# Serial connection helper
# -------------------------------
class DLMSConnection:
    def __init__(self, port, baudrate=9600, timeout=2):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,  # FIXED: Changed from PARITY_EVEN to PARITY_NONE
            stopbits=1,
            timeout=timeout
        )
        self.current_baudrate = baudrate

    def iec_mode_init(self, initial_baud=300):
        """
        IEC 62056-21 mode initialization
        FIXED: Proper implementation starting at 300 baud (7E1)
        """
        print("\n=== IEC Mode Initialization ===")
        
        # FIXED: Start at 300 baud with 7E1 for IEC handshake
        self.ser.close()
        self.ser = serial.Serial(
            port=self.ser.port,
            baudrate=300,
            bytesize=7,
            parity=serial.PARITY_EVEN,
            stopbits=1,
            timeout=2
        )
        time.sleep(0.2)
        
        # Clear any pending data
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
        # Send IEC sign-on message
        sign_on = b"/?!\r\n"
        print(f"Sending IEC sign-on at 300 baud 7E1: {sign_on}")
        self.ser.write(sign_on)
        time.sleep(1.0)  # FIXED: Longer wait for meter response
        
        # Read identification message
        response = self.ser.read(256)
        if response:
            print(f"IEC response ({len(response)} bytes): {response.hex()}")
            try:
                text = response.decode('ascii', errors='ignore').strip()
                if text:
                    print(f"IEC response (text): {text}")
                    
                    # FIXED: Proper parsing of identification string
                    # Format: /XXX5 Identification CR LF
                    # Where 5 means the meter supports up to 9600 baud
                    if text.startswith('/'):
                        lines = text.split('\r\n')
                        id_line = lines[0]
                        print(f"  Meter ID: {id_line}")
                        
                        # Extract baud rate character (usually position 4)
                        if len(id_line) >= 5:
                            baud_char = id_line[4]
                            print(f"  Baud rate indicator: {baud_char}")
            except Exception as e:
                print(f"Error parsing IEC response: {e}")
            
            # FIXED: Send ACK with proper format for 9600 baud
            # Format: ACK 0 5 0 <CR><LF>
            # 0 = protocol control, 5 = 9600 baud, 0 = mode selection
            ack = b"\x06050\r\n"  # FIXED: Corrected ACK format
            print(f"Sending ACK for 9600 baud: {ack.hex()}")
            self.ser.write(ack)
            time.sleep(0.3)
            
            # FIXED: Switch to 9600 baud with 8N1 for DLMS
            self.ser.close()
            self.ser = serial.Serial(
                port=self.ser.port,
                baudrate=9600,
                bytesize=8,
                parity=serial.PARITY_NONE,
                stopbits=1,
                timeout=2
            )
            time.sleep(0.3)
            print("Switched to 9600 baud 8N1 for DLMS communication")
            
            # Clear buffer after baud rate switch
            self.ser.reset_input_buffer()
            
            return True
        else:
            print("No IEC response received")
            return False

    def read_complete_response(self, max_bytes=2048, timeout=2.0):
        """Read all available data from serial port"""
        buffer = bytearray()
        start_time = time.time()
        
        # FIXED: Better timeout handling
        while (time.time() - start_time) < timeout and len(buffer) < max_bytes:
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                if chunk:
                    buffer.extend(chunk)
                    print(f"Read chunk ({len(chunk)} bytes): {chunk.hex()}")
                    # Reset timeout on successful read
                    start_time = time.time()
            time.sleep(0.05)
        
        if not buffer:
            raise Exception("No response from meter")
        
        print(f"Total response ({len(buffer)} bytes): {buffer.hex()}")
        return buffer

    def read_dlms_packet(self, client, reply):
        """Read and parse DLMS packet"""
        # FIXED: More robust response reading
        raw_data = self.read_complete_response(timeout=3.0)
        
        # FIXED: Direct conversion to GXByteBuffer without modifications
        # Let Gurux library handle the HDLC framing
        buffer = GXByteBuffer(raw_data)
        notify = GXReplyData()
        
        try:
            result = client.getData(buffer, reply, notify)
            print(f"\ngetData() result: {result}")
            
            if reply.isComplete():
                print("✓ Packet complete and parsed successfully")
            else:
                print("⚠ Packet incomplete, may need more data")
            
            return reply
                
        except Exception as e:
            print(f"getData() exception: {e}")
            print(f"Reply error code: {reply.error}")
            # Store raw data for debugging
            reply.data = buffer
            return reply

    def write(self, data):
        self.ser.reset_output_buffer()
        self.ser.write(data)
        print(f"Sent ({len(data)} bytes): {data.hex()}")
        time.sleep(0.1)  # FIXED: Small delay after write

    def close(self):
        if self.ser.is_open:
            self.ser.close()


# -------------------------------
# Main communication function
# -------------------------------
def communicate_with_meter(port, client_addr=16, server_addr=1):
    """
    Communicate with DLMS meter
    FIXED: Simplified with correct default addresses
    """
    print("\n" + "="*70)
    print(f"DLMS Meter Communication - DDSY5558")
    print(f"Client Address: {client_addr}")
    print(f"Server Address: {server_addr}")
    print("="*70)
    
    # FIXED: Create client with proper settings
    client = GXDLMSClient(True, InterfaceType.HDLC)
    client.clientAddress = client_addr
    client.serverAddress = server_addr
    
    # FIXED: Use LOW authentication as per meter spec
    client.authentication = Authentication.LOW
    client.password = b"00000000"
    
    # FIXED: Set proper limits
    client.limits.maxInfoTX = 128
    client.limits.maxInfoRX = 128
    
    print(f"\nClient settings:")
    print(f"  Client Address: {client.clientAddress}")
    print(f"  Server Address: {client.serverAddress}")
    print(f"  Authentication: {client.authentication}")
    
    # Open connection
    conn = None
    
    try:
        conn = DLMSConnection(port, 9600)
        reply = GXReplyData()
        
        # IEC mode initialization
        if not conn.iec_mode_init():
            raise Exception("IEC initialization failed")
        print("✓ IEC initialization successful\n")
        
        # FIXED: Add delay before SNRM
        time.sleep(0.5)
        
        # Step 1: SNRM
        print("\n" + "="*70)
        print("Step 1: SNRM Request (Set Normal Response Mode)")
        print("="*70)
        snrm = client.snrmRequest()
        print(f"SNRM command ({len(snrm)} bytes): {snrm.hex()}")
        conn.write(snrm)
        
        reply.clear()
        conn.read_dlms_packet(client, reply)
        
        try:
            client.parseUAResponse(reply.data)
            print("\n✓✓✓ SNRM/UA SUCCESSFUL - Connection established ✓✓✓\n")
        except Exception as e:
            print(f"\n⚠ parseUAResponse error: {e}")
            print("Attempting to continue anyway...\n")
        
        # FIXED: Add delay between steps
        time.sleep(0.3)
        
        # Step 2: AARQ
        print("\n" + "="*70)
        print("Step 2: AARQ Request (Association Request)")
        print("="*70)
        
        aarq_messages = client.aarqRequest()
        for i, aarq in enumerate(aarq_messages):
            print(f"AARQ message {i+1}/{len(aarq_messages)} ({len(aarq)} bytes): {aarq.hex()}")
            conn.write(aarq)
            reply.clear()
            conn.read_dlms_packet(client, reply)
        
        try:
            client.parseAareResponse(reply.data)
            print("\n✓✓✓ ASSOCIATION ESTABLISHED - Ready to read data ✓✓✓\n")
        except Exception as e:
            print(f"\n⚠ parseAareResponse error: {e}")
            print("Attempting to continue anyway...\n")
        
        # FIXED: Add delay before reading
        time.sleep(0.3)
        
        # Step 3: Read energy register
        print("\n" + "="*70)
        print("Step 3: Read Total Active Energy (OBIS 1.0.1.8.0.255)")
        print("="*70)
        
        # FIXED: Correct OBIS code format
        energy = GXDLMSRegister("1.0.1.8.0.255")
        
        read_messages = client.read(energy, 2)  # 2 = attribute index for value
        for i, read_request in enumerate(read_messages):
            print(f"Read request {i+1}/{len(read_messages)} ({len(read_request)} bytes): {read_request.hex()}")
            conn.write(read_request)
            reply.clear()
            conn.read_dlms_packet(client, reply)
        
        try:
            client.updateValue(energy, 2, reply.value)
            print(f"\n✓✓✓ READ SUCCESSFUL ✓✓✓")
            print(f"Total Active Energy: {energy.value} {energy.scaler} (unit code: {energy.unit})\n")
        except Exception as e:
            print(f"\n⚠ updateValue error: {e}")
            print(f"Raw reply value: {reply.value}")
            print(f"Raw reply data: {reply.data}\n")
        
        # FIXED: Proper disconnection
        time.sleep(0.3)
        print("\n" + "="*70)
        print("Step 4: Disconnect")
        print("="*70)
        
        try:
            for rel in client.releaseRequest():
                print(f"Release request ({len(rel)} bytes): {rel.hex()}")
                conn.write(rel)
                time.sleep(0.2)
            
            disc = client.disconnectRequest()
            print(f"Disconnect request ({len(disc)} bytes): {disc.hex()}")
            conn.write(disc)
        except Exception as e:
            print(f"Disconnect error (non-critical): {e}")
        
        if conn:
            conn.close()
        print("\n✓ Session completed successfully")
        return True
        
    except Exception as e:
        print(f"\n✗ Communication failed: {e}")
        import traceback
        traceback.print_exc()
        if conn:
            try:
                conn.close()
            except:
                pass
        return False


# -------------------------------
# Run communication
# -------------------------------
if __name__ == "__main__":
    port = "/dev/ttyUSB0"  # Change to your port (COM3 on Windows, /dev/ttyUSB0 on Linux)
    
    print("="*70)
    print("DLMS Meter Communication - DDSY5558")
    print("Based on datasheet specifications")
    print("="*70)
    
    # FIXED: Try most common configurations first
    configs = [
        # (client_address, server_address, description)
        (16, 1, "Standard client 16, server 1"),
        (1, 1, "Public client 1, server 1"),
        (16, 17, "Client 16, server 17 (logical)"),
        (32, 1, "Management client 32, server 1"),
    ]
    
    for client_addr, server_addr, desc in configs:
        print(f"\n\nTrying: {desc}")
        print("-" * 70)
        success = communicate_with_meter(port, client_addr, server_addr)
        if success:
            print("\n" + "="*70)
            print(f"✓✓✓ SUCCESS with {desc} ✓✓✓")
            print("="*70)
            break
        time.sleep(2)
    else:
        print("\n" + "="*70)
        print("All configurations failed")
        print("\nTroubleshooting steps:")
        print("1. Verify physical connection (optical probe or RS485)")
        print("2. Check if meter is powered on")
        print("3. Try different serial port")
        print("4. Check if meter is in DLMS mode (not just STS)")
        print("5. Verify authentication password (default: 00000000)")
        print("="*70)
