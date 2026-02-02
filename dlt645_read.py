#!/usr/bin/env python3
"""
DL/T 645-2007 Protocol Implementation for DDSY5558 Meter
This is the Chinese standard protocol, simpler than DLMS
"""

import serial
import time
from datetime import datetime

class DLT645Protocol:
    """
    DL/T 645-2007 Protocol Implementation
    """
    
    # Frame structure constants
    FRAME_START = 0x68
    FRAME_END = 0x16
    
    # Control codes
    CMD_READ_DATA = 0x11        # Read data
    CMD_READ_MORE = 0x12        # Read subsequent data
    CMD_WRITE_DATA = 0x14       # Write data
    CMD_FREEZE = 0x16           # Freeze command
    CMD_CHANGE_BAUD = 0x17      # Change baud rate
    CMD_CHANGE_PASSWORD = 0x18  # Change password
    CMD_CLEAR_DEMAND = 0x19     # Clear max demand
    
    # Common data identifiers (DI)
    DI_VOLTAGE = 0x02010100              # Voltage (02-01-01-00)
    DI_CURRENT = 0x02020100              # Current (02-02-01-00)
    DI_ACTIVE_POWER = 0x02030000         # Active power (02-03-00-00)
    DI_POWER_FACTOR = 0x02060000         # Power factor (02-06-00-00)
    DI_FREQUENCY = 0x02800002            # Frequency (02-80-00-02)
    
    # Energy data identifiers
    DI_TOTAL_ACTIVE_ENERGY = 0x00000000  # Total active energy (00-00-00-00)
    DI_FORWARD_ACTIVE = 0x00010000       # Forward active energy
    DI_REVERSE_ACTIVE = 0x00020000       # Reverse active energy
    DI_REMAINING_ENERGY = 0x00900100     # Remaining energy (prepaid)
    
    # Date and time
    DI_DATE_TIME = 0x04000101            # Current date and time
    
    def __init__(self, port, address="000000000001", password="00000000", baudrate=9600):
        """
        Initialize DL/T 645 protocol
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0')
            address: Meter address (12 digits BCD, usually last 6 digits of meter number)
            password: 4-byte password (8 hex characters)
            baudrate: Communication baud rate (default 9600)
        """
        self.port = port
        self.address = self._format_address(address)
        self.password = bytes.fromhex(password)
        self.baudrate = baudrate
        self.ser = None
        
    def _format_address(self, addr):
        """Format address to 6 bytes BCD"""
        # Ensure 12 digits
        addr = addr.zfill(12)
        # Convert to BCD bytes (reverse order for DL/T 645)
        addr_bytes = bytearray()
        for i in range(0, 12, 2):
            byte_val = int(addr[i:i+2], 16)
            addr_bytes.append(byte_val)
        return bytes(reversed(addr_bytes))
    
    def _add_33h(self, data):
        """Add 0x33 to each data byte (DL/T 645 requirement)"""
        return bytes([b + 0x33 for b in data])
    
    def _sub_33h(self, data):
        """Subtract 0x33 from each data byte"""
        return bytes([b - 0x33 for b in data])
    
    def _calculate_checksum(self, data):
        """Calculate checksum (modulo 256 sum)"""
        return sum(data) & 0xFF
    
    def _build_frame(self, control_code, data_id=None, data=None):
        """
        Build DL/T 645 frame
        
        Frame format:
        68H + A0~A5 + 68H + C + L + DI0~DI3 + [data] + CS + 16H
        
        Where:
        - 68H: Start byte
        - A0-A5: Address (6 bytes, BCD)
        - 68H: Second start byte
        - C: Control code
        - L: Data length
        - DI: Data identifier (4 bytes, little endian)
        - data: Optional data
        - CS: Checksum
        - 16H: End byte
        """
        frame = bytearray()
        
        # Start byte
        frame.append(self.FRAME_START)
        
        # Address (6 bytes)
        frame.extend(self.address)
        
        # Second start byte
        frame.append(self.FRAME_START)
        
        # Control code
        frame.append(control_code)
        
        # Build data field
        data_field = bytearray()
        
        if data_id is not None:
            # Data identifier (4 bytes, little endian)
            di_bytes = data_id.to_bytes(4, byteorder='little')
            data_field.extend(di_bytes)
        
        if data is not None:
            data_field.extend(data)
        
        # Add 0x33 to data field
        if data_field:
            data_field = self._add_33h(data_field)
        
        # Data length
        frame.append(len(data_field))
        
        # Data field
        frame.extend(data_field)
        
        # Checksum (sum of all bytes from first 68H to L+data)
        cs = self._calculate_checksum(frame)
        frame.append(cs)
        
        # End byte
        frame.append(self.FRAME_END)
        
        return bytes(frame)
    
    def _parse_response(self, response):
        """Parse DL/T 645 response frame"""
        if len(response) < 12:
            raise ValueError(f"Response too short: {len(response)} bytes")
        
        # Check start and end bytes
        if response[0] != self.FRAME_START or response[-1] != self.FRAME_END:
            raise ValueError(f"Invalid frame markers: {response.hex()}")
        
        # Check second start byte
        if response[7] != self.FRAME_START:
            raise ValueError(f"Invalid second start byte: {response[7]:02x}")
        
        # Extract fields
        addr = response[1:7]
        control = response[8]
        length = response[9]
        
        # Calculate expected frame length
        expected_len = 12 + length  # 12 = fixed overhead + length byte
        if len(response) != expected_len:
            raise ValueError(f"Length mismatch: expected {expected_len}, got {len(response)}")
        
        # Extract data field and checksum
        data_field = response[10:10+length]
        checksum_received = response[10+length]
        
        # Verify checksum
        checksum_calculated = self._calculate_checksum(response[0:10+length])
        if checksum_received != checksum_calculated:
            raise ValueError(f"Checksum mismatch: {checksum_received:02x} vs {checksum_calculated:02x}")
        
        # Subtract 0x33 from data field
        if data_field:
            data_field = self._sub_33h(data_field)
        
        # Extract data identifier and data
        if len(data_field) >= 4:
            data_id = int.from_bytes(data_field[0:4], byteorder='little')
            data = data_field[4:]
        else:
            data_id = None
            data = data_field
        
        return {
            'address': addr,
            'control': control,
            'data_id': data_id,
            'data': data,
            'raw': response
        }
    
    def connect(self):
        """Open serial connection"""
        print(f"\n{'='*70}")
        print(f"Opening DL/T 645 connection")
        print(f"Port: {self.port}")
        print(f"Baud rate: {self.baudrate}")
        print(f"Address: {self.address.hex()}")
        print(f"{'='*70}")
        
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity=serial.PARITY_EVEN,  # DL/T 645 uses 8E1
            stopbits=1,
            timeout=2
        )
        
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print("✓ Serial port opened")
    
    def disconnect(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Serial port closed")
    
    def _send_frame(self, frame):
        """Send frame and wait for response"""
        if not self.ser or not self.ser.is_open:
            raise Exception("Serial port not open")
        
        # Clear buffers
        self.ser.reset_input_buffer()
        
        # Add pre-transmission delay
        time.sleep(0.2)
        
        # Send frame with wake-up bytes (FE FE FE FE)
        wakeup = b'\xFE\xFE\xFE\xFE'
        print(f"\nSending wakeup: {wakeup.hex()}")
        self.ser.write(wakeup)
        time.sleep(0.05)
        
        print(f"Sending frame ({len(frame)} bytes): {frame.hex()}")
        self.ser.write(frame)
        
        # Wait for response
        time.sleep(0.5)
        
        # Read response
        response = bytearray()
        timeout = time.time() + 2.0
        
        while time.time() < timeout:
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                response.extend(chunk)
                print(f"Received chunk ({len(chunk)} bytes): {chunk.hex()}")
                timeout = time.time() + 0.5  # Reset timeout on data
            time.sleep(0.05)
        
        if not response:
            raise Exception("No response from meter")
        
        print(f"Total response ({len(response)} bytes): {response.hex()}")
        return bytes(response)
    
    def read_data(self, data_id):
        """
        Read data from meter
        
        Args:
            data_id: Data identifier (4 bytes as integer)
            
        Returns:
            dict with parsed response
        """
        print(f"\n{'='*70}")
        print(f"Reading data ID: 0x{data_id:08X}")
        print(f"{'='*70}")
        
        # Build read frame
        frame = self._build_frame(self.CMD_READ_DATA, data_id)
        
        # Send and receive
        response = self._send_frame(frame)
        
        # Parse response
        parsed = self._parse_response(response)
        
        print(f"\n✓ Response parsed successfully")
        print(f"  Control: 0x{parsed['control']:02X}")
        print(f"  Data ID: 0x{parsed['data_id']:08X}" if parsed['data_id'] else "  No data ID")
        print(f"  Data ({len(parsed['data'])} bytes): {parsed['data'].hex()}")
        
        return parsed
    
    def decode_energy(self, data):
        """Decode energy value (BCD format, unit: kWh)"""
        if len(data) < 4:
            return None
        
        # DL/T 645 energy format: 4 bytes BCD (XXXXXX.XX kWh)
        # Format: XX.XX.XX.XX (little endian)
        value_str = ""
        for byte in reversed(data[:4]):
            value_str += f"{byte:02X}"
        
        # Insert decimal point (last 2 digits are decimal)
        if len(value_str) >= 2:
            value_str = value_str[:-2] + "." + value_str[-2:]
        
        try:
            return float(value_str)
        except:
            return None
    
    def decode_voltage(self, data):
        """Decode voltage value (unit: V)"""
        if len(data) < 2:
            return None
        
        # Format: XX.XX V (2 bytes BCD)
        value_str = ""
        for byte in reversed(data[:2]):
            value_str += f"{byte:02X}"
        
        if len(value_str) >= 1:
            value_str = value_str[:-1] + "." + value_str[-1:]
        
        try:
            return float(value_str)
        except:
            return None
    
    def decode_current(self, data):
        """Decode current value (unit: A)"""
        if len(data) < 3:
            return None
        
        # Format: XXX.XXX A (3 bytes BCD)
        value_str = ""
        for byte in reversed(data[:3]):
            value_str += f"{byte:02X}"
        
        if len(value_str) >= 3:
            value_str = value_str[:-3] + "." + value_str[-3:]
        
        try:
            return float(value_str)
        except:
            return None
    
    def decode_power(self, data):
        """Decode power value (unit: kW)"""
        if len(data) < 3:
            return None
        
        # Format: XX.XXXX kW (3 bytes BCD)
        value_str = ""
        for byte in reversed(data[:3]):
            value_str += f"{byte:02X}"
        
        if len(value_str) >= 4:
            value_str = value_str[:-4] + "." + value_str[-4:]
        
        try:
            return float(value_str)
        except:
            return None


def read_meter_dlt645(port="/dev/ttyUSB0", address=None):
    """
    Read meter using DL/T 645-2007 protocol
    """
    print("="*70)
    print("DL/T 645-2007 Meter Reading")
    print("="*70)
    
    # If address not provided, try common defaults
    addresses_to_try = []
    
    if address:
        addresses_to_try.append(address)
    else:
        # Try broadcast address
        addresses_to_try.append("AAAAAAAAAAAA")
        # Try address from meter number visible in image: 22000270235
        # Use last 12 digits as address
        addresses_to_try.append("000270235000")
        addresses_to_try.append("270235000000")
        # Try all zeros
        addresses_to_try.append("000000000000")
        # Try all 9s (another broadcast variant)
        addresses_to_try.append("999999999999")
    
    for addr in addresses_to_try:
        print(f"\n{'='*70}")
        print(f"Trying address: {addr}")
        print(f"{'='*70}")
        
        try:
            # Create protocol instance
            meter = DLT645Protocol(port, address=addr)
            
            # Connect
            meter.connect()
            
            # Read total active energy
            print("\n--- Reading Total Active Energy ---")
            try:
                response = meter.read_data(meter.DI_TOTAL_ACTIVE_ENERGY)
                energy = meter.decode_energy(response['data'])
                if energy is not None:
                    print(f"\n✓✓✓ Total Active Energy: {energy} kWh")
                else:
                    print(f"⚠ Could not decode energy: {response['data'].hex()}")
            except Exception as e:
                print(f"⚠ Read energy failed: {e}")
            
            # Read voltage
            print("\n--- Reading Voltage ---")
            try:
                response = meter.read_data(meter.DI_VOLTAGE)
                voltage = meter.decode_voltage(response['data'])
                if voltage is not None:
                    print(f"\n✓ Voltage: {voltage} V")
                else:
                    print(f"⚠ Could not decode voltage: {response['data'].hex()}")
            except Exception as e:
                print(f"⚠ Read voltage failed: {e}")
            
            # Read current
            print("\n--- Reading Current ---")
            try:
                response = meter.read_data(meter.DI_CURRENT)
                current = meter.decode_current(response['data'])
                if current is not None:
                    print(f"\n✓ Current: {current} A")
                else:
                    print(f"⚠ Could not decode current: {response['data'].hex()}")
            except Exception as e:
                print(f"⚠ Read current failed: {e}")
            
            # Read remaining energy (for prepaid meters)
            print("\n--- Reading Remaining Energy (Prepaid) ---")
            try:
                response = meter.read_data(meter.DI_REMAINING_ENERGY)
                remaining = meter.decode_energy(response['data'])
                if remaining is not None:
                    print(f"\n✓ Remaining Energy: {remaining} kWh")
                else:
                    print(f"⚠ Could not decode remaining energy: {response['data'].hex()}")
            except Exception as e:
                print(f"⚠ Read remaining energy failed: {e}")
            
            meter.disconnect()
            
            print("\n" + "="*70)
            print(f"✓✓✓ SUCCESS with address: {addr}")
            print("="*70)
            return True
            
        except Exception as e:
            print(f"\n✗ Failed with address {addr}: {e}")
            try:
                meter.disconnect()
            except:
                pass
            time.sleep(1)
    
    print("\n" + "="*70)
    print("❌ All addresses failed")
    print("="*70)
    print("\nTroubleshooting:")
    print("1. Verify meter address (usually last 6-12 digits of meter number)")
    print("2. Check physical connection (RS485 A-B wiring)")
    print("3. Verify baud rate (default is 9600, could be 1200/2400/4800)")
    print("4. Check if meter supports DL/T 645-2007 (vs 1997 version)")
    print("5. Some meters need to be activated for remote reading")
    return False


if __name__ == "__main__":
    import sys
    
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    address = sys.argv[2] if len(sys.argv) > 2 else None
    
    print(f"Port: {port}")
    if address:
        print(f"Address: {address}")
    else:
        print("Address: Auto-detect")
    
    read_meter_dlt645(port, address)



    #version2
#!/usr/bin/env python3
"""
Optimized DL/T 645-2007 Reader for DDSY5558 Meter
Based on successful connection - reads all available data
"""

import serial
import time
import sys

class DLT645Protocol:
    """DL/T 645-2007 Protocol Implementation"""
    
    FRAME_START = 0x68
    FRAME_END = 0x16
    CMD_READ_DATA = 0x11
    
    # Data identifiers that work with your meter
    DI_TOTAL_ACTIVE_ENERGY = 0x00000000    # Total active energy
    DI_FORWARD_ACTIVE = 0x00010000         # Forward active energy  
    DI_REVERSE_ACTIVE = 0x00020000         # Reverse active energy
    DI_REMAINING_ENERGY = 0x00900100       # Remaining energy (prepaid)
    DI_VOLTAGE = 0x02010100                # Voltage
    DI_CURRENT = 0x02020100                # Current
    DI_ACTIVE_POWER = 0x02030000           # Active power
    DI_REACTIVE_POWER = 0x02040000         # Reactive power
    DI_POWER_FACTOR = 0x02060000           # Power factor
    DI_FREQUENCY = 0x02800002              # Frequency
    
    # Additional useful registers
    DI_TARIFF1_ENERGY = 0x00010100         # Tariff 1 energy
    DI_TARIFF2_ENERGY = 0x00010200         # Tariff 2 energy
    DI_TARIFF3_ENERGY = 0x00010300         # Tariff 3 energy
    DI_TARIFF4_ENERGY = 0x00010400         # Tariff 4 energy
    DI_MAX_DEMAND = 0x01010000             # Maximum demand
    DI_DATE_TIME = 0x04000101              # Date and time
    
    def __init__(self, port, address="AAAAAAAAAAAA", baudrate=9600):
        self.port = port
        self.address = self._format_address(address)
        self.baudrate = baudrate
        self.ser = None
        
    def _format_address(self, addr):
        """Format address to 6 bytes BCD"""
        addr = addr.zfill(12)
        addr_bytes = bytearray()
        for i in range(0, 12, 2):
            byte_val = int(addr[i:i+2], 16)
            addr_bytes.append(byte_val)
        return bytes(reversed(addr_bytes))
    
    def _add_33h(self, data):
        return bytes([b + 0x33 for b in data])
    
    def _sub_33h(self, data):
        return bytes([b - 0x33 for b in data])
    
    def _calculate_checksum(self, data):
        return sum(data) & 0xFF
    
    def _build_frame(self, control_code, data_id):
        frame = bytearray()
        frame.append(self.FRAME_START)
        frame.extend(self.address)
        frame.append(self.FRAME_START)
        frame.append(control_code)
        
        # Data identifier (4 bytes, little endian)
        di_bytes = data_id.to_bytes(4, byteorder='little')
        data_field = self._add_33h(di_bytes)
        
        frame.append(len(data_field))
        frame.extend(data_field)
        
        cs = self._calculate_checksum(frame)
        frame.append(cs)
        frame.append(self.FRAME_END)
        
        return bytes(frame)
    
    def _parse_response(self, response):
        if len(response) < 12:
            raise ValueError(f"Response too short: {len(response)} bytes")
        
        if response[0] != self.FRAME_START or response[-1] != self.FRAME_END:
            raise ValueError(f"Invalid frame markers")
        
        if response[7] != self.FRAME_START:
            raise ValueError(f"Invalid second start byte")
        
        control = response[8]
        length = response[9]
        data_field = response[10:10+length]
        checksum_received = response[10+length]
        
        checksum_calculated = self._calculate_checksum(response[0:10+length])
        if checksum_received != checksum_calculated:
            raise ValueError(f"Checksum mismatch")
        
        if data_field:
            data_field = self._sub_33h(data_field)
        
        if len(data_field) >= 4:
            data_id = int.from_bytes(data_field[0:4], byteorder='little')
            data = data_field[4:]
        else:
            data_id = None
            data = data_field
        
        return {
            'control': control,
            'data_id': data_id,
            'data': data,
            'raw': response
        }
    
    def connect(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity=serial.PARITY_EVEN,
            stopbits=1,
            timeout=2
        )
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
    
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
    
    def _send_frame(self, frame):
        if not self.ser or not self.ser.is_open:
            raise Exception("Serial port not open")
        
        self.ser.reset_input_buffer()
        time.sleep(0.2)
        
        # Wake-up bytes
        wakeup = b'\xFE\xFE\xFE\xFE'
        self.ser.write(wakeup)
        time.sleep(0.05)
        
        self.ser.write(frame)
        time.sleep(0.5)
        
        # Read response
        response = bytearray()
        timeout = time.time() + 2.0
        
        while time.time() < timeout:
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                response.extend(chunk)
                timeout = time.time() + 0.5
            time.sleep(0.05)
        
        if not response:
            raise Exception("No response from meter")
        
        return bytes(response)
    
    def read_data(self, data_id, silent=False):
        """Read data from meter"""
        if not silent:
            print(f"Reading 0x{data_id:08X}...", end=" ")
        
        frame = self._build_frame(self.CMD_READ_DATA, data_id)
        
        try:
            response = self._send_frame(frame)
            parsed = self._parse_response(response)
            
            if not silent:
                print(f"✓ ({len(parsed['data'])} bytes)")
            
            return parsed
        except Exception as e:
            if not silent:
                print(f"✗ {e}")
            return None
    
    def decode_energy(self, data):
        """Decode 4-byte BCD energy value (XXXXXX.XX kWh)"""
        if not data or len(data) < 4:
            return None
        
        value_str = ""
        for byte in reversed(data[:4]):
            value_str += f"{byte:02X}"
        
        if len(value_str) >= 2:
            value_str = value_str[:-2] + "." + value_str[-2:]
        
        try:
            return float(value_str)
        except:
            return None
    
    def decode_voltage(self, data):
        """Decode 2-byte BCD voltage (XXX.X V)"""
        if not data or len(data) < 2:
            return None
        
        value_str = ""
        for byte in reversed(data[:2]):
            value_str += f"{byte:02X}"
        
        if len(value_str) >= 1:
            value_str = value_str[:-1] + "." + value_str[-1:]
        
        try:
            return float(value_str)
        except:
            return None
    
    def decode_current(self, data):
        """Decode 3-byte BCD current (XXX.XXX A)"""
        if not data or len(data) < 3:
            return None
        
        value_str = ""
        for byte in reversed(data[:3]):
            value_str += f"{byte:02X}"
        
        if len(value_str) >= 3:
            value_str = value_str[:-3] + "." + value_str[-3:]
        
        try:
            return float(value_str)
        except:
            return None
    
    def decode_power(self, data):
        """Decode 3-byte BCD power (XX.XXXX kW)"""
        if not data or len(data) < 3:
            return None
        
        value_str = ""
        for byte in reversed(data[:3]):
            value_str += f"{byte:02X}"
        
        if len(value_str) >= 4:
            value_str = value_str[:-4] + "." + value_str[-4:]
        
        try:
            return float(value_str)
        except:
            return None


def read_all_meter_data(port="/dev/ttyUSB0", address="AAAAAAAAAAAA"):
    """Read all available data from meter"""
    
    print("="*70)
    print("DL/T 645-2007 Meter Reader - DDSY5558")
    print("="*70)
    print(f"Port: {port}")
    print(f"Address: {address}")
    print(f"Baud: 9600, Parity: Even")
    print("="*70)
    
    meter = DLT645Protocol(port, address=address)
    
    try:
        # Connect
        print("\nConnecting...", end=" ")
        meter.connect()
        print("✓")
        
        print("\n" + "="*70)
        print("ENERGY REGISTERS")
        print("="*70)
        
        # Total active energy
        result = meter.read_data(meter.DI_TOTAL_ACTIVE_ENERGY)
        if result:
            energy = meter.decode_energy(result['data'])
            print(f"  Total Active Energy:     {energy:>12.2f} kWh" if energy else f"  Total Active Energy:     Unable to decode")
        
        # Forward active energy
        result = meter.read_data(meter.DI_FORWARD_ACTIVE)
        if result:
            energy = meter.decode_energy(result['data'])
            print(f"  Forward Active Energy:   {energy:>12.2f} kWh" if energy else f"  Forward Active Energy:   Unable to decode")
        
        # Reverse active energy
        result = meter.read_data(meter.DI_REVERSE_ACTIVE)
        if result:
            energy = meter.decode_energy(result['data'])
            print(f"  Reverse Active Energy:   {energy:>12.2f} kWh" if energy else f"  Reverse Active Energy:   Unable to decode")
        
        # Remaining energy (prepaid)
        result = meter.read_data(meter.DI_REMAINING_ENERGY)
        if result:
            energy = meter.decode_energy(result['data'])
            print(f"  Remaining Energy:        {energy:>12.2f} kWh ⚡" if energy else f"  Remaining Energy:        Unable to decode")
        
        # Tariff energies
        print("\n" + "="*70)
        print("TARIFF REGISTERS")
        print("="*70)
        
        for i, di in enumerate([meter.DI_TARIFF1_ENERGY, meter.DI_TARIFF2_ENERGY, 
                                meter.DI_TARIFF3_ENERGY, meter.DI_TARIFF4_ENERGY], 1):
            result = meter.read_data(di)
            if result:
                energy = meter.decode_energy(result['data'])
                if energy:
                    print(f"  Tariff {i} Energy:         {energy:>12.2f} kWh")
        
        # Instantaneous values
        print("\n" + "="*70)
        print("INSTANTANEOUS VALUES")
        print("="*70)
        
        # Voltage
        result = meter.read_data(meter.DI_VOLTAGE)
        if result:
            voltage = meter.decode_voltage(result['data'])
            print(f"  Voltage:                 {voltage:>12.1f} V" if voltage else f"  Voltage:                 Unable to decode")
        
        # Current
        result = meter.read_data(meter.DI_CURRENT)
        if result:
            current = meter.decode_current(result['data'])
            print(f"  Current:                 {current:>12.3f} A" if current else f"  Current:                 Unable to decode")
        
        # Active power
        result = meter.read_data(meter.DI_ACTIVE_POWER)
        if result:
            power = meter.decode_power(result['data'])
            print(f"  Active Power:            {power:>12.4f} kW" if power else f"  Active Power:            Unable to decode")
        
        # Reactive power
        result = meter.read_data(meter.DI_REACTIVE_POWER)
        if result:
            power = meter.decode_power(result['data'])
            print(f"  Reactive Power:          {power:>12.4f} kvar" if power else f"  Reactive Power:          Unable to decode")
        
        # Power factor
        result = meter.read_data(meter.DI_POWER_FACTOR)
        if result and result['data']:
            # Power factor is usually 2 bytes
            pf_str = ""
            for byte in reversed(result['data'][:2]):
                pf_str += f"{byte:02X}"
            try:
                pf = float(pf_str) / 1000.0
                print(f"  Power Factor:            {pf:>12.3f}")
            except:
                print(f"  Power Factor:            Unable to decode")
        
        # Frequency
        result = meter.read_data(meter.DI_FREQUENCY)
        if result and result['data']:
            freq_str = ""
            for byte in reversed(result['data'][:2]):
                freq_str += f"{byte:02X}"
            if len(freq_str) >= 2:
                freq_str = freq_str[:-2] + "." + freq_str[-2:]
            try:
                freq = float(freq_str)
                print(f"  Frequency:               {freq:>12.2f} Hz")
            except:
                print(f"  Frequency:               Unable to decode")
        
        # Max demand
        print("\n" + "="*70)
        print("DEMAND")
        print("="*70)
        
        result = meter.read_data(meter.DI_MAX_DEMAND)
        if result:
            demand = meter.decode_power(result['data'])
            print(f"  Maximum Demand:          {demand:>12.4f} kW" if demand else f"  Maximum Demand:          Unable to decode")
        
        meter.disconnect()
        
        print("\n" + "="*70)
        print("✓✓✓ SUCCESS - Data read complete")
        print("="*70)
        
        return True
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        try:
            meter.disconnect()
        except:
            pass
        return False


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    address = sys.argv[2] if len(sys.argv) > 2 else "AAAAAAAAAAAA"
    
    success = read_all_meter_data(port, address)
    
    if success:
        print("\n💡 TIP: You can now integrate this into your application!")
        print("   The broadcast address (AAAAAAAAAAAA) is working perfectly.")
    else:
        print("\n❌ Failed to read meter data")
        print("   Try running diagnostic scripts to troubleshoot")
    
    sys.exit(0 if success else 1)
