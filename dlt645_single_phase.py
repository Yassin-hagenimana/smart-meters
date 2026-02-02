#!/usr/bin/env python3
"""
DL/T 645-2007 Reader for DDSY5558 Single-Phase Smart Meter
Fixed version - separated current and voltage readings
"""

import serial
import time
import sys
import json
from datetime import datetime

class DLT645Protocol:
    """DL/T 645-2007 Protocol Implementation"""
    
    FRAME_START = 0x68
    FRAME_END = 0x16
    CMD_READ_DATA = 0x11
    
    # Energy registers
    DI_TOTAL_ACTIVE_ENERGY = 0x00000000
    DI_FORWARD_ACTIVE = 0x00010000
    DI_REVERSE_ACTIVE = 0x00020000
    DI_REMAINING_ENERGY = 0x00900100
    
    # Tariff registers (T1-T8)
    DI_TARIFF1_ENERGY = 0x00010100
    DI_TARIFF2_ENERGY = 0x00010200
    DI_TARIFF3_ENERGY = 0x00010300
    DI_TARIFF4_ENERGY = 0x00010400
    DI_TARIFF5_ENERGY = 0x00010500
    DI_TARIFF6_ENERGY = 0x00010600
    DI_TARIFF7_ENERGY = 0x00010700
    DI_TARIFF8_ENERGY = 0x00010800
    
    # Instantaneous values
    DI_VOLTAGE = 0x02010100
    DI_CURRENT = 0x02020100
    DI_ACTIVE_POWER = 0x02030000
    DI_REACTIVE_POWER = 0x02040000
    DI_APPARENT_POWER = 0x02050000
    DI_POWER_FACTOR = 0x02060000
    DI_FREQUENCY = 0x02800002
    
    # Demand
    DI_MAX_DEMAND = 0x01010000
    DI_CURRENT_DEMAND = 0x03040000
    
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
            parity=serial.PARITY_NONE,
            stopbits=1,
            timeout=2
        )
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
    
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
    
    def _send_frame(self, frame, retries=2):
        if not self.ser or not self.ser.is_open:
            raise Exception("Serial port not open")
        
        for attempt in range(retries):
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                time.sleep(0.3)
                
                wakeup = b'\xFE\xFE\xFE\xFE'
                self.ser.write(wakeup)
                time.sleep(0.1)
                
                self.ser.write(frame)
                time.sleep(0.6)
                
                response = bytearray()
                timeout = time.time() + 3.0
                
                while time.time() < timeout:
                    if self.ser.in_waiting > 0:
                        chunk = self.ser.read(self.ser.in_waiting)
                        response.extend(chunk)
                        timeout = time.time() + 0.5
                    time.sleep(0.05)
                
                if response:
                    return bytes(response)
                
                if attempt < retries - 1:
                    time.sleep(1.0)
                    
            except Exception as e:
                if attempt < retries - 1:
                    time.sleep(0.5)
                else:
                    raise
        
        raise Exception("No response from meter")
    
    def read_data(self, data_id, silent=False):
        """Read data from meter"""
        if not silent:
            print(f"Reading 0x{data_id:08X}... ", end="", flush=True)
        
        frame = self._build_frame(self.CMD_READ_DATA, data_id)
        
        try:
            response = self._send_frame(frame)
            parsed = self._parse_response(response)
            
            if not silent:
                print(f"OK ({len(parsed['data'])} bytes) - Data: {parsed['data'].hex()}")
            
            time.sleep(0.3)
            
            return parsed
        except Exception as e:
            if not silent:
                print(f"FAIL - {str(e)}")
            
            time.sleep(0.2)
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
    
    def decode_generic_bcd(self, data, decimal_places=2):
        """Decode generic BCD value with configurable decimal places"""
        if not data:
            return None
        
        value_str = ""
        for byte in reversed(data):
            value_str += f"{byte:02X}"
        
        if decimal_places > 0 and len(value_str) >= decimal_places:
            value_str = value_str[:-decimal_places] + "." + value_str[-decimal_places:]
        
        try:
            return float(value_str) if value_str and value_str != "." else None
        except:
            return None


def read_meter(port="/dev/ttyUSB0", address="AAAAAAAAAAAA"):
    """Read meter data with detailed output"""
    
    print("="*80)
    print("DDSY5558 Single-Phase Prepaid Meter - DL/T 645-2007")
    print("="*80)
    print(f"Port:      {port}")
    print(f"Address:   {address}")
    print(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*80)
    
    meter = DLT645Protocol(port, address=address)
    
    try:
        print("\nConnecting...", end=" ", flush=True)
        meter.connect()
        print("OK\n")
        
        # Energy Registers
        print("="*80)
        print("ENERGY REGISTERS")
        print("="*80)
        
        result = meter.read_data(meter.DI_TOTAL_ACTIVE_ENERGY)
        if result and result['data']:
            value = meter.decode_energy(result['data'])
            print(f"Total Active Energy:           {value:.2f} kWh" if value is not None else "Total Active Energy:           Failed to decode")
        
        result = meter.read_data(meter.DI_FORWARD_ACTIVE)
        if result and result['data']:
            value = meter.decode_energy(result['data'])
            print(f"Forward Active Energy:         {value:.2f} kWh" if value is not None else "Forward Active Energy:         Failed to decode")
        
        result = meter.read_data(meter.DI_REVERSE_ACTIVE)
        if result and result['data']:
            value = meter.decode_energy(result['data'])
            print(f"Reverse Active Energy:         {value:.2f} kWh" if value is not None else "Reverse Active Energy:         Failed to decode")
        
        result = meter.read_data(meter.DI_REMAINING_ENERGY)
        if result and result['data']:
            value = meter.decode_energy(result['data'])
            if value is not None:
                print(f"Remaining Energy (Prepaid):    {value:.2f} kWh")
                if value < 10.0:
                    print("                               WARNING: Low balance")
            else:
                print("Remaining Energy (Prepaid):    Failed to decode")
        
        # Instantaneous Values
        print("\n" + "="*80)
        print("INSTANTANEOUS VALUES")
        print("="*80)
        
        # Voltage
        print("\n[VOLTAGE]")
        result = meter.read_data(meter.DI_VOLTAGE)
        if result and result['data']:
            print(f"Raw data bytes: {result['data'].hex()}")
            value = meter.decode_voltage(result['data'])
            if value is not None:
                print(f"Decoded voltage: {value:.1f} V")
            else:
                print("Failed to decode voltage")
        else:
            print("No voltage data received")
        
        # Current
        print("\n[CURRENT]")
        result = meter.read_data(meter.DI_CURRENT)
        if result and result['data']:
            print(f"Raw data bytes: {result['data'].hex()}")
            value = meter.decode_current(result['data'])
            if value is not None:
                print(f"Decoded current: {value:.3f} A")
            else:
                print("Failed to decode current")
        else:
            print("No current data received")
        
        # Active Power
        print("\n[ACTIVE POWER]")
        result = meter.read_data(meter.DI_ACTIVE_POWER)
        if result and result['data']:
            print(f"Raw data bytes: {result['data'].hex()}")
            value = meter.decode_power(result['data'])
            if value is not None:
                print(f"Decoded power: {value:.4f} kW ({value*1000:.1f} W)")
            else:
                print("Failed to decode power")
        else:
            print("No power data received")
        
        # Reactive Power
        print("\n[REACTIVE POWER]")
        result = meter.read_data(meter.DI_REACTIVE_POWER)
        if result and result['data']:
            print(f"Raw data bytes: {result['data'].hex()}")
            value = meter.decode_power(result['data'])
            if value is not None:
                print(f"Decoded reactive power: {value:.4f} kvar")
            else:
                print("Failed to decode reactive power")
        
        # Frequency
        print("\n[FREQUENCY]")
        result = meter.read_data(meter.DI_FREQUENCY)
        if result and result['data']:
            print(f"Raw data bytes: {result['data'].hex()}")
            value = meter.decode_generic_bcd(result['data'], decimal_places=2)
            if value is not None:
                print(f"Decoded frequency: {value:.2f} Hz")
            else:
                print("Failed to decode frequency")
        
        # Power Factor
        print("\n[POWER FACTOR]")
        result = meter.read_data(meter.DI_POWER_FACTOR)
        if result and result['data']:
            print(f"Raw data bytes: {result['data'].hex()}")
            value = meter.decode_generic_bcd(result['data'], decimal_places=3)
            if value is not None:
                print(f"Decoded power factor: {value:.3f}")
            else:
                print("Failed to decode power factor")
        
        meter.disconnect()
        
        print("\n" + "="*80)
        print("Reading complete")
        print("="*80)
        
        return True
        
    except Exception as e:
        print(f"\nError: {e}")
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
    
    success = read_meter(port, address)
    sys.exit(0 if success else 1)