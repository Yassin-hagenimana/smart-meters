#!/usr/bin/env python3
"""
Comprehensive DL/T 645-2007 Reader for DDSY5558 Smart Meter
Reads all available energy, demand, and instantaneous data
Includes discovery mode and export functionality
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
    
    # Tariff registers
    DI_TARIFF1_ENERGY = 0x00010100         # Tariff 1 energy
    DI_TARIFF2_ENERGY = 0x00010200         # Tariff 2 energy
    DI_TARIFF3_ENERGY = 0x00010300         # Tariff 3 energy
    DI_TARIFF4_ENERGY = 0x00010400         # Tariff 4 energy
    DI_MAX_DEMAND = 0x01010000             # Maximum demand
    
    # Additional phase-specific and harmonics data
    DI_PHASE_A_VOLTAGE = 0x02010101        # Phase A voltage
    DI_PHASE_B_VOLTAGE = 0x02010102        # Phase B voltage
    DI_PHASE_C_VOLTAGE = 0x02010103        # Phase C voltage
    DI_PHASE_A_CURRENT = 0x02020101        # Phase A current
    DI_PHASE_B_CURRENT = 0x02020102        # Phase B current
    DI_PHASE_C_CURRENT = 0x02020103        # Phase C current
    DI_PHASE_A_POWER = 0x02030101          # Phase A active power
    DI_PHASE_B_POWER = 0x02030102          # Phase B active power
    DI_PHASE_C_POWER = 0x02030103          # Phase C active power
    
    # Additional reactive and power factor data
    DI_PHASE_A_REACTIVE = 0x02040101       # Phase A reactive power
    DI_PHASE_B_REACTIVE = 0x02040102       # Phase B reactive power
    DI_PHASE_C_REACTIVE = 0x02040103       # Phase C reactive power
    DI_PHASE_A_POWER_FACTOR = 0x02060101   # Phase A power factor
    DI_PHASE_B_POWER_FACTOR = 0x02060102   # Phase B power factor
    DI_PHASE_C_POWER_FACTOR = 0x02060103   # Phase C power factor
    
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
    
    def _send_frame(self, frame, retries=1):
        if not self.ser or not self.ser.is_open:
            raise Exception("Serial port not open")
        
        for attempt in range(retries):
            try:
                # Clear buffers
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                time.sleep(0.3)
                
                # Send wake-up bytes (required for each request)
                wakeup = b'\xFE\xFE\xFE\xFE'
                self.ser.write(wakeup)
                time.sleep(0.1)
                
                # Send frame
                self.ser.write(frame)
                time.sleep(0.6)
                
                # Read response
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
                
                # If no response, wait longer before retry
                if attempt < retries - 1:
                    time.sleep(1.0)
                    
            except Exception as e:
                if attempt < retries - 1:
                    time.sleep(1.0)
                else:
                    raise
        
        raise Exception("No response from meter")
    
    def read_data(self, data_id, silent=False):
        """Read data from meter"""
        if not silent:
            print(f"Reading 0x{data_id:08X}...", end=" ")
        
        frame = self._build_frame(self.CMD_READ_DATA, data_id)
        
        try:
            response = self._send_frame(frame)
            parsed = self._parse_response(response)
            
            if not silent:
                print(f"OK ({len(parsed['data'])} bytes)")
            
            # Wait between successful reads to avoid overwhelming the meter
            time.sleep(1.5)
            
            return parsed
        except Exception as e:
            if not silent:
                print(f"FAIL {e}")
            
            # Shorter wait after failures (likely non-existent register)
            time.sleep(0.5)
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
    
    def discover_data_ids(self, start=0x00000000, end=0x00FFFFFF, step=0x00000100):
        """Discover available data IDs (slow, for mapping)"""
        print("Discovering available data IDs... (this may take a while)")
        found = []
        
        di = start
        while di <= end:
            try:
                result = self.read_data(di, silent=True)
                if result and result['data'] and len(result['data']) > 0:
                    found.append({
                        'di': f"0x{di:08X}",
                        'data_len': len(result['data']),
                        'data_hex': result['data'].hex()
                    })
                    print(f"  Found: 0x{di:08X} ({len(result['data'])} bytes)")
            except:
                pass
            
            di += step
        
        return found


def read_all_meter_data(port="/dev/ttyUSB0", address="AAAAAAAAAAAA", export_json=False):
    """Read all available data from meter"""
    
    print("="*70)
    print("DL/T 645-2007 Comprehensive Meter Reader - DDSY5558")
    print("="*70)
    print(f"Port: {port}")
    print(f"Address: {address}")
    print(f"Baud: 9600, Parity: Even")
    print(f"Timestamp: {datetime.now().isoformat()}")
    print("="*70)
    
    meter = DLT645Protocol(port, address=address)
    data_log = {'timestamp': datetime.now().isoformat(), 'readings': {}}
    
    try:
        # Connect
        print("\nConnecting...", end=" ")
        meter.connect()
        print("OK")
        
        print("\n" + "="*70)
        print("ENERGY REGISTERS (kWh)")
        print("="*70)
        
        # Total active energy
        result = meter.read_data(meter.DI_TOTAL_ACTIVE_ENERGY)
        if result:
            energy = meter.decode_energy(result['data'])
            val_str = f"{energy:>12.2f}" if energy else "Unable to decode"
            print(f"  Total Active Energy:     {val_str} kWh")
            if energy:
                data_log['readings']['total_active_energy'] = energy
        
        # Forward active energy
        result = meter.read_data(meter.DI_FORWARD_ACTIVE)
        if result:
            energy = meter.decode_energy(result['data'])
            val_str = f"{energy:>12.2f}" if energy else "Unable to decode"
            print(f"  Forward Active Energy:   {val_str} kWh")
            if energy:
                data_log['readings']['forward_active_energy'] = energy
        
        # Reverse active energy
        result = meter.read_data(meter.DI_REVERSE_ACTIVE)
        if result:
            energy = meter.decode_energy(result['data'])
            val_str = f"{energy:>12.2f}" if energy else "Unable to decode"
            print(f"  Reverse Active Energy:   {val_str} kWh")
            if energy:
                data_log['readings']['reverse_active_energy'] = energy
        
        # Remaining energy (prepaid)
        result = meter.read_data(meter.DI_REMAINING_ENERGY)
        if result:
            energy = meter.decode_energy(result['data'])
            val_str = f"{energy:>12.2f}" if energy else "Unable to decode"
            print(f"  Remaining Energy:        {val_str} kWh [PREPAID]")
            if energy:
                data_log['readings']['remaining_energy'] = energy
        
        # Tariff energies
        print("\n" + "="*70)
        print("TARIFF REGISTERS (kWh)")
        print("="*70)
        
        tariff_dis = [
            (meter.DI_TARIFF1_ENERGY, "Tariff 1"),
            (meter.DI_TARIFF2_ENERGY, "Tariff 2"),
            (meter.DI_TARIFF3_ENERGY, "Tariff 3"),
            (meter.DI_TARIFF4_ENERGY, "Tariff 4"),
        ]
        
        for di, label in tariff_dis:
            result = meter.read_data(di)
            if result:
                energy = meter.decode_energy(result['data'])
                if energy:
                    print(f"  {label:<20}     {energy:>12.2f} kWh")
                    data_log['readings'][label.lower().replace(" ", "_")] = energy
        
        # Instantaneous values
        print("\n" + "="*70)
        print("INSTANTANEOUS VALUES")
        print("="*70)
        
        # Voltage (Total)
        result = meter.read_data(meter.DI_VOLTAGE)
        if result:
            voltage = meter.decode_voltage(result['data'])
            val_str = f"{voltage:>12.1f}" if voltage else "Unable to decode"
            print(f"  Total Voltage:           {val_str} V")
            if voltage:
                data_log['readings']['voltage'] = voltage
        
        # Phase voltages
        print("\n  Phase Voltages:")
        phases_v = [
            (meter.DI_PHASE_A_VOLTAGE, "A"),
            (meter.DI_PHASE_B_VOLTAGE, "B"),
            (meter.DI_PHASE_C_VOLTAGE, "C"),
        ]
        
        for di, phase in phases_v:
            result = meter.read_data(di)
            if result:
                voltage = meter.decode_voltage(result['data'])
                if voltage:
                    print(f"    Phase {phase}:           {voltage:>12.1f} V")
                    data_log['readings'][f'phase_{phase.lower()}_voltage'] = voltage
        
        # Current (Total)
        result = meter.read_data(meter.DI_CURRENT)
        if result:
            current = meter.decode_current(result['data'])
            val_str = f"{current:>12.3f}" if current else "Unable to decode"
            print(f"\n  Total Current:           {val_str} A")
            if current:
                data_log['readings']['current'] = current
        
        # Phase currents
        print("\n  Phase Currents:")
        phases_i = [
            (meter.DI_PHASE_A_CURRENT, "A"),
            (meter.DI_PHASE_B_CURRENT, "B"),
            (meter.DI_PHASE_C_CURRENT, "C"),
        ]
        
        for di, phase in phases_i:
            result = meter.read_data(di)
            if result:
                current = meter.decode_current(result['data'])
                if current:
                    print(f"    Phase {phase}:           {current:>12.3f} A")
                    data_log['readings'][f'phase_{phase.lower()}_current'] = current
        
        # Active power (Total)
        result = meter.read_data(meter.DI_ACTIVE_POWER)
        if result:
            power = meter.decode_power(result['data'])
            val_str = f"{power:>12.4f}" if power else "Unable to decode"
            print(f"\n  Total Active Power:      {val_str} kW")
            if power:
                data_log['readings']['active_power'] = power
        
        # Phase active powers
        print("\n  Phase Active Powers:")
        phases_p = [
            (meter.DI_PHASE_A_POWER, "A"),
            (meter.DI_PHASE_B_POWER, "B"),
            (meter.DI_PHASE_C_POWER, "C"),
        ]
        
        for di, phase in phases_p:
            result = meter.read_data(di)
            if result:
                power = meter.decode_power(result['data'])
                if power:
                    print(f"    Phase {phase}:           {power:>12.4f} kW")
                    data_log['readings'][f'phase_{phase.lower()}_active_power'] = power
        
        # Reactive power (Total)
        result = meter.read_data(meter.DI_REACTIVE_POWER)
        if result:
            power = meter.decode_power(result['data'])
            val_str = f"{power:>12.4f}" if power else "Unable to decode"
            print(f"\n  Total Reactive Power:    {val_str} kvar")
            if power:
                data_log['readings']['reactive_power'] = power
        
        # Phase reactive powers
        print("\n  Phase Reactive Powers:")
        phases_q = [
            (meter.DI_PHASE_A_REACTIVE, "A"),
            (meter.DI_PHASE_B_REACTIVE, "B"),
            (meter.DI_PHASE_C_REACTIVE, "C"),
        ]
        
        for di, phase in phases_q:
            result = meter.read_data(di)
            if result:
                power = meter.decode_power(result['data'])
                if power:
                    print(f"    Phase {phase}:           {power:>12.4f} kvar")
                    data_log['readings'][f'phase_{phase.lower()}_reactive_power'] = power
        
        # Power factor (Total)
        result = meter.read_data(meter.DI_POWER_FACTOR)
        if result and result['data']:
            pf = meter.decode_generic_bcd(result['data'], decimal_places=3)
            if pf:
                print(f"\n  Total Power Factor:      {pf:>12.3f}")
                data_log['readings']['power_factor'] = pf
            else:
                print(f"\n  Total Power Factor:      Unable to decode")
        
        # Phase power factors
        print("\n  Phase Power Factors:")
        phases_pf = [
            (meter.DI_PHASE_A_POWER_FACTOR, "A"),
            (meter.DI_PHASE_B_POWER_FACTOR, "B"),
            (meter.DI_PHASE_C_POWER_FACTOR, "C"),
        ]
        
        for di, phase in phases_pf:
            result = meter.read_data(di)
            if result:
                pf = meter.decode_generic_bcd(result['data'], decimal_places=3)
                if pf:
                    print(f"    Phase {phase}:           {pf:>12.3f}")
                    data_log['readings'][f'phase_{phase.lower()}_power_factor'] = pf
        
        # Frequency
        result = meter.read_data(meter.DI_FREQUENCY)
        if result and result['data']:
            freq = meter.decode_generic_bcd(result['data'], decimal_places=2)
            if freq:
                print(f"\n  Frequency:               {freq:>12.2f} Hz")
                data_log['readings']['frequency'] = freq
            else:
                print(f"\n  Frequency:               Unable to decode")
        
        # Max demand
        print("\n" + "="*70)
        print("DEMAND")
        print("="*70)
        
        result = meter.read_data(meter.DI_MAX_DEMAND)
        if result:
            demand = meter.decode_power(result['data'])
            val_str = f"{demand:>12.4f}" if demand else "Unable to decode"
            print(f"  Maximum Demand:          {val_str} kW")
            if demand:
                data_log['readings']['max_demand'] = demand
        
        meter.disconnect()
        
        # Export data if requested
        if export_json:
            filename = f"meter_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(filename, 'w') as f:
                json.dump(data_log, f, indent=2)
            print(f"\n[OK] Data exported to: {filename}")
        
        print("\n" + "="*70)
        print("[SUCCESS] Data read complete")
        print(f"[OK] Total readings captured: {len(data_log['readings'])}")
        print("="*70)
        
        return True, data_log
        
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        try:
            meter.disconnect()
        except:
            pass
        return False, None


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    address = sys.argv[2] if len(sys.argv) > 2 else "AAAAAAAAAAAA"
    export_json = "--export" in sys.argv or "-j" in sys.argv
    
    success, data = read_all_meter_data(port, address, export_json=export_json)
    
    if success:
        print("\n[TIPS]")
        print("   • Use --export flag to save data as JSON")
        print("   • The broadcast address (AAAAAAAAAAAA) works for all meters")
        print("   • If you know the meter's specific address, use it for faster queries")
        print("   • Schedule this script with cron for periodic data collection")
    else:
        print("\n[FAILED] Failed to read meter data")
        print("   • Check RS485 A/B wiring")
        print("   • Verify meter is powered and awake")
        print("   • Try different serial port (e.g., /dev/ttyUSB1)")
    
    sys.exit(0 if success else 1)