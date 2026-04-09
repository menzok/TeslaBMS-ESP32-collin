
#PI SIDE DRIVER


import os
import sys
import time
import serial
import struct
import logging
from dbus.mainloop.glib import DBusGMainLoop
import dbus
import gobject

# Add velib_python path (standard on Venus OS)
sys.path.insert(1, '/opt/victronenergy/dbus-serialbattery/ext/velib_python')
from vedbus import VeDbusService

class TeslaBMS:
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        self.ser = None
        self.voltage = 0.0
        self.current = 0.0
        self.soc = 50
        self.temp = 25.0
        self.power = 0
        self.avg_cell_volt = 3.3
        self.connected = False
        self.alarm_flags = 0

    def modbus_crc16(self, data):
        """Exact same CRC-16/MODBUS as in ExternalCommsLayer.cpp"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.5)
            logging.info(f"✅ Opened {self.port} for TeslaBMS-ESP32")
            return True
        except Exception as e:
            logging.error(f"❌ Failed to open {self.port}: {e}")
            return False

    def request_data(self):
        """Send "Send me a data packet immediately" command (same as EXT_CMD_SEND_DATA)"""
        if not self.ser:
            return
        cmd_byte = bytes([0x03])                                      # 0x03 = send data now
        crc = self.modbus_crc16(cmd_byte)
        frame = b'\xAA' + cmd_byte + struct.pack('<H', crc)          # low byte first (matches C++)
        try:
            self.ser.write(frame)
            time.sleep(0.05)  # tiny delay for ESP32 to respond
        except Exception as e:
            logging.error(f"Command send error: {e}")

    def read_frame(self):
        """Read and parse one full HEX frame (unsolicited or requested)"""
        if not self.ser:
            return False

        try:
            self.request_data()                     # ask for fresh data

            # Find start byte 0xAA
            while True:
                byte = self.ser.read(1)
                if not byte or byte[0] != 0xAA:
                    continue
                break

            # Read 20 bytes: 18 payload + 2-byte CRC
            frame = self.ser.read(20)
            if len(frame) != 20:
                return False

            payload = frame[:18]
            crc_received = struct.unpack('<H', frame[18:])[0]
            crc_calculated = self.modbus_crc16(payload)

            if crc_calculated != crc_received:
                logging.warning("CRC mismatch – ignoring frame")
                return False

            # Parse exactly as defined in ExternalCommsLayer.cpp (big-endian for 16-bit fields)
            pack_v_mv     = struct.unpack('>H', payload[0:2])[0]
            pack_i_ma     = struct.unpack('>h', payload[2:4])[0]
            soc           = payload[4]
            temp_c        = struct.unpack('b', payload[5:6])[0]
            power_w       = struct.unpack('>h', payload[6:8])[0]
            avg_cell_mv   = struct.unpack('>H', payload[8:10])[0]
            # reserved16    = struct.unpack('>H', payload[10:12])[0]
            alarm_flags   = struct.unpack('>H', payload[12:14])[0]
            # overlordState = payload[14]
            # contactorState= payload[15]

            self.voltage      = pack_v_mv / 1000.0      # 10 mV resolution → V
            self.current      = pack_i_ma / 10.0         # 100 mA resolution → A
            self.soc          = soc
            self.temp         = float(temp_c)
            self.power        = power_w
            self.avg_cell_volt= avg_cell_mv / 1000.0
            self.alarm_flags  = alarm_flags
            self.connected    = True

            logging.info(f"✅ Parsed: {self.voltage:.2f}V | {self.current:+.1f}A | SOC {self.soc}% | {self.temp}°C")
            return True

        except Exception as e:
            logging.error(f"Frame read error: {e}")
            return False

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'

    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s: %(message)s')

    DBusGMainLoop(set_as_default=True)

    bms = TeslaBMS(port)
    if not bms.connect():
        sys.exit(1)

    # Register as Victron battery on DBus
    dbusservice = VeDbusService("com.victronenergy.battery.teslabms",
                                get_bus=dbus.SessionBus() if 'DBUS_SESSION_BUS_ADDRESS' in os.environ else dbus.SystemBus())

    # Required paths
    dbusservice.add_path('/Connected',           1)
    dbusservice.add_path('/DeviceInstance',      0)
    dbusservice.add_path('/ProductId',           0)
    dbusservice.add_path('/ProductName',         'TeslaBMS-ESP32')
    dbusservice.add_path('/CustomName',          'TeslaBMS', writeable=True)
    dbusservice.add_path('/FirmwareVersion',     '1.0')
    dbusservice.add_path('/HardwareVersion',     'ESP32 HEX')
    dbusservice.add_path('/Soc',                 50)
    dbusservice.add_path('/Dc/0/Voltage',        0.0)
    dbusservice.add_path('/Dc/0/Current',        0.0)
    dbusservice.add_path('/Dc/0/Power',          0)
    dbusservice.add_path('/Dc/0/Temperature',    25)
    dbusservice.add_path('/Info/MaxChargeCurrent', 200)
    dbusservice.add_path('/Info/MaxDischargeCurrent', 200)
    dbusservice.add_path('/Info/MaxChargeVoltage', 58.0)   # adjust for your pack
    dbusservice.add_path('/Info/MinDischargeVoltage', 42.0)

    # Alarms (expand later with your alarm_flags)
    dbusservice.add_path('/Alarms/LowVoltage', 0)
    dbusservice.add_path('/Alarms/HighVoltage', 0)
    dbusservice.add_path('/Alarms/LowSoc', 0)
    dbusservice.add_path('/Alarms/HighTemperature', 0)

    def update():
        if bms.read_frame():
            dbusservice['/Soc']               = bms.soc
            dbusservice['/Dc/0/Voltage']      = round(bms.voltage, 2)
            dbusservice['/Dc/0/Current']      = round(bms.current, 1)
            dbusservice['/Dc/0/Power']        = bms.power
            dbusservice['/Dc/0/Temperature']  = round(bms.temp, 1)
            dbusservice['/Connected']         = 1
        else:
            dbusservice['/Connected'] = 0
        return True

    gobject.timeout_add(800, update)   # ~1.25 Hz – matches your 1-second update()

    logging.info("🚀 TeslaBMS-ESP32 driver started – looking for HEX frames on " + port)
    mainloop = gobject.MainLoop()
    mainloop.run()

if __name__ == "__main__":
    main()