from typing import Literal
from enum import Enum
import serial
from dataclasses import dataclass
from typing import Iterable
from ModTest import hexString
from textwrap import dedent

_CRC_TABLE = [
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
   0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
   0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
   0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
   0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
   0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
   0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
   0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
   0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
   0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
   0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
   0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
   0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
   0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
   0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
   0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
   0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
   0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
   0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
   0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
   0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
   0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
   0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
   0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
   0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
   0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
   0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
   0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
   0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
   0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
   0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
   0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
]

_LONG_RESPONSE_LENGTH = 66
_SHORT_RESPONSE_LENGTH = 10
_RS485_SETTINGS_BYTE_INDEX_RANGE = slice(8, 21+1)
_RS232_SETTINGS_BYTE_INDEX_RANGE = slice(24, 37+1)
_TCP_SETTINGS_BYTE_INDEX_RANGE = slice(38, 59+1)
_SERIAL_INFO_LENGTH = _RS485_SETTINGS_BYTE_INDEX_RANGE.stop - _RS485_SETTINGS_BYTE_INDEX_RANGE.start
_TCP_INFO_LENGTH = _TCP_SETTINGS_BYTE_INDEX_RANGE.stop - _TCP_SETTINGS_BYTE_INDEX_RANGE.start

_NUM_ADDRESS_BYTES = 1
_NUM_TIMEOUT_BYTES = 2
_NUM_BAUDRATE_BYTES = 3
_RS232_TYPE_ID = bytes((0x00,))
_RS485_TYPE_ID = bytes((0x01,))

_NUM_KEEP_ALIVE_BYTES = 1
_NUM_PORT_BYTES = 2

_SERIAL_ADDR_BYTE_INDEX = 0
_SERIAL_PROTOCOL_BYTE_INDEX = 1
_SERIAL_DEVICE_TYPE_BYTE_INDEX = 2
_SERIAL_DEVICE_ID_BYTE_INDEXES = (3, 11) #Indexes in the serial param byte arrays where the RS232/485 identifying byte is
_SERIAL_BAUD_BYTE_INDEX_RANGE = slice(4, 6+1)
_SERIAL_PARITY_BYTE_INDEX = 8
_SERIAL_STOP_BITS_BYTE_INDEX = 10
_SERIAL_TIMEOUT_BYTE_INDEX_RANGE = slice(12,13+1)

_TCP_KEEPALIVE_BYTE_INDEX = 0
_TCP_MAC_ADDR_BYTE_INDEX_RANGE = slice(2,7+1)
_TCP_DEVICE_IP_BYTE_INDEX_RANGE = slice(8, 11+1)
_TCP_PORT_BYTE_INDEX_RANGE = slice(12,13+1)
_TCP_NETMASK_BYTE_INDEX_RANGE = slice(14, 17+1)
_TCP_GATEWAY_BYTE_INDEX_RANGE = slice(18,21+1)

SERIAL_TRY_TIMEOUT_S = 5

def _modbusCRC16(modbusData: Iterable[int]) -> bytes:
    crc = 0xFFFF
    for byte in modbusData:
        crc = (crc >> 8) ^ _CRC_TABLE[((byte) ^ crc) & 0xFF]
    return crc.to_bytes(2, 'little', signed=False)


def _intToBytesCapped(lengthBytes: int, dataSource: int, name: str, endianness: Literal['little', 'big'] = 'little') -> bytes:
        try:
            binaryVal = dataSource.to_bytes(lengthBytes, endianness, signed=False)
        except OverflowError:
            maxVal = int(pow(0xFF, lengthBytes))
            print(f"{name} too large or is negative! Capping to the maximum of {maxVal}.")
            binaryVal = _intToBytesCapped(lengthBytes, maxVal, name)
        return binaryVal


def _ipStringFromBytes(ipBytes: bytes) -> str:
    return '.'.join([str(ipByte) for ipByte in ipBytes])


class CWTCardConfig:
    @dataclass
    class UARTConfig:
        class Parity(Enum):
            NONE = bytes((0x04,))
            ODD = bytes((0x01,))
            EVEN = bytes((0x00,))

        class StopBits(Enum):
            ONE = bytes((0x00,))
            TWO = bytes((0x02,))

        class DeviceType(Enum):
            IO_DEVICE = bytes((0x00,))
            MASTER_DEVICE = bytes((0x01,))

        class Protocol(Enum):
            USER = bytes((0x00,))
            MODBUS_RTU = bytes((0x01,))
            MODBUS_ASCII = bytes((0x02,))
            RS485_TO_NET = bytes((0x03,))
            RTU_TO_TCP = bytes((0x04,))

        address: int
        baudRate: Literal[1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]
        parity: Parity
        stopBits: StopBits
        devType: DeviceType
        protocol: Protocol
        timeoutMs: int
        serialType: Literal["RS232", "RS485"]

        def getData(self) -> bytes:

            bAddr = _intToBytesCapped(_NUM_ADDRESS_BYTES, self.address, "Address")
            bBaud = _intToBytesCapped(_NUM_BAUDRATE_BYTES, self.baudRate, "Baud Rate")
            bTimeout = _intToBytesCapped(_NUM_TIMEOUT_BYTES, int(self.timeoutMs / 10), "Timeout")

            bSerType = self._getSerialTypeByte()

            return  bAddr +\
                    self.protocol.value +\
                    self.devType.value +\
                    bSerType +\
                    bBaud +\
                    bytes((0x00,)) +\
                    self.parity.value +\
                    bytes((0x03,)) +\
                    self.stopBits.value +\
                    bSerType +\
                    bTimeout
        
        def _getSerialTypeByte(self) -> bytes:
            if self.serialType == "RS232":
                return _RS232_TYPE_ID
            else:
                return _RS485_TYPE_ID

        def __str__(self):
            return dedent(f"""\
            CWT {self.serialType} Controller
            Device Type: {self.devType.name}
            Protocol: {self.protocol.name}
            Timeout: {self.timeoutMs}ms
            Address: {self.address}
            Baudrate: {self.baudRate}
            Parity: {self.parity.name}
            Stop Bits: {self.stopBits.name}""")

        @classmethod
        def from_bytes(cls, controllerData: bytes):
            if len(controllerData) != _SERIAL_INFO_LENGTH:
                raise ValueError(f"Cannot parse serial config data of length {len(controllerData)}")
            
            serialTypeByte = bytes(set([controllerData[i] for i in _SERIAL_DEVICE_ID_BYTE_INDEXES]))
            if len(serialTypeByte) > 1:
                raise ValueError("Unable to parse serial type identifier bytes")
            if serialTypeByte == _RS232_TYPE_ID:
                serialType = "RS232"
            elif serialTypeByte == _RS485_TYPE_ID:
                serialType = "RS485"
            else:
                raise ValueError(f"Unknown device identifier byte {serialTypeByte}")
            
            address = controllerData[_SERIAL_ADDR_BYTE_INDEX]
            
            protocol = cls.Protocol(bytes([controllerData[_SERIAL_PROTOCOL_BYTE_INDEX]]))

            devType = cls.DeviceType(bytes([controllerData[_SERIAL_DEVICE_TYPE_BYTE_INDEX]]))
            
            baudBytes = controllerData[_SERIAL_BAUD_BYTE_INDEX_RANGE]
            baudRate = int.from_bytes(baudBytes, 'little', signed=False)

            parity = cls.Parity(bytes([controllerData[_SERIAL_PARITY_BYTE_INDEX]]))

            stopBits = cls.StopBits(bytes([controllerData[_SERIAL_STOP_BITS_BYTE_INDEX]]))

            timeoutBytes = controllerData[_SERIAL_TIMEOUT_BYTE_INDEX_RANGE]
            timeoutMs = 10 * (int.from_bytes(timeoutBytes, 'little', signed=False))

            return cls(address, baudRate, parity, stopBits, devType, protocol, timeoutMs, serialType)

    @dataclass
    class TCPConfig:
        keepAliveSecs: int
        macAddress: bytes
        deviceIP: str
        port: int
        netmask: str
        gatewayIP: str

        def parseIP(self, ip: str) -> bytes:
            ipNums = [int(netByte) for netByte in ip.split('.')]
            if len(ipNums) != 4 or any([(netNum < 0 or netNum > 255) for netNum in ipNums]):
                raise ValueError(f"IP Address {ip} is invalid!")
            return bytes(ipNums)
        
        def getData(self) -> bytes:
            bKeepAlive = _intToBytesCapped(_NUM_KEEP_ALIVE_BYTES, int(self.keepAliveSecs / 30), "Keep Alive")
            bPort = _intToBytesCapped(_NUM_PORT_BYTES, self.port, "Port", endianness='big')
            bDevIP = self.parseIP(self.deviceIP)
            bNetmask = self.parseIP(self.netmask)
            bGateway = self.parseIP(self.gatewayIP)

            return bKeepAlive + self.macAddress + bDevIP + bPort + bNetmask + bGateway
        
        def __str__(self):
            return dedent(f"""\
            CWT TCP/IP Controller
            Keep Alive Time: {self.keepAliveSecs}s
            MAC Address: {hexString(self.macAddress, ':')}
            Device IP: {self.deviceIP}
            Port: {self.port}
            Netmask: {self.netmask}
            Gateway IP: {self.gatewayIP}""")

        @classmethod
        def from_bytes(cls, controllerData: bytes):
            if len(controllerData) != _TCP_INFO_LENGTH:
                raise ValueError(f"Cannot parse TCP/IP config data of length {len(controllerData)}")
            
            keepAliveSecs = 30 * controllerData[_TCP_KEEPALIVE_BYTE_INDEX]
            macAddr = controllerData[_TCP_MAC_ADDR_BYTE_INDEX_RANGE]
            devIP = _ipStringFromBytes(controllerData[_TCP_DEVICE_IP_BYTE_INDEX_RANGE])
            port = int.from_bytes(controllerData[_TCP_PORT_BYTE_INDEX_RANGE], 'big', signed=False)
            netmask = _ipStringFromBytes(controllerData[_TCP_NETMASK_BYTE_INDEX_RANGE])
            gateway = _ipStringFromBytes(controllerData[_TCP_GATEWAY_BYTE_INDEX_RANGE])

            return cls(keepAliveSecs, macAddr, devIP, port, netmask, gateway)


    @staticmethod
    def _parseReply(reply: bytes) -> tuple[UARTConfig, UARTConfig, TCPConfig]:
        if len(reply) != _LONG_RESPONSE_LENGTH:
            print(f"Reply is {len(reply)} bytes long, can only parse replies of length {_LONG_RESPONSE_LENGTH}!")
            return
        return (
            CWTCardConfig.UARTConfig.from_bytes(reply[_RS485_SETTINGS_BYTE_INDEX_RANGE]),
            CWTCardConfig.UARTConfig.from_bytes(reply[_RS232_SETTINGS_BYTE_INDEX_RANGE]),
            CWTCardConfig.TCPConfig.from_bytes(reply[_TCP_SETTINGS_BYTE_INDEX_RANGE])
        )

    @staticmethod
    def _sendBytesToIOCard(data: bytes, responseLength: int, serialPort: str, baudrate: int = 9600, parity: str = serial.PARITY_NONE, stopbits: Literal[1,2] = 1) -> bytes:
        with serial.Serial(serialPort, baudrate, 8, parity, stopbits, timeout=SERIAL_TRY_TIMEOUT_S) as ioCardConnection:
            ioCardConnection.write(data)
            response = ioCardConnection.read(responseLength)
        return response

    @staticmethod
    def _getConfigRequestPayload(deviceModbusAddress: int) -> bytes:
        idByte = _intToBytesCapped(_NUM_ADDRESS_BYTES, deviceModbusAddress, "Device Modbus Address")
        mbFunctionCode = bytes((0x89,))
        infoRequest = idByte + mbFunctionCode + idByte + bytes([0x01] * 2 + [0x00] * 3)
        return infoRequest + _modbusCRC16(infoRequest)

    @staticmethod
    def _getConfigWritePayload(deviceModbusAddress: int, rs485Bytes: bytes, rs232Bytes: bytes, tcpBytes: bytes) -> bytes:
        idByte = _intToBytesCapped(_NUM_ADDRESS_BYTES, deviceModbusAddress, "Device Modbus Address")
        mbFunctionCode = bytes((0x89,))
        payloadHeader = idByte + mbFunctionCode + idByte + bytes([0x01, 0x02] + [0x00] * 3)
        payload = payloadHeader + rs485Bytes + bytes([0x00] * 2) + rs232Bytes + bytes((0x01,)) + tcpBytes + bytes((0x01, 0x04, 0x01, 0x00,))
        return payload + _modbusCRC16(payload)

    @staticmethod
    def writeSettings(config485: UARTConfig, config232: UARTConfig, configTCP: TCPConfig, deviceModbusAddress: int, serialPort: str, baudrate: int = 9600, parity: str = serial.PARITY_NONE, stopbits: Literal[1,2] = 1) -> bool: 
        config485.serialType = "RS485"
        config232.serialType = "RS232"

        readResponse = CWTCardConfig._sendBytesToIOCard(CWTCardConfig._getConfigRequestPayload(deviceModbusAddress), _LONG_RESPONSE_LENGTH, serialPort, baudrate, parity, stopbits)
        readTCP = CWTCardConfig._parseReply(readResponse)[2]
        configTCP.macAddress = readTCP.macAddress

        configUpdatePayload = CWTCardConfig._getConfigWritePayload(deviceModbusAddress, config485.getData(), config232.getData(), configTCP.getData())
        expectedResponse = configUpdatePayload[:8] + _modbusCRC16(configUpdatePayload[:8])
        
        writeResponse = CWTCardConfig._sendBytesToIOCard(configUpdatePayload, _SHORT_RESPONSE_LENGTH, serialPort, baudrate, parity, stopbits)
        return writeResponse == expectedResponse

    @staticmethod
    def readSettings(deviceModbusAddress: int, serialPort: str, baudrate: int = 9600, parity: str = serial.PARITY_NONE, stopbits: Literal[1,2] = 1) -> tuple[UARTConfig, UARTConfig, TCPConfig]: 
        bytesBack = CWTCardConfig._sendBytesToIOCard(CWTCardConfig._getConfigRequestPayload(deviceModbusAddress), _LONG_RESPONSE_LENGTH, serialPort, baudrate, parity, stopbits)
        return CWTCardConfig._parseReply(bytesBack)

if __name__ == "__main__":
    settings485, settings232, settingsTCP = CWTCardConfig.readSettings(254, "/dev/ttyUSB0")
    print(str(settings485) + '\n')
    print(str(settings232) + '\n')
    print(str(settingsTCP))
