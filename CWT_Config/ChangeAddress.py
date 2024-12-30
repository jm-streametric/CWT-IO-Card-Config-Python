from typing import Literal
from enum import Enum
import serial, socket
from dataclasses import dataclass
from Utils import hexString, _intToBytesCapped, _ipStringFromBytes, _modbusCRC16
from textwrap import dedent
import re

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
            roundedKeepAlive = round(self.keepAliveSecs / 30, ndigits=None)
            bKeepAlive = _intToBytesCapped(_NUM_KEEP_ALIVE_BYTES, roundedKeepAlive, "Keep Alive")
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
    def _sendBytesToIOCard(data: bytes, responseLength: int, serialPortOrIPAddress: str, baudrate: int = 9600, parity: str = serial.PARITY_NONE, stopbits: Literal[1,2] = 1) -> bytes:
        
        if re.match(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}:\d{1,5}$", serialPortOrIPAddress) is not None: #Just look sorta like an IP address and we'll try a socket connection
            socketAddr = (serialPortOrIPAddress.split(':')[0], int(serialPortOrIPAddress.split(':')[1]))
            sockUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sockUDP.settimeout(float(SERIAL_TRY_TIMEOUT_S))
            sockUDP.connect_ex(socketAddr)
            numBytesSent = sockUDP.send(data)
            if numBytesSent != len(data):
                raise ValueError(f"Tried to send {len(data)} bytes but actually sent {numBytesSent} to the IP Address {serialPortOrIPAddress}")
            response = sockUDP.recvfrom(responseLength)[0]
        else:
            with serial.Serial(serialPortOrIPAddress, baudrate, 8, parity, stopbits, timeout=SERIAL_TRY_TIMEOUT_S) as ioCardConnection:
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
    def writeSettings(config485: UARTConfig, config232: UARTConfig, configTCP: TCPConfig, deviceModbusAddress: int, serialPortOrIPAddress: str, baudrate: int = 9600, parity: str = serial.PARITY_NONE, stopbits: Literal[1,2] = 1, allowMACChanges: bool = False) -> bool: 
        config485.serialType = "RS485"
        config232.serialType = "RS232"

        if not allowMACChanges:
            readResponse = CWTCardConfig._sendBytesToIOCard(CWTCardConfig._getConfigRequestPayload(deviceModbusAddress), _LONG_RESPONSE_LENGTH, serialPortOrIPAddress, baudrate, parity, stopbits)
            readTCP = CWTCardConfig._parseReply(readResponse)[2]
            configTCP.macAddress = readTCP.macAddress

        configUpdatePayload = CWTCardConfig._getConfigWritePayload(deviceModbusAddress, config485.getData(), config232.getData(), configTCP.getData())
        expectedResponse = configUpdatePayload[:8] + _modbusCRC16(configUpdatePayload[:8])
        
        writeResponse = CWTCardConfig._sendBytesToIOCard(configUpdatePayload, _SHORT_RESPONSE_LENGTH, serialPortOrIPAddress, baudrate, parity, stopbits)
        return writeResponse == expectedResponse

    @staticmethod
    def readSettings(deviceModbusAddress: int, serialPortOrIPAddress: str, baudrate: int = 9600, parity: str = serial.PARITY_NONE, stopbits: Literal[1,2] = 1) -> tuple[UARTConfig, UARTConfig, TCPConfig]: 
        bytesBack = CWTCardConfig._sendBytesToIOCard(CWTCardConfig._getConfigRequestPayload(deviceModbusAddress), _LONG_RESPONSE_LENGTH, serialPortOrIPAddress, baudrate, parity, stopbits)
        return CWTCardConfig._parseReply(bytesBack)

DEFAULT_RS485_CONFIG = CWTCardConfig.UARTConfig(1, 9600, CWTCardConfig.UARTConfig.Parity.NONE, CWTCardConfig.UARTConfig.StopBits.ONE, CWTCardConfig.UARTConfig.DeviceType.IO_DEVICE, CWTCardConfig.UARTConfig.Protocol.MODBUS_RTU, 1000, 'RS485')
DEFAULT_RS232_CONFIG = CWTCardConfig.UARTConfig(1, 9600, CWTCardConfig.UARTConfig.Parity.NONE, CWTCardConfig.UARTConfig.StopBits.ONE, CWTCardConfig.UARTConfig.DeviceType.IO_DEVICE, CWTCardConfig.UARTConfig.Protocol.MODBUS_RTU, 1000, 'RS232')
DEFAULT_TCPIP_CONFIG = CWTCardConfig.TCPConfig(30, bytes([0x00] * 6), "192.168.1.75", 502, "255.255.255.0", "192.168.1.1")
