from typing import Literal
from enum import Enum
import serial, socket
from dataclasses import dataclass
from Utils import hexString, intToBytesCapped, ipStringFromBytes, modbusCRC16, ipAddrToBytes
from textwrap import dedent
import re

_LONG_RESPONSE_LENGTH = 66
"""Response length in bytes from CWT IO Card with all config parameters"""
_SHORT_RESPONSE_LENGTH = 10 
"""Response length after writing parameters to CWT IO Card"""
_RS485_SETTINGS_BYTE_INDEX_RANGE = slice(8, 21+1) 
"""Range within long response where RS485 settings are"""
_RS232_SETTINGS_BYTE_INDEX_RANGE = slice(24, 37+1) 
"""Range within long response where RS232 settings are"""
_TCP_SETTINGS_BYTE_INDEX_RANGE = slice(38, 59+1) 
"""Range within long response where TCPIP settings are"""
_SERIAL_INFO_LENGTH = _RS485_SETTINGS_BYTE_INDEX_RANGE.stop - _RS485_SETTINGS_BYTE_INDEX_RANGE.start 
"""Number of bytes in RS485/RS232 settings ranges"""
_TCP_INFO_LENGTH = _TCP_SETTINGS_BYTE_INDEX_RANGE.stop - _TCP_SETTINGS_BYTE_INDEX_RANGE.start 
"""Number of bytes in TCPIP settings ranges"""

#----------------------------------------UART byte indexes/definitions----------------------------------------
_NUM_ADDRESS_BYTES = 1 
_NUM_TIMEOUT_BYTES = 2 
"""Number of bytes used to represent timeout (ms * 10)"""
_NUM_BAUDRATE_BYTES = 3 
_RS232_TYPE_ID = bytes((0x00,))
_RS485_TYPE_ID = bytes((0x01,))

#Within the byte ranges for RS232/RS485, the local indexes of specific parameters are as follows:
_SERIAL_ADDR_BYTE_INDEX = 0
_SERIAL_PROTOCOL_BYTE_INDEX = 1
_SERIAL_DEVICE_TYPE_BYTE_INDEX = 2
_SERIAL_DEVICE_ID_BYTE_INDEXES = (3, 11) 
"""Indexes in the serial param byte arrays where the RS232/485 type ID byte is"""
_SERIAL_BAUD_BYTE_INDEX_RANGE = slice(4, 6+1)
_SERIAL_PARITY_BYTE_INDEX = 8
_SERIAL_STOP_BITS_BYTE_INDEX = 10
_SERIAL_TIMEOUT_BYTE_INDEX_RANGE = slice(12,13+1)

SERIAL_TRY_TIMEOUT_S = 5

#----------------------------------------TCPIP byte indexes/definitions----------------------------------------
_NUM_KEEP_ALIVE_BYTES = 1
_NUM_PORT_BYTES = 2

#Within the byte range for TCP/IP settings, the local indexes of specific parameters are as follows:
_TCP_KEEPALIVE_BYTE_INDEX = 0
_TCP_MAC_ADDR_BYTE_INDEX_RANGE = slice(2,7+1)
_TCP_DEVICE_IP_BYTE_INDEX_RANGE = slice(8, 11+1)
_TCP_PORT_BYTE_INDEX_RANGE = slice(12,13+1)
_TCP_NETMASK_BYTE_INDEX_RANGE = slice(14, 17+1)
_TCP_GATEWAY_BYTE_INDEX_RANGE = slice(18,21+1)


@dataclass
class UARTConfig:
    """Variables and functions to specify a config for the CWT IO Card's RS232 and RS485 parameters"""
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
    """Modbus slave address for the RS232/RS485 controller (The slave address for Modbus TCP is shared with RS485 on CWT IO cards)"""
    baudRate: Literal[1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200] 
    """RS232/RS485 baud rate"""
    parity: Parity 
    """RS232/RS485 parity bit"""
    stopBits: StopBits 
    """RS232/RS485 stop bit(s)"""
    devType: DeviceType 
    """Modbus master/slave device selection"""
    protocol: Protocol 
    """Modbus protocol selection"""
    timeoutMs: int
    """Modbus timeout in milliseconds"""

    serialType: Literal["RS232", "RS485"]

    def toBytes(self) -> bytes:
        """Return the representation of this RS232/RS485 configuration in bytes, in a way that the CWT IO Card can read

        Returns:
            byte representation of this class, `_SERIAL_INFO_LENGTH` bytes long
        """
        bAddr = intToBytesCapped(_NUM_ADDRESS_BYTES, self.address, "Address")
        bBaud = intToBytesCapped(_NUM_BAUDRATE_BYTES, self.baudRate, "Baud Rate")
        bTimeout = intToBytesCapped(_NUM_TIMEOUT_BYTES, int(self.timeoutMs / 10), "Timeout")

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
        """Return the byte which identifies this configuration as either RS232 or RS485 to a CWT IO Card"""
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
        """Create a `UARTConfig` instance from a string of bytes read from a CWT IO Card

        Arguments:
            controllerData -- RS232/RS485 parameter data read from a CWT IO Card, `_SERIAL_INFO_LENGTH` bytes long

        Raises:
            IndexError: If data is of an incorrect length
            LookupError: If there are two different serial ID bytes in the byte string
            ValueError: If the serial identification byte is unknown (not RS232 or RS485)

        Returns:
            `UARTConfig` instance
        """
        if len(controllerData) != _SERIAL_INFO_LENGTH:
            raise IndexError(f"Cannot parse serial config data of length {len(controllerData)}")
        
        serialTypeByte = bytes(set([controllerData[i] for i in _SERIAL_DEVICE_ID_BYTE_INDEXES]))
        if len(serialTypeByte) > 1:
            raise LookupError("Unable to parse serial type identifier bytes")
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
    """Variables and functions to specify a config for the CWT IO Card's TCP/IP parameters"""
    keepAliveSecs: int
    """Number of seconds to keep the TCP connection active without data sent/received"""
    macAddress: bytes
    """CWT IO Card MAC Address"""
    deviceIP: str
    """Modbus TCP IP address"""
    port: int
    """Modbus TCP port"""
    netmask: str
    """Modbus TCP netmask"""
    gatewayIP: str
    """Modbus TCP gateway IP address"""
    
    def toBytes(self) -> bytes:
        """Return the representation of this TCP configuration in bytes, in a way that the CWT IO Card can read

        Returns:
            byte representation of this class, `_TCP_INFO_LENGTH` bytes long
        """
        roundedKeepAlive = round(self.keepAliveSecs / 30, ndigits=None)
        bKeepAlive = intToBytesCapped(_NUM_KEEP_ALIVE_BYTES, roundedKeepAlive, "Keep Alive")
        bPort = intToBytesCapped(_NUM_PORT_BYTES, self.port, "Port", endianness='big')
        bDevIP = ipAddrToBytes(self.deviceIP)
        bNetmask = ipAddrToBytes(self.netmask)
        bGateway = ipAddrToBytes(self.gatewayIP)

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
        """Create a `TCPConfig` instance from a string of bytes read from a CWT IO Card

        Arguments:
            controllerData -- TCP parameter data read from a CWT IO Card, `_TCP_INFO_LENGTH` bytes long

        Raises:
            IndexError: If data is of an incorrect length

        Returns:
            `TCPConfig` instance
        """
        if len(controllerData) != _TCP_INFO_LENGTH:
            raise IndexError(f"Cannot parse TCP/IP config data of length {len(controllerData)}")
        
        keepAliveSecs = 30 * controllerData[_TCP_KEEPALIVE_BYTE_INDEX]
        macAddr = controllerData[_TCP_MAC_ADDR_BYTE_INDEX_RANGE]
        devIP = ipStringFromBytes(controllerData[_TCP_DEVICE_IP_BYTE_INDEX_RANGE])
        port = int.from_bytes(controllerData[_TCP_PORT_BYTE_INDEX_RANGE], 'big', signed=False)
        netmask = ipStringFromBytes(controllerData[_TCP_NETMASK_BYTE_INDEX_RANGE])
        gateway = ipStringFromBytes(controllerData[_TCP_GATEWAY_BYTE_INDEX_RANGE])

        return cls(keepAliveSecs, macAddr, devIP, port, netmask, gateway)


def _parseReply(reply: bytes) -> tuple[UARTConfig, UARTConfig, TCPConfig]:
    """Parse a full CWT IO Card config payload into RS485, RS232, and TCP config objects

    Arguments:
        reply -- CWT IO card response to a config request payload, `_LONG_RESPONSE_LENGTH` bytes long

    Returns:
        tuple of (RS485 `UARTConfig`, RS232 `UARTConfig`, `TCPConfig`)
    """
    if len(reply) != _LONG_RESPONSE_LENGTH:
        print(f"Reply is {len(reply)} bytes long, can only parse replies of length {_LONG_RESPONSE_LENGTH}!")
        return
    return (
        UARTConfig.from_bytes(reply[_RS485_SETTINGS_BYTE_INDEX_RANGE]),
        UARTConfig.from_bytes(reply[_RS232_SETTINGS_BYTE_INDEX_RANGE]),
        TCPConfig.from_bytes(reply[_TCP_SETTINGS_BYTE_INDEX_RANGE])
    )

def _sendBytesToIOCard(data: bytes, responseLength: int, serialPortOrIPAddress: str, baudrate: int = 9600, parity: str = serial.PARITY_NONE, stopbits: Literal[1,2] = 1) -> bytes:
    """Send a byte string to the CWT IO Card over serial or UDP (selected automatically)

    Arguments:
        data -- Byte string to send to the CWT IO Card
        responseLength -- Expected length of response from the CWT IO Card
        serialPortOrIPAddress -- If sending over RS232 or RS485, this is the computer serial port where the CWT IO Card is connected, e.g. '/dev/ttyUSB0' or 'COM5'.
        If sending over UDP, this is ip address:port to reach the CWT card at, e.g. '192.168.1.75:502'

    Keyword Arguments:
        baudrate -- RS232/RS485 baudrate. Ignored if sending over UDP (default: 9600)
        parity -- RS232/RS485 parity. Ignored if sending over UDP (default: serial.PARITY_NONE)
        stopbits -- RS232/RS485 stop bit(s). Ignored if sending over UDP (default: 1)

    Raises:
        ConnectionError: If sending over UDP and the full data payload was unable to be sent

    Returns:
        CWT IO Card's response byte string
    """
    if re.match(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}:\d{1,5}$", serialPortOrIPAddress) is not None: #Just look sorta like an IP address and we'll try a socket connection
        socketAddr = (serialPortOrIPAddress.split(':')[0], int(serialPortOrIPAddress.split(':')[1]))
        sockUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sockUDP.settimeout(float(SERIAL_TRY_TIMEOUT_S))
        sockUDP.connect_ex(socketAddr)
        numBytesSent = sockUDP.send(data)
        if numBytesSent != len(data):
            raise ConnectionError(f"Tried to send {len(data)} bytes but actually sent {numBytesSent} to the IP Address {serialPortOrIPAddress}")
        response = sockUDP.recvfrom(responseLength)[0]
    else:
        with serial.Serial(serialPortOrIPAddress, baudrate, 8, parity, stopbits, timeout=SERIAL_TRY_TIMEOUT_S) as ioCardConnection:
            ioCardConnection.write(data)
            response = ioCardConnection.read(responseLength)
    return response

def _getConfigRequestPayload(deviceModbusAddress: int) -> bytes:
    """Get the payload sent to a CWT IO Card to request its config

    Arguments:
        deviceModbusAddress -- Modbus slave address of the CWT IO Card.
        If sending over RS232/RS485, make sure this slave address matches the UART protocol you send the request with

    Returns:
        Config request payload that can be sent to a CWT IO Card, `_SHORT_RESPONSE_LENGTH` bytes long
    """
    idByte = intToBytesCapped(_NUM_ADDRESS_BYTES, deviceModbusAddress, "Device Modbus Address")
    mbFunctionCode = bytes((0x89,))
    infoRequest = idByte + mbFunctionCode + idByte + bytes([0x01] * 2 + [0x00] * 3)
    return infoRequest + modbusCRC16(infoRequest)

def _getConfigWritePayload(deviceModbusAddress: int, config485: UARTConfig, config232: UARTConfig, configTCP: TCPConfig) -> bytes:
    """Get the payload sent to a CWT IO Card to modify its config

    Arguments:
        deviceModbusAddress -- Modbus slave address of the CWT IO Card.
        If sending over RS232/RS485, make sure this slave address matches the UART protocol you send the request with
        config485 -- Config with parameters you want to write to the CWT IO Card's RS485 controller
        config232 -- Config with parameters you want to write to the CWT IO Card's RS232 controller
        configTCP -- Config with parameters you want to write to the CWT IO Card's TCP/IP controller

    Returns:
        Config write payload that can be sent to a CWT IO Card, `_LONG_RESPONSE_LENGTH` bytes long
    """
    idByte = intToBytesCapped(_NUM_ADDRESS_BYTES, deviceModbusAddress, "Device Modbus Address")
    mbFunctionCode = bytes((0x89,))
    payloadHeader = idByte + mbFunctionCode + idByte + bytes([0x01, 0x02] + [0x00] * 3)
    payload = payloadHeader + config485.toBytes() + bytes([0x00] * 2) + config232.toBytes + bytes((0x01,)) + configTCP.toBytes + bytes((0x01, 0x04, 0x01, 0x00,))
    return payload + modbusCRC16(payload)

def writeSettings(deviceModbusAddress: int, config485: UARTConfig, config232: UARTConfig, configTCP: TCPConfig, serialPortOrIPAddress: str, baudrate: int = 9600, parity: str = serial.PARITY_NONE, stopbits: Literal[1,2] = 1, allowMACChanges: bool = False) -> bool: 
    """Write config settings to a connected CWT IO Card

    Arguments:
        deviceModbusAddress -- Modbus slave address of the CWT IO Card.
        
        config485 -- Config with parameters you want to write to the CWT IO Card's RS485 controller
        
        config232 -- Config with parameters you want to write to the CWT IO Card's RS232 controller
        
        configTCP -- Config with parameters you want to write to the CWT IO Card's TCP/IP controller
        
        serialPortOrIPAddress -- If sending over RS232 or RS485, this is the computer serial port where the CWT IO Card is connected, e.g. '/dev/ttyUSB0' or 'COM5'. If sending over UDP, this is ip address:port to reach the CWT card at, e.g. '192.168.1.75:502'

    Keyword Arguments:
        baudrate -- RS232/RS485 baudrate. Ignored if sending over UDP (default: 9600)
        
        parity -- RS232/RS485 parity. Ignored if sending over UDP (default: serial.PARITY_NONE)
        
        stopbits -- RS232/RS485 stop bit(s). Ignored if sending over UDP (default: 1)
        
        allowMACChanges -- If True, the CWT IO Card's MAC address will take the value of the MAC address in `configTCP`.  If False, read the config from the CWT IO Card before writing and use its current MAC address instead. YOU PROBABLY DON'T WANT TO CHANGE THE DEVICE MAC ADDRESS! (default: False)

    Returns:
        True if the write was successful, False otherwise
    """
    config485.serialType = "RS485"
    config232.serialType = "RS232"
    try:
        if not allowMACChanges:
            readResponse = _sendBytesToIOCard(_getConfigRequestPayload(deviceModbusAddress), _LONG_RESPONSE_LENGTH, serialPortOrIPAddress, baudrate, parity, stopbits)
            readTCP = _parseReply(readResponse)[2]
            configTCP.macAddress = readTCP.macAddress

        configUpdatePayload = _getConfigWritePayload(deviceModbusAddress, config485, config232, configTCP)
        expectedResponse = configUpdatePayload[:8] + modbusCRC16(configUpdatePayload[:8])
        
        writeResponse = _sendBytesToIOCard(configUpdatePayload, _SHORT_RESPONSE_LENGTH, serialPortOrIPAddress, baudrate, parity, stopbits)
    except Exception:
        return False
    return writeResponse == expectedResponse

def readSettings(deviceModbusAddress: int, serialPortOrIPAddress: str, baudrate: int = 9600, parity: str = serial.PARITY_NONE, stopbits: Literal[1,2] = 1) -> tuple[UARTConfig, UARTConfig, TCPConfig]: 
    """Read config settings from a connected CWT IO Card

    Arguments:
        deviceModbusAddress -- Modbus slave address of the CWT IO Card.

        serialPortOrIPAddress -- If sending over RS232 or RS485, this is the computer serial port where the CWT IO Card is connected, e.g. '/dev/ttyUSB0' or 'COM5'. If sending over UDP, this is ip address:port to reach the CWT card at, e.g. '192.168.1.75:502'

    Keyword Arguments:
        baudrate -- RS232/RS485 baudrate. Ignored if sending over UDP (default: 9600)
        
        parity -- RS232/RS485 parity. Ignored if sending over UDP (default: serial.PARITY_NONE)
        
        stopbits -- RS232/RS485 stop bit(s). Ignored if sending over UDP (default: 1)

    Returns:
        tuple of (RS485 `UARTConfig`, RS232 `UARTConfig`, `TCPConfig`)
    """
    bytesBack = _sendBytesToIOCard(_getConfigRequestPayload(deviceModbusAddress), _LONG_RESPONSE_LENGTH, serialPortOrIPAddress, baudrate, parity, stopbits)
    return _parseReply(bytesBack)

DEFAULT_RS485_CONFIG = UARTConfig(1, 9600, UARTConfig.Parity.NONE, UARTConfig.StopBits.ONE, UARTConfig.DeviceType.IO_DEVICE, UARTConfig.Protocol.MODBUS_RTU, 1000, 'RS485')
DEFAULT_RS232_CONFIG = UARTConfig(1, 9600, UARTConfig.Parity.NONE, UARTConfig.StopBits.ONE, UARTConfig.DeviceType.IO_DEVICE, UARTConfig.Protocol.MODBUS_RTU, 1000, 'RS232')
DEFAULT_TCPIP_CONFIG = TCPConfig(30, bytes([0x00] * 6), "192.168.1.75", 502, "255.255.255.0", "192.168.1.1")
