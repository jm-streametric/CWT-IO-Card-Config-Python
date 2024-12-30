from dataclasses import dataclass
from Utils import modbusCRC16, intToBytesCapped, sendBytesToIOCard, MODIFY_SETTINGS_FUNCTION_CODE, hexString
from enum import Enum
import serial
from typing import Literal
import tabulate

_NUM_DATA_RANGE_BYTES = 2
_NUM_SETPOINT_BYTES = 2
_NUM_TIMEOUT_BYTES = 1

_DATA_LOW_BYTE_INDEX = 0
_DATA_HIGH_BYTE_INDEX = 2
_4MA_SETPOINT_BYTE_INDEX = 4
_20MA_SETPOINT_BYTE_INDEX = 6
_1V_SETPOINT_BYTE_INDEX = 8
_5V_SETPOINT_BYTE_INDEX = 10
_10V_SETPOINT_BYTE_INDEX = 12
_TYPE_BYTE_INDEX = 14
_TIMEOUT_BYTE_INDEX = 15

_LEN_ANALOG_PIN_DEF = 16
_NUM_PINS_PER_PAYLOAD = 16
_ANALOG_DATA_REQUEST_LENGTH = 8
_CRC_LENGTH = 2
_ANALOG_DATA_RESPONSE_LENGTH = _ANALOG_DATA_REQUEST_LENGTH + (_NUM_PINS_PER_PAYLOAD * _LEN_ANALOG_PIN_DEF) + _CRC_LENGTH

ANALOG_0_15 = bytes((0x00,))
ANALOG_16_31 = bytes((0x10,))
READ_ANALOG_INPUTS_BYTE = bytes((0xA1,))

_ANALOG_FIRST_SEQ = bytes((0x03,))
_ANALOG_SECOND_SEQ = bytes((0x04,))

@dataclass
class AnalogPin:

    class PinType(Enum):
        T_4_20mA_L_0mA = bytes((0x00,))
        T_0_20mA       = bytes((0x01,))
        T_0_5V         = bytes((0x02,))
        T_1_5V         = bytes((0x03,))
        T_0_10V        = bytes((0x04,))
        T_4_20mA_L_4mA = bytes((0x05,))

    dataLow: int
    dataHigh: int
    setpoint4mA: int
    setpoint20mA: int
    setpoint1V: int
    setpoint5V: int
    setpoint10V: int
    pinType: PinType
    timeout: int

    def toBytes(self):
        return \
        intToBytesCapped(_NUM_DATA_RANGE_BYTES, self.dataLow,        "Data Low") + \
        intToBytesCapped(_NUM_DATA_RANGE_BYTES, self.dataHigh,       "Data High") + \
        intToBytesCapped(_NUM_SETPOINT_BYTES,   self.setpoint4mA,    "4mA Setpoint") + \
        intToBytesCapped(_NUM_SETPOINT_BYTES,   self.setpoint20mA,   "20mA Setpoint") + \
        intToBytesCapped(_NUM_SETPOINT_BYTES,   self.setpoint1V,     "1V Setpoint") + \
        intToBytesCapped(_NUM_SETPOINT_BYTES,   self.setpoint5V,     "5V Setpoint") + \
        intToBytesCapped(_NUM_SETPOINT_BYTES,   self.setpoint10V,    "10V Setpoint") + \
        self.pinType.value + \
        intToBytesCapped(_NUM_TIMEOUT_BYTES,    self.timeout,        "Timeout (s)")

    @classmethod
    def from_bytes(cls, controllerData: bytes):

        if len(controllerData) != _LEN_ANALOG_PIN_DEF:
            raise IndexError(f"Cannot parse individual Analog Pin data of length {len(controllerData)}")

        return cls(
            dataLow =       int.from_bytes(controllerData[_DATA_LOW_BYTE_INDEX      : _DATA_LOW_BYTE_INDEX      + _NUM_DATA_RANGE_BYTES], 'little'),
            dataHigh =      int.from_bytes(controllerData[_DATA_HIGH_BYTE_INDEX     : _DATA_HIGH_BYTE_INDEX     + _NUM_DATA_RANGE_BYTES], 'little'),
            setpoint4mA =   int.from_bytes(controllerData[_4MA_SETPOINT_BYTE_INDEX  : _4MA_SETPOINT_BYTE_INDEX  + _NUM_SETPOINT_BYTES]  , 'little'),
            setpoint20mA =  int.from_bytes(controllerData[_20MA_SETPOINT_BYTE_INDEX : _20MA_SETPOINT_BYTE_INDEX + _NUM_SETPOINT_BYTES]  , 'little'),
            setpoint1V =    int.from_bytes(controllerData[_1V_SETPOINT_BYTE_INDEX   : _1V_SETPOINT_BYTE_INDEX   + _NUM_SETPOINT_BYTES]  , 'little'),
            setpoint5V =    int.from_bytes(controllerData[_5V_SETPOINT_BYTE_INDEX   : _5V_SETPOINT_BYTE_INDEX   + _NUM_SETPOINT_BYTES]  , 'little'),
            setpoint10V =   int.from_bytes(controllerData[_10V_SETPOINT_BYTE_INDEX  : _10V_SETPOINT_BYTE_INDEX  + _NUM_SETPOINT_BYTES]  , 'little'),
            pinType =       AnalogPin.PinType(bytes((controllerData[_TYPE_BYTE_INDEX],))),
            timeout =       controllerData[_TIMEOUT_BYTE_INDEX]
            )


def _getPinSettingRequestPayload(deviceModbusAddress: int, pinRange: bytes, sequenceByte: bytes) -> bytes:
    """Get the payload sent to a CWT IO Card to request its config

    Arguments:
        deviceModbusAddress -- Modbus slave address of the CWT IO Card.
        If sending over RS232/RS485, make sure this slave address matches the UART protocol you send the request with

    Returns:
        Config request payload that can be sent to a CWT IO Card, `_SHORT_RESPONSE_LENGTH` bytes long
    """
    idByte = bytes((deviceModbusAddress,))
    infoRequest = idByte + MODIFY_SETTINGS_FUNCTION_CODE + idByte + READ_ANALOG_INPUTS_BYTE + sequenceByte + bytes((0x00,)) + pinRange + bytes((0x00,))
    return infoRequest + modbusCRC16(infoRequest)


def readAnalogSettings(deviceModbusAddress: int, serialPortOrIPAddress: str, baudrate: int = 9600, parity: str = serial.PARITY_NONE, stopbits: Literal[1,2] = 1, pinRange = ANALOG_0_15, ignoreZeroPins: bool = True) -> list[AnalogPin]:
    requestPayload = _getPinSettingRequestPayload(deviceModbusAddress, pinRange, _ANALOG_FIRST_SEQ)
    response = sendBytesToIOCard(requestPayload, _ANALOG_DATA_RESPONSE_LENGTH, serialPortOrIPAddress, baudrate, parity, stopbits)
    analogPinData = response[_ANALOG_DATA_REQUEST_LENGTH : -_CRC_LENGTH]
    
    pinList = []
    for i in range(0, len(analogPinData), _LEN_ANALOG_PIN_DEF):
        currentPinBytes = analogPinData[i:i + _LEN_ANALOG_PIN_DEF]
        if (ignoreZeroPins == False) or any([byte != 0x00 for byte in currentPinBytes]):
            pinList.append(AnalogPin.from_bytes(currentPinBytes))
    return pinList


def printAnalogPinTable(pinList: list[AnalogPin]):
    titles = ["CHANNEL", "Time", "L", "H", "4mA", "20mA", "1V", "5V", "10V", "TYPE"]
    dataLists = []
    for i, ap in enumerate(pinList):
        dataLists.append([
            i,
            ap.timeout,
            ap.dataLow,
            ap.dataHigh,
            ap.setpoint4mA,
            ap.setpoint20mA,
            ap.setpoint1V,
            ap.setpoint5V,
            ap.setpoint10V,
            ap.pinType.name.removeprefix("T_"),
        ])
    print(tabulate.tabulate(dataLists, headers=titles))


def writeAnalogSettings(pinData: list[AnalogPin], deviceModbusAddress: int, serialPortOrIPAddress: str, baudrate: int = 9600, parity: str = serial.PARITY_NONE, stopbits: Literal[1,2] = 1, pinRange = ANALOG_0_15) -> bool:
    if len(pinData) < _NUM_PINS_PER_PAYLOAD:
        print("Not enough pins to send to CWT Card... Padding with existing pin data")
        existingPinData = readAnalogSettings(deviceModbusAddress, serialPortOrIPAddress, baudrate, parity, stopbits, pinRange, ignoreZeroPins=False)
        newPinData = pinData + existingPinData[len(pinData):]
    elif len(pinData) == _NUM_PINS_PER_PAYLOAD:
        newPinData = pinData
    else:
        print(f"Given {len(pinData)} pins to update but maximum is {_NUM_PINS_PER_PAYLOAD}! Truncating pins from end of list...")
        newPinData = pinData[:_NUM_PINS_PER_PAYLOAD]
    
    payloadHeader = _getPinSettingRequestPayload(deviceModbusAddress, pinRange, _ANALOG_SECOND_SEQ)
    payloadSettings = bytes()
    for ap in newPinData:
        payloadSettings += ap.toBytes()
    payload = payloadHeader[:-_CRC_LENGTH] + payloadSettings
    payload += modbusCRC16(payload)

    response = sendBytesToIOCard(payload, _ANALOG_DATA_RESPONSE_LENGTH, serialPortOrIPAddress, baudrate, parity, stopbits)
    
    if response == payloadHeader:
        return True
    else:
        return hexString(response)
