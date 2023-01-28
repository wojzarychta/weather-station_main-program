import serial

"""
transmission properties:
baudrate: 9600 
bytesize: 8 bits?
parity: 
stopbits: 1
timeout: 
"""


class UART:
    def __init__(self, port_name: str, baud_rate: int):
        ser = serial.Serial(port_name, baud_rate)
