from timeflux.core.node import Node
from threading import Thread, Lock
from time import sleep, time
import queue
import serial
from datetime import datetime

def interpret24bitAsInt32(byte_array):
    new_int = (
        ((0xFF & byte_array[0]) << 16) |
        ((0xFF & byte_array[1]) << 8) |
        (0xFF & byte_array[2])
    )

    if (new_int & 0x00800000) > 0:
        new_int |= 0xFF000000
    else:
        new_int &= 0x00FFFFFF

    # To handle sign extension in Python
    if new_int & (1 << (32 - 1)): # if sign bit is set e.g., 8bit: 128-255
        new_int = new_int - (1 << 32)        # compute negative value

    return new_int

def bytes_to_numbers(buffer):
    numbers = []
    # starts with one since pck num comes too but not used atm
    for i in range(1, len(buffer), 3):
        byte_slice = buffer[i:i+3]
        if len(byte_slice) == 3:  # Make sure we have exactly 3 bytes
            number = interpret24bitAsInt32(list(byte_slice))
            number=number * (((2500000 * 1) / ((2.**24 - 1) * 32))) # to microvolts for freeeg128 change gain/voltage for others
            numbers.append(number)
    return numbers

class FreeEEG(Node):
    """FreeEEG driver.

        Attributes:
            o (Port): Default output, provides DataFrame.

        Args:
            port (string): The serial port.
                e.g. ``COM3`` on Windows;  ``/dev/cu.usbmodem14601`` on MacOS;
                ``/dev/ttyUSB0`` on GNU/Linux.
            device (string): The device Type
                Allowed values: ``FreeEEG16``, ``FreeEEG32``, ``FreeEEG128``
        Example:
            .. literalinclude:: /../examples/simple.yaml
               :language: yaml
        """


    #default for Freeeg128 for now
    def __init__(self, port, device="FreeEEG128", rate=250):
        self.FreeEEG128_chs = "Timestamp,PacketOrder,FP1,FPz,FP2,AFp1,AFPz,AFp2,AF7,AF3,AF4,AF8,AFF5h,AFF1h,AFF2h,AFF6h,F9,F7,F5,F3,F1,Fz,F2,F4,F6,F8,F10,FFT9h,FFT7h,FFC5h,FFC3h,FFC1h,FFC2h,FFC4h,FFC6h,FFT8h,FFT10h,FT9,FT7,FC5,FC3,FC1,FCz,FC2,FC4,FC6,FT8,FT10,FTT9h,FTT7h,FCC5h,FCC3h,FCC1h,FCC2h,FCC4h,FCC6h,FTT8h,FTT10h,T7,C5,C3,C1,Cz,C2,C4,C6,T8,TTP7h,CCP5h,CCP3h,CCP1h,CCP2h,CCP4h,CCP6h,TTP8h,TP9,TP7,CP5,CP3,Cpz,CP4,CP6,TP8,TP10,TPP9h,TPP7h,CPP5h,CPP3h,CPP1h,CPP2h,CPP4h,CPP6h,TPP8h,TPP10h,P9,P7,P5,P3,P1,Pz,P2,P4,P6,P8,P10,PPO9h,PPO5h,PPO1h,PPO2h,PPO6h,PPO10h,PO9,PO7,PO3,POz,PO4,PO8,PO10,POO9h,POO1,POO2,POO10h,O1,Oz,O2,OI1h,OI2h,I1,Iz,I2"
        self.names= self.FreeEEG128_chs.split(",")

        self.ser = serial.Serial()
        self.ser.baudrate = 12000000
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.port = port
        self.ser.open()
        print(self.ser)
        self.BUFFSIZE=2048
        self.numchs=128
        self.buffer = bytearray()  # Now a bytearray, not bytes
        self.sequences = queue.Queue()  # Thread-safe queue for storing sequences
        # Set meta
        self.meta = {"rate": rate}
        self.lastpck=-1
        self._missed = 0
        # Launch background thread
        self._reset()
        self._lock = Lock()
        self._running = True
        self._thread = Thread(target=self._loop).start()

    def _reset(self):
        """Empty cache."""
        self._rows = []
        self._timestamps = []

    def _loop(self):
        """Acquire and cache data."""

        while self._running:
            try:
                data = self.ser.read(self.BUFFSIZE)
                self.buffer.extend(data)
                bksize = 3+self.numchs*3 #3 bytes + ch + 3 start min
                while True:  # Keep searching in the current buffer
                    start_index = self.buffer.find(b'\xC0\xA0')
                    if start_index == -1:  # If the start sequence is not found, stop searching
                        break
                    # If there are enough bytes after the start sequence
                    if len(self.buffer) >= start_index + 2 + bksize:
                        blk = self.buffer[start_index: start_index + 2 + bksize]                        
                        eegdata = bytes_to_numbers(blk[2:3 + (3*self.numchs)])
                        self._lock.acquire()  # `with self.lock:` is about twice as slow
                        self.lastpck=(blk[3] & 0xff) # not implemented yet but this packet count from 0-255 from freeeg
                        self._timestamps.append(datetime.now().timestamp())
                        self._rows.append(eegdata)
                        self._lock.release()                        
                        del self.buffer[:start_index + 2 + bksize]  # Remove the processed data from the buffer
                    else:
                        break  # Not enough data for a full sequence, stop searching

            except:
                pass
    def update(self):
        """Update the node output."""
        with self._lock:
            if self._rows:
                self.o.set(self._rows, self._timestamps, self.names, meta=self.meta)
                self._reset()
    def terminate(self):
        """Cleanup. and close Freeeg comm port"""
        self._running = False
        while self._thread and self._thread.is_alive():
            sleep(0.001)
        try:
            self.ser.close()
        except:
            pass
