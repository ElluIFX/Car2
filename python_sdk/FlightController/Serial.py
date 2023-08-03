import threading
import time
from queue import Queue
from typing import Union

import serial
from loguru import logger


class SerialReader:
    def __init__(self, serial_instance: serial.Serial, startBit):
        """
        初始化串口读取器。

        Args:
            serial_instance (serial.Serial): 串口实例。
            startBit (list): 读取起始位。

        Returns:
            None
        """
        self.data = bytes()
        self._ser = serial_instance
        self._in_waiting_buffer = bytes()
        self._reading_flag = False
        self._pack_length = -1
        self._read_start_bit = bytes(startBit)
        self._read_start_bit_sum = sum(self._read_start_bit) & 0xFF
        try:
            self._ser.set_buffer_size(rx_size=1024 * 1024)  # 1MB
        except:
            pass

    def read(self) -> bool:
        """
        轮询读取串口数据。

        Returns:
            bool: 是否读取到完整的一包数据。
        """
        while self._ser.in_waiting > 0:
            if not self._reading_flag:
                self._in_waiting_buffer += self._ser.read(1)
                if self._in_waiting_buffer[-len(self._read_start_bit) :] == self._read_start_bit:
                    self._reading_flag = True
                    self._pack_length = -1
                    self._in_waiting_buffer = bytes()
            else:
                if self._pack_length == -1:
                    self._pack_length = self._ser.read(1)[0]
                if self._ser.in_waiting >= self._pack_length:
                    data = self._ser.read(self._pack_length)
                    self._reading_flag = False
                    checksum = (sum(data) + self._pack_length + self._read_start_bit_sum) & 0xFF
                    received_checksum = self._ser.read(1)[0]
                    if checksum == received_checksum:
                        self.data = data
                        return True
                    else:
                        logger.warning("[SerialReader] Checksum error")
        return False

    def close(self):
        """
        关闭串口连接。

        Returns:
            None
        """
        if self._ser is not None:
            self._ser.close()
            self._ser = None


class SerialReaderBuffered:
    """
    类似于SerialReader, 但在内部维护一个缓冲区, 以尝试提高读取效率。
    """

    def __init__(self, serial_instance: serial.Serial, startBit):
        """
        初始化串口读取器。

        Args:
            serial_instance (serial.Serial): 串口实例。
            startBit (list): 读取起始位。

        Returns:
            None
        """
        self.data = bytes()
        self._ser = serial_instance
        self._buffer = bytes()
        self._reading_flag = False
        self._pack_length = -1
        self._read_pos = 0
        self._read_start_bit = bytes(startBit)
        self._read_start_bit_sum = sum(self._read_start_bit) & 0xFF
        self._read_start_bit_length = len(self._read_start_bit)
        try:
            self._ser.set_buffer_size(rx_size=1024 * 1024)  # 1MB
        except:
            pass

    def read(self) -> bool:
        """
        轮询读取串口数据。

        Returns:
            bool: 是否读取到完整的一包数据。
        """
        if self._ser.in_waiting > 0 or len(self._buffer) > 0:
            self._buffer += self._ser.read(self._ser.in_waiting)
            if not self._reading_flag:
                idx = self._buffer.find(self._read_start_bit)
                if idx != -1:
                    self._read_pos = idx + self._read_start_bit_length
                    self._reading_flag = True
                    self._pack_length = -1
            if self._reading_flag:
                if self._pack_length == -1 and len(self._buffer) > self._read_pos:
                    self._pack_length = self._buffer[self._read_pos]
                    self._read_pos += 1
                if self._pack_length != -1 and len(self._buffer) > self._read_pos + self._pack_length:
                    self._reading_flag = False
                    data_e = self._buffer[self._read_pos : self._read_pos + self._pack_length + 1]
                    self._buffer = self._buffer[self._read_pos + self._pack_length + 1 :]
                    if (sum(data_e[:-1]) + self._pack_length + self._read_start_bit_sum) & 0xFF == data_e[-1]:
                        self.data = data_e[:-1]
                        return True
                    else:
                        logger.warning("[SerialReader] Checksum error")
        return False

    def close(self):
        """
        关闭串口连接。

        Returns:
            None
        """
        if self._ser is not None:
            self._ser.close()
            self._ser = None


class SerialReaderThreaded:
    """
    多线程的串口读取器
    """

    def __init__(self, serial_instance: serial.Serial, startBit, buffered=True):
        """
        初始化串口读取器。

        Args:
            serial_reader (SerialReader): 串口读取器。

        Returns:
            None
        """
        self._queue: Queue[bytes] = Queue()
        self._serial_reader = (
            SerialReaderBuffered(serial_instance, startBit) if buffered else SerialReader(serial_instance, startBit)
        )
        self._thread_running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _worker(self):
        while self._thread_running:
            if self._serial_reader.read():
                self._queue.put(self._serial_reader.data)
            else:
                time.sleep(0.001)

    def read(self) -> bool:
        """
        是否有数据可读
        """
        return not self._queue.empty()

    @property
    def data(self) -> bytes:
        """
        读取数据(阻塞,一个数据仅能读取一次)
        """
        return self._queue.get()

    def close(self, join=True):
        """
        关闭串口连接。

        Returns:
            None
        """
        self._thread_running = False
        if join:
            self._thread.join()
        self._serial_reader.close()


SerialReaderLike = Union[SerialReader, SerialReaderBuffered, SerialReaderThreaded]
