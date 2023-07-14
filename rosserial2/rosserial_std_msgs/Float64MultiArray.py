from .MultiArrayLayout import MultiArrayLayout
from .Float64 import Float64
from .UInt32 import UInt32
import rosserial2 as ros2

class Float64MultiArray:
    def __init__(self):
        self.layout = MultiArrayLayout()
        self.data_length = UInt32()
        self.data = []

    def deserialize(self, byte_arr):
        offset = 0
        offset += self.layout.deserialize(byte_arr[offset:])
        ros2._logger.info(str(self.layout.__dict__()))
        for i in self.layout.dim:
            ros2._logger.info(str(self.i.__dict__()))
        offset += self.data_length.deserialize(byte_arr[offset:])
        ros2._logger.info(str(self.data_length.__dict__()))
        self.data = []
        for i in range(self.data_length.data):
            float64 = Float64()
            offset += float64.deserialize(byte_arr[offset:])
            self.data.append(float64.data)
            ros2._logger.info(str(i))
        return offset

    def serialize(self, message=None):
        if message is None:
            message = self
        bytes_msg = self.layout.serialize(message.layout) + self.data_length.serialize(len(message.data))
        for i in range(len(message.data)):
            bytes_msg += self.data[i].serialize(message.data[i])
        return bytes_msg

    def __dict__(self):
        return {'layout': self.layout.__dict__(),
                'data': [f.data for f in self.data]}

    def set(self, value):
        self.layout = value.layout
        self.data_length.data = len(value.data)
        self.data = [Float64() for _ in range(len(value.data))]
        for i in range(len(value.data)):
            self.data[i].data = value.data[i]

    def __hash__(self):
        return 0x4b7d974086d4060e7db4613a7e6c3ba4