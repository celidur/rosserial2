from .MultiArrayDimension import MultiArrayDimension
from .UInt32 import UInt32
from ..LoadMsg import load_message


class MultiArrayLayout:
    def __init__(self):
        self.dim_length = UInt32()
        self.dim = [MultiArrayDimension() for _ in range(self.dim_length.data)]
        self.data_offset = UInt32()

    def deserialize(self, byte_arr):
        offset = 0
        offset += self.dim_length.deserialize(byte_arr[offset:])
        self.dim = [MultiArrayDimension() for _ in range(self.dim_length.data)]
        for i in range(self.dim_length.data):
            offset += self.dim[i].deserialize(byte_arr[offset:])
        offset += self.data_offset.deserialize(byte_arr[offset:])
        return offset

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        bytes_msg = self.dim_length.serialize()
        for i in range(self.dim_length.data):
            bytes_msg += self.dim[i].serialize()
        bytes_msg += self.data_offset.serialize()
        return bytes_msg

    def __dict__(self):
        dim_type = load_message("std_msgs", "MultiArrayDimension")
        dim = []
        for i in self.dim:
            m = dim_type()
            m.label = i.label.data
            m.size = i.size.data
            m.stride = i.stride.data
            dim.append(m)
        return {'dim': dim,
                'data_offset': self.data_offset.data}

    # TODO: Check if this is correct
    def set(self, value):
        self.dim_length.data = value.dim_length
        self.dim = value.dim
        self.data_offset.data = value.data_offset

    def __hash__(self):
        return 0x0fed2a11c13e11c5571b4e2a995a91a3