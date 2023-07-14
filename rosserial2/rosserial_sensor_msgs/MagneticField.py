from typing import List

from .. import rosserial_std_msgs as std_msgs
from .. import rosserial_geometry_msgs as geometry_msgs
from ..fuction import *


class MagneticField:
    def __init__(self):
        self.header: std_msgs.Header = std_msgs.Header()
        self.magnetic_field: geometry_msgs.Vector3 = geometry_msgs.Vector3()
        self.magnetic_field_covariance: List[float] = [0] * 9

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        bytes_ = self.header.serialize() + self.magnetic_field.serialize()
        for i in range(9):
            bytes_ += serialization_float64(self.magnetic_field_covariance[i])
        return bytes_

    def deserialize(self, data, offset=0):
        offset = self.header.deserialize(data, offset)
        offset = self.magnetic_field.deserialize(data, offset)
        for i in range(9):
            offset, self.magnetic_field_covariance[i] = deserialization_float64(data, offset)
        return offset

    def __dict__(self):
        return {'header': self.header.__dict__(), 'magnetic_field': self.magnetic_field.__dict__(),
                'magnetic_field_covariance': self.magnetic_field_covariance}

    def set(self, value):
        self.header.set(value.header)
        self.magnetic_field.set(value.magnetic_field)
        self.magnetic_field_covariance = value.magnetic_field_covariance

    def __hash__(self):
        return 0x2f3b0b43eed0c9501de0fa3ff89a45aa
