from .. import rosserial_std_msgs as std_msgs
from .. import rosserial_geometry_msgs as geometry_msgs


class MagneticField:
    def __init__(self):
        self.header = std_msgs.Header()
        self.magnetic_field = geometry_msgs.Vector3()
        self.magnetic_field_covariance = [std_msgs.Float64() for _ in range(9)]

    def serialize(self, message=None):
        if message is None:
            message = self
        bytes_ = self.header.serialize(message.header) + self.magnetic_field.serialize(message.orientation)
        for i in range(9):
            bytes_ += self.magnetic_field_covariance[i].serialize(message.magnetic_field_covariance[i])
        return bytes_

    def deserialize(self, data):
        offset = 0
        offset += self.header.deserialize(data[offset:])
        offset += self.magnetic_field.deserialize(data[offset:])
        for i in range(9):
            offset += self.magnetic_field_covariance[i].deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {'header': self.header.__dict__(), 'magnetic_field': self.magnetic_field.__dict__(),
                'magnetic_field_covariance': [i.data for i in self.magnetic_field_covariance]}

    def __set__(self, instance, value):
        self.header = value.header
        self.magnetic_field = value.magnetic_field
        for i in range(9):
            self.magnetic_field_covariance[i].data = value.magnetic_field_covariance[i]

    def __hash__(self):
        return 0x2f3b0b43eed0c9501de0fa3ff89a45aa
