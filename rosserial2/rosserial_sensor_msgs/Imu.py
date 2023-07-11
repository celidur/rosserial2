from .. import rosserial_std_msgs as std_msgs
from .. import rosserial_geometry_msgs as geometry_msgs


class Imu:
    def __init__(self):
        self.header = std_msgs.Header()
        self.orientation = geometry_msgs.Quaternion()
        self.orientation_covariance = [std_msgs.Float64() for _ in range(9)]
        self.angular_velocity = geometry_msgs.Vector3()
        self.linear_acceleration = geometry_msgs.Vector3()
        self.linear_acceleration_covariance = [std_msgs.Float64() for _ in range(9)]

    def serialize(self, message=None):
        if message is None:
            message = self
        bytes_ = self.header.serialize(message.header) + self.orientation.serialize(message.orientation)
        for i in range(9):
            bytes_ += self.orientation_covariance[i].serialize(message.orientation_covariance[i])
        bytes_ += self.angular_velocity.serialize(message.angular_velocity)
        bytes_ += self.linear_acceleration.serialize(message.linear_acceleration)
        for i in range(9):
            bytes_ += self.linear_acceleration_covariance[i].serialize(message.linear_acceleration_covariance[i])
        return bytes_

    def deserialize(self, data):
        offset = 0
        offset += self.header.deserialize(data[offset:])
        offset += self.orientation.deserialize(data[offset:])
        for i in range(9):
            offset += self.orientation_covariance[i].deserialize(data[offset:])
        offset += self.angular_velocity.deserialize(data[offset:])
        offset += self.linear_acceleration.deserialize(data[offset:])
        for i in range(9):
            offset += self.linear_acceleration_covariance[i].deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {'header': self.header.__dict__(), 'orientation': self.orientation.__dict__(),
                'orientation_covariance': [i.data for i in self.orientation_covariance],
                'angular_velocity': self.angular_velocity.__dict__(),
                'linear_acceleration': self.linear_acceleration.__dict__(),
                'linear_acceleration_covariance': [i.data for i in self.linear_acceleration_covariance]}

    def __set__(self, instance, value):
        self.header = value.header
        self.orientation = value.orientation
        for i in range(9):
            self.orientation_covariance[i].data = value.orientation_covariance[i]
        self.angular_velocity = value.angular_velocity
        self.linear_acceleration = value.linear_acceleration
        for i in range(9):
            self.linear_acceleration_covariance[i].data = value.linear_acceleration_covariance[i]

    def __hash__(self):
        return 0x6a62c6daae103f4ff57a132d6f95cec2
