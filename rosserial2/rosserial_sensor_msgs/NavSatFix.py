from .NavSatStatus import NavSatStatus
from .. import rosserial_std_msgs as std_msgs
import rosserial2 as ros2

# TODO : implement Arrays first in std_msgs
class NavSatFix:
    def __init__(self):
        self.header = std_msgs.Header()
        self.status = NavSatStatus()
        self.latitude = std_msgs.Float64()
        self.longitude = std_msgs.Float64()
        self.altitude = std_msgs.Float64()
        self.position_covariance = [std_msgs.Float64() for i in range(9)]
        self.position_covariance_type = std_msgs.UInt8()

    def serialize(self, message=None):
        if message is None:
            message = self
        return self.header.serialize(message.header) + self.status.serialize(message.status) + self.latitude.serialize(
            message.latitude) + self.longitude.serialize(message.longitude) + self.altitude.serialize(
            message.altitude) + self.position_covariance.serialize(
            message.position_covariance) + self.position_covariance_type.serialize(message.position_covariance_type)

    def deserialize(self, data):
        offset = 0
        offset += self.header.deserialize(data[offset:])
        offset += self.status.deserialize(data[offset:])
        offset += self.latitude.deserialize(data[offset:])
        offset += self.longitude.deserialize(data[offset:])
        offset += self.altitude.deserialize(data[offset:])
        for i in range(9):
            offset += self.position_covariance[i].deserialize(data[offset:])
        offset += self.position_covariance_type.deserialize(data[offset:])

        return offset

    def __dict__(self):
        return {"header": self.header.__dict__(), "status": self.status.__dict__(),
                "latitude": self.latitude.data, "longitude": self.longitude.data,
                "altitude": self.altitude.data, "position_covariance": [i.data for i in self.position_covariance],
                "position_covariance_type": self.position_covariance_type.data}

    def __set__(self, instance, value):
        self.header = value.header
        self.status = value.status
        self.latitude.data = value.latitude
        self.longitude.data = value.longitude
        self.altitude.data = value.altitude
        self.position_covariance = value.position_covariance
        self.position_covariance_type.data = value.position_covariance_type

    def __hash__(self):
        return 0x2d3a8cd499b9b4a0249fb98fd05cfa48
