from .NavSatStatus import NavSatStatus
from .. import rosserial_std_msgs as std_msgs


class NavSatFix:
    def __init__(self):
        self.header = std_msgs.Header()
        self.status = NavSatStatus()
        self.latitude = std_msgs.Float64()
        self.longitude = std_msgs.Float64()
        self.altitude = std_msgs.Float64()
        self.position_covariance = [std_msgs.Float64() for _ in range(9)]
        self.position_covariance_type = std_msgs.UInt8()

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        bytes_ = self.header.serialize() + self.status.serialize() + self.latitude.serialize() + \
                 self.longitude.serialize() + self.altitude.serialize()
        for i in range(9):
            bytes_ += self.position_covariance[i].serialize()
        bytes_ += self.position_covariance_type.serialize()
        return bytes_

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

    def set(self, value):
        self.header.set(value.header)
        self.status.set(value.status)
        self.latitude.data = value.latitude
        self.longitude.data = value.longitude
        self.altitude.data = value.altitude
        for i in range(9):
            self.position_covariance[i].data = value.position_covariance[i]
        self.position_covariance_type.data = value.position_covariance_type

    def __hash__(self):
        return 0x2d3a8cd499b9b4a0249fb98fd05cfa48
