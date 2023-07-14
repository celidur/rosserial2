from typing import List

from .NavSatStatus import NavSatStatus
from .. import rosserial_std_msgs as std_msgs
from ..fuction import *


class NavSatFix:
    def __init__(self):
        self.header: std_msgs.Header = std_msgs.Header()
        self.status: NavSatStatus = NavSatStatus()
        self.latitude: float = 0
        self.longitude: float = 0
        self.altitude: float = 0
        self.position_covariance: List[float] = [0] * 9
        self.position_covariance_type: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        bytes_ = self.header.serialize() + self.status.serialize() + serialization_float64(
            self.latitude) + serialization_float64(self.longitude) + serialization_float64(self.altitude)
        for i in range(9):
            bytes_ += serialization_float64(self.position_covariance[i])
        bytes_ += serialization_uint8(self.position_covariance_type)
        return bytes_

    def deserialize(self, data, offset=0):
        offset = self.header.deserialize(data, offset)
        offset = self.status.deserialize(data, offset)
        offset, self.latitude = deserialization_float64(data, offset)
        offset, self.longitude = deserialization_float64(data, offset)
        offset, self.altitude = deserialization_float64(data, offset)
        for i in range(9):
            offset, self.position_covariance[i] = deserialization_float64(data, offset)
        offset, self.position_covariance_type = deserialization_uint8(data, offset)
        return offset

    def __dict__(self):
        return {"header": self.header.__dict__(), "status": self.status.__dict__(),
                "latitude": self.latitude, "longitude": self.longitude,
                "altitude": self.altitude, "position_covariance": self.position_covariance,
                "position_covariance_type": self.position_covariance_type}

    def set(self, value):
        self.header.set(value.header)
        self.status.set(value.status)
        self.latitude = value.latitude
        self.longitude = value.longitude
        self.altitude = value.altitude
        self.position_covariance = value.position_covariance
        self.position_covariance_type = value.position_covariance_type

    def __hash__(self):
        return 0x2d3a8cd499b9b4a0249fb98fd05cfa48
