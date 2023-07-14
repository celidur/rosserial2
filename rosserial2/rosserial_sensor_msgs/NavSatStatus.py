from ..fuction import *


class NavSatStatus:
    def __init__(self):
        self.status: int = 0
        self.service: int = 0

    def serialize(self, message=None):
        if message is not None:
            self.set(message)
        return serialization_int8(self.status) + serialization_uint16(self.service)

    def deserialize(self, data, offset=0):
        offset, self.status = deserialization_int8(data, offset)
        offset, self.service = deserialization_uint16(data, offset)
        return offset

    def __dict__(self):
        return {'status': self.status, 'service': self.service}

    def set(self, value):
        self.status = value.status
        self.service = value.service

    def __hash__(self):
        return 0x331cdbddfa4bc96ffc3b9ad98900a54c
