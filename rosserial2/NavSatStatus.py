import std_msgs

class NavSatStatus:
    def __init__(self):
        self.status = std_msgs.Int8()
        self.service = std_msgs.Int16()

    def serialize(self, message=None):
        if message is not None:
            self.satus.data = message.status
            self.service.data = message.service
        return self.satus.serialize() + self.service.serialize()

    def deserialize(self, data):
        offset = 0
        offset += self.status.deserialize(data[offset:])
        offset += self.service.deserialize(data[offset:])
        return offset

    def __dict__(self):
        return {'status': self.status.data, 'service': self.service.data}

    def __set__(self, instance, value):
        self.status.data = value.status
        self.service.data = value.service

    def __hash__(self):
        return 0x331cdbddfa4bc96ffc3b9ad98900a54c
