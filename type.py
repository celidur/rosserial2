import struct


def getRosType(typeName, message_type):
    if typeName == "Int8":
        return typeRosInt8(message_type)
    elif typeName == "Int16":
        return typeRosInt16(message_type)
    elif typeName == "Int32":
        return typeRosInt32(message_type)
    elif typeName == "UInt8":
        return typeRosUInt8(message_type)
    elif typeName == "UInt16":
        return typeRosUInt16(message_type)
    else:
        return typeRos(message_type, "", {})


class typeRos:
    def __init__(self, message_type, md5sum, data):
        self._md5sum = md5sum
        self._type = message_type
        self.data = data

    def serialize(self, data):
        pass

    def deserialize(self, data):
        pass


class typeRosInt8(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "27ffa0c9c4b8fb8492252bcad9e5c57b", {"data": 0})

    def serialize(self, data):
        return struct.pack('b', data)

    def deserialize(self, data):
        return struct.unpack('b', data)[0]


class typeRosInt16(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "8524586e34fbd7cb1c08c5f5f1ca0e57", {"data": 0})

    def serialize(self, data):
        return struct.pack('h', data)

    def deserialize(self, data):
        return struct.unpack('h', data)[0]


class typeRosInt32(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "da5909fbe378aeaf85e547e830cc1bb7", {"data": 0})

    def serialize(self, data):
        return struct.pack('i', data)

    def deserialize(self, data):
        return struct.unpack('i', data)[0]


class typeRosUInt8(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "7c8164229e7d2c17eb95e9231617fdee", {"data": 0})

    def serialize(self, data):
        return struct.pack('B', data)

    def deserialize(self, data):
        return struct.unpack('B', data)[0]


class typeRosUInt16(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "1df79edf208b629fe6b81923a544552d", {"data": 0})

    def serialize(self, data):
        return struct.pack('H', data)

    def deserialize(self, data):
        return struct.unpack('H', data)[0]
