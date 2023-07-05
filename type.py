import struct


def getRosType(typeName, message_type):
    if typeName == "Int8":
        return typeRosInt8(message_type)
    elif typeName == "Int16":
        return typeRosInt16(message_type)
    elif typeName == "Int32":
        return typeRosInt32(message_type)
    elif typeName == "Int64":
        return typeRosInt64(message_type)
    elif typeName == "UInt8":
        return typeRosUInt8(message_type)
    elif typeName == "UInt16":
        return typeRosUInt16(message_type)
    elif typeName == "UInt32":
        return typeRosUInt32(message_type)
    elif typeName == "UInt64":
        return typeRosUInt64(message_type)
    elif typeName == "Float32":
        return typeRosFloat32(message_type)
    elif typeName == "Float64":
        return typeRosFloat64(message_type)
    elif typeName == "String":
        return typeRosString(message_type)
    elif typeName == "Bool":
        return typeRosBool(message_type)
    else:
        return typeRos(message_type, "", {})


class typeRos:
    def __init__(self, message_type, md5sum, data):
        self._md5sum = md5sum
        self._type = message_type
        self.data = data

    def serialize(self, message):
        return None

    def deserialize(self, data, message):
        return None


class typeRosInt8(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "27ffa0c9c4b8fb8492252bcad9e5c57b", {"data": 0})

    def serialize(self, message):
        return struct.pack('b', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('b', data)[0]
        return message


class typeRosInt16(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "8524586e34fbd7cb1c08c5f5f1ca0e57", {"data": 0})

    def serialize(self, message):
        return struct.pack('h', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('h', data)[0]
        return message


class typeRosInt32(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "da5909fbe378aeaf85e547e830cc1bb7", {"data": 0})

    def serialize(self, message):
        return struct.pack('i', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('i', data)[0]
        return message


class typeRosInt64(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "34add168574510e6e17f5d23ecc077ef", {"data": 0})

    def serialize(self, message):
        return struct.pack('q', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('q', data)[0]
        return message


class typeRosUInt8(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "7c8164229e7d2c17eb95e9231617fdee", {"data": 0})

    def serialize(self, message):
        return struct.pack('B', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('B', data)[0]
        return message


class typeRosUInt16(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "1df79edf208b629fe6b81923a544552d", {"data": 0})

    def serialize(self, message):
        return struct.pack('H', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('H', data)[0]
        return message


class typeRosUInt32(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "304a39449588c7f8ce2df6e8001c5fce", {"data": 0})

    def serialize(self, message):
        return struct.pack('I', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('I', data)[0]
        return message


class typeRosUInt64(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "1b2a79973e8bf53d7b53acb71299cb57", {"data": 0})

    def serialize(self, message):
        return struct.pack('Q', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('Q', data)[0]
        return message


class typeRosFloat32(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "73fcbf46b49191e672908e50842a83d4", {"data": 0})

    def serialize(self, message):
        return struct.pack('f', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('f', data)[0]
        return message


class typeRosFloat64(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "fdb28210bfa9d7c91146260178d9a584", {"data": 0})

    def serialize(self, message):
        return struct.pack('d', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('d', data)[0]
        return message


class typeRosString(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "992ce8a1687cec8c8bd883ec73ca41d1", {"data": ""})

    def serialize(self, message):
        length = len(message.data)
        return struct.pack('I', length) + message.data.encode('utf-8')

    def deserialize(self, data, message):
        message.data = data[4:].decode('utf-8')
        return message


class typeRosBool(typeRos):
    def __init__(self, message_type):
        super().__init__(message_type, "8b94c1b53db61fb6aed406028ad6332a", {"data": False})

    def serialize(self, message):
        return struct.pack('?', message.data)

    def deserialize(self, data, message):
        message.data = struct.unpack('?', data)[0]
        return message
