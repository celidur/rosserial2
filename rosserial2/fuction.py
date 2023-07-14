import struct

from typing import Tuple


def serialization_int8(data) -> bytes:
    return struct.pack('b', data)


def serialization_int16(data) -> bytes:
    return struct.pack('h', data)


def serialization_int32(data) -> bytes:
    return struct.pack('i', data)


def serialization_int64(data) -> bytes:
    return struct.pack('q', data)


def serialization_uint8(data) -> bytes:
    return struct.pack('B', data)


def serialization_uint16(data) -> bytes:
    return struct.pack('H', data)


def serialization_uint32(data) -> bytes:
    return struct.pack('I', data)


def serialization_uint64(data) -> bytes:
    return struct.pack('Q', data)


def serialization_float32(data) -> bytes:
    return struct.pack('f', data)


def serialization_float64(data) -> bytes:
    return struct.pack('d', data)


def serialization_string(data) -> bytes:
    len_ = len(data)
    return serialization_uint32(len_) + data.encode('utf-8')


def serialization_bool(data) -> bytes:
    return struct.pack('?', data)


def deserialization_int8(data, offset=0) -> Tuple[int, int]:
    return offset + 1, struct.unpack('b', data[offset:offset + 1])[0]


def deserialization_int16(data, offset=0) -> Tuple[int, int]:
    return offset + 2, struct.unpack('h', data[offset:offset + 2])[0]


def deserialization_int32(data, offset=0) -> Tuple[int, int]:
    return offset + 4, struct.unpack('i', data[offset:offset + 4])[0]


def deserialization_int64(data, offset=0) -> Tuple[int, int]:
    return offset + 8, struct.unpack('q', data[offset:offset + 8])[0]


def deserialization_uint8(data, offset=0) -> Tuple[int, int]:
    return offset + 1, struct.unpack('B', data[offset:offset + 1])[0]


def deserialization_uint16(data, offset=0) -> Tuple[int, int]:
    return offset + 2, struct.unpack('H', data[offset:offset + 2])[0]


def deserialization_uint32(data, offset=0) -> Tuple[int, int]:
    return offset + 4, struct.unpack('I', data[offset:offset + 4])[0]


def deserialization_uint64(data, offset=0) -> Tuple[int, int]:
    return offset + 8, struct.unpack('Q', data[offset:offset + 8])[0]


def deserialization_float32(data, offset=0) -> Tuple[int, float]:
    return offset + 4, struct.unpack('f', data[offset:offset + 4])[0]


def deserialization_float64(data, offset=0) -> Tuple[int, float]:
    return offset + 8, struct.unpack('d', data[offset:offset + 8])[0]


def deserialization_string(data, offset=0) -> Tuple[int, str]:
    offset, size = deserialization_uint32(data, offset)
    data = data[offset:offset + size].decode('utf-8')
    return offset + size, data


def deserialization_bool(data, offset=0) -> Tuple[int, bool]:
    return offset + 1, struct.unpack('?', data[offset:offset + 1])[0]
