import rclpy
import rclpy.logging
import rclpy.qos
import rclpy.qos_event
import threading
from serial import SerialException
import time
import sys
from .SerialClient import SerialClient

_node = None
_logger = None
_on_shutdown = None
_time = None


def _thread_spin_target():
    global _node, _on_shutdown
    rclpy.spin(_node)
    if _on_shutdown:
        _on_shutdown()


def main():
    global _node, _logger, _time
    rclpy.init()
    _node = rclpy.create_node(
        "serial_node",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    _logger = _node.get_logger()
    _time = _node.get_clock()
    rclpy.logging.set_logger_level(_logger.name, rclpy.logging.LoggingSeverity.INFO)
    _thread_spin = threading.Thread(target=_thread_spin_target, daemon=True)
    _thread_spin.start()
    _logger.info("ROS Serial Python Node")

    port = '/dev/ttyUSB0'
    if not _node.has_parameter("port"):
        _node.declare_parameter("port", port)
    port = _node.get_parameter("port")._value

    baud = '500_000'
    if not _node.has_parameter("baud"):
        _node.declare_parameter("baud", baud)
    baud = int(_node.get_parameter("baud")._value)

    while rclpy.ok():
        _logger.info("Connecting to %s at %d baud" % (port, baud))
        try:
            client = SerialClient(port, baud)
            client.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            time.sleep(1.0)
            continue
        except OSError:
            time.sleep(1.0)
            continue
        except:
            _logger.warning("Unexpected Error: %s" % sys.exc_info()[0])
            client.port.close()
            time.sleep(1.0)
            continue
