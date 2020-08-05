#ifndef ENUMERATION_H
#define ENUMERATION_H

enum class Direction { forward = 0, backward = 1 };

enum class MovingMode { standby = 0, fixed_length = 1, constant_speed = 2 };

enum class Wheel { left = 0, right = 1 };

enum class AxisEnableStatus { on = 0, off = 1 };

enum class ElectricalLevel { low = 0, high = 1 };

enum class ConnectType { serial_port = 1, ethercat = 2 };

enum class ConnectionStatus { unconnected = -1, connected = 0 };

#endif  // ENUMERATION_H
