#ifndef ENUMERATION_H
#define ENUMERATION_H

enum class Direction { forward = 0, backward = 1 };

enum class MovingMode { standby = 0, fixedLength = 1, constantSpeed = 2 };

enum class Wheel { left = 0, right = 1 };

enum class AxisEnableStatus { on = 0, off = 1 };

enum class SignalEnableStatus { disable = 0, enable = 1 };

enum class ElectricalLevel { low = 0, high = 1 };

enum class ConnectType { serialPort = 1, ethercat = 2 };

enum class ConnectionStatus { unconnected = -1, connected = 0 };

enum class IoStatus { on = 0, off = 1 };

enum class IoType {
  AxisIoInMsg_PEL = 0,
  AxisIoInMsg_NEL = 1,
  AxisIoInMsg_ORG = 2,
  AxisIoInMsg_EMG = 3,
  AxisIoInMsg_DSTP = 4,
  AxisIoInMsg_ALM = 5,
  AxisIoInMsg_RDY = 6,
  AxisIoInMsg_INP = 7
};

enum class MapIoType {
  AxisIoInPort_PEL = 0,
  AxisIoInPort_NEL = 1,
  AxisIoInPort_ORG = 2,
  AxisIoInPort_ALM = 3,
  AxisIoInPort_RDY = 4,
  AxisIoInPort_INP = 5,
  AxisIoInPort_IO = 6
};

#endif  // ENUMERATION_H
