# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dynamixel_workbench_msgs/PRO.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class PRO(genpy.Message):
  _md5sum = "c0387b4038d0572ab0d83e71bcb8fa5e"
  _type = "dynamixel_workbench_msgs/PRO"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# This message is compatible with control table of Dynamixel PRO L42-10-S300-R
# If you want to specific information about control table, please follow the link (http://emanual.robotis.com/)

uint16 Model_Number
uint8  Firmware_Version
uint8  ID
uint8  Baud_Rate
uint8  Return_Delay_Time
uint8  Operating_Mode
uint32 Moving_Threshold
uint8  Temperature_Limit
uint16 Max_Voltage_Limit
uint16 Min_Voltage_Limit
uint32 Acceleration_Limit
uint16 Torque_Limit
uint32 Velocity_Limit
int32  Max_Position_Limit
int32  Min_Position_Limit
uint8  External_Port_Mode_1
uint8  External_Port_Mode_2
uint8  External_Port_Mode_3
uint8  External_Port_Mode_4
uint8  Shutdown

uint8  Torque_Enable
uint8  LED_RED
uint8  LED_GREEN
uint8  LED_BLUE
uint16 Velocity_I_Gain
uint16 Velocity_P_Gain
uint16 Position_P_Gain
int32  Goal_Position
uint32 Goal_Velocity
uint16 Goal_Torque
int32  Goal_Acceleration
uint8  Moving
int32  Present_Position
uint32 Present_Velocity
uint16 Present_Current
uint16 Present_Input_Voltage
uint8  Present_Temperature
uint8  Registered_Instruction
uint8  Status_Return_Level
uint8  Hardware_Error_Status


"""
  __slots__ = ['Model_Number','Firmware_Version','ID','Baud_Rate','Return_Delay_Time','Operating_Mode','Moving_Threshold','Temperature_Limit','Max_Voltage_Limit','Min_Voltage_Limit','Acceleration_Limit','Torque_Limit','Velocity_Limit','Max_Position_Limit','Min_Position_Limit','External_Port_Mode_1','External_Port_Mode_2','External_Port_Mode_3','External_Port_Mode_4','Shutdown','Torque_Enable','LED_RED','LED_GREEN','LED_BLUE','Velocity_I_Gain','Velocity_P_Gain','Position_P_Gain','Goal_Position','Goal_Velocity','Goal_Torque','Goal_Acceleration','Moving','Present_Position','Present_Velocity','Present_Current','Present_Input_Voltage','Present_Temperature','Registered_Instruction','Status_Return_Level','Hardware_Error_Status']
  _slot_types = ['uint16','uint8','uint8','uint8','uint8','uint8','uint32','uint8','uint16','uint16','uint32','uint16','uint32','int32','int32','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint8','uint16','uint16','uint16','int32','uint32','uint16','int32','uint8','int32','uint32','uint16','uint16','uint8','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       Model_Number,Firmware_Version,ID,Baud_Rate,Return_Delay_Time,Operating_Mode,Moving_Threshold,Temperature_Limit,Max_Voltage_Limit,Min_Voltage_Limit,Acceleration_Limit,Torque_Limit,Velocity_Limit,Max_Position_Limit,Min_Position_Limit,External_Port_Mode_1,External_Port_Mode_2,External_Port_Mode_3,External_Port_Mode_4,Shutdown,Torque_Enable,LED_RED,LED_GREEN,LED_BLUE,Velocity_I_Gain,Velocity_P_Gain,Position_P_Gain,Goal_Position,Goal_Velocity,Goal_Torque,Goal_Acceleration,Moving,Present_Position,Present_Velocity,Present_Current,Present_Input_Voltage,Present_Temperature,Registered_Instruction,Status_Return_Level,Hardware_Error_Status

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PRO, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.Model_Number is None:
        self.Model_Number = 0
      if self.Firmware_Version is None:
        self.Firmware_Version = 0
      if self.ID is None:
        self.ID = 0
      if self.Baud_Rate is None:
        self.Baud_Rate = 0
      if self.Return_Delay_Time is None:
        self.Return_Delay_Time = 0
      if self.Operating_Mode is None:
        self.Operating_Mode = 0
      if self.Moving_Threshold is None:
        self.Moving_Threshold = 0
      if self.Temperature_Limit is None:
        self.Temperature_Limit = 0
      if self.Max_Voltage_Limit is None:
        self.Max_Voltage_Limit = 0
      if self.Min_Voltage_Limit is None:
        self.Min_Voltage_Limit = 0
      if self.Acceleration_Limit is None:
        self.Acceleration_Limit = 0
      if self.Torque_Limit is None:
        self.Torque_Limit = 0
      if self.Velocity_Limit is None:
        self.Velocity_Limit = 0
      if self.Max_Position_Limit is None:
        self.Max_Position_Limit = 0
      if self.Min_Position_Limit is None:
        self.Min_Position_Limit = 0
      if self.External_Port_Mode_1 is None:
        self.External_Port_Mode_1 = 0
      if self.External_Port_Mode_2 is None:
        self.External_Port_Mode_2 = 0
      if self.External_Port_Mode_3 is None:
        self.External_Port_Mode_3 = 0
      if self.External_Port_Mode_4 is None:
        self.External_Port_Mode_4 = 0
      if self.Shutdown is None:
        self.Shutdown = 0
      if self.Torque_Enable is None:
        self.Torque_Enable = 0
      if self.LED_RED is None:
        self.LED_RED = 0
      if self.LED_GREEN is None:
        self.LED_GREEN = 0
      if self.LED_BLUE is None:
        self.LED_BLUE = 0
      if self.Velocity_I_Gain is None:
        self.Velocity_I_Gain = 0
      if self.Velocity_P_Gain is None:
        self.Velocity_P_Gain = 0
      if self.Position_P_Gain is None:
        self.Position_P_Gain = 0
      if self.Goal_Position is None:
        self.Goal_Position = 0
      if self.Goal_Velocity is None:
        self.Goal_Velocity = 0
      if self.Goal_Torque is None:
        self.Goal_Torque = 0
      if self.Goal_Acceleration is None:
        self.Goal_Acceleration = 0
      if self.Moving is None:
        self.Moving = 0
      if self.Present_Position is None:
        self.Present_Position = 0
      if self.Present_Velocity is None:
        self.Present_Velocity = 0
      if self.Present_Current is None:
        self.Present_Current = 0
      if self.Present_Input_Voltage is None:
        self.Present_Input_Voltage = 0
      if self.Present_Temperature is None:
        self.Present_Temperature = 0
      if self.Registered_Instruction is None:
        self.Registered_Instruction = 0
      if self.Status_Return_Level is None:
        self.Status_Return_Level = 0
      if self.Hardware_Error_Status is None:
        self.Hardware_Error_Status = 0
    else:
      self.Model_Number = 0
      self.Firmware_Version = 0
      self.ID = 0
      self.Baud_Rate = 0
      self.Return_Delay_Time = 0
      self.Operating_Mode = 0
      self.Moving_Threshold = 0
      self.Temperature_Limit = 0
      self.Max_Voltage_Limit = 0
      self.Min_Voltage_Limit = 0
      self.Acceleration_Limit = 0
      self.Torque_Limit = 0
      self.Velocity_Limit = 0
      self.Max_Position_Limit = 0
      self.Min_Position_Limit = 0
      self.External_Port_Mode_1 = 0
      self.External_Port_Mode_2 = 0
      self.External_Port_Mode_3 = 0
      self.External_Port_Mode_4 = 0
      self.Shutdown = 0
      self.Torque_Enable = 0
      self.LED_RED = 0
      self.LED_GREEN = 0
      self.LED_BLUE = 0
      self.Velocity_I_Gain = 0
      self.Velocity_P_Gain = 0
      self.Position_P_Gain = 0
      self.Goal_Position = 0
      self.Goal_Velocity = 0
      self.Goal_Torque = 0
      self.Goal_Acceleration = 0
      self.Moving = 0
      self.Present_Position = 0
      self.Present_Velocity = 0
      self.Present_Current = 0
      self.Present_Input_Voltage = 0
      self.Present_Temperature = 0
      self.Registered_Instruction = 0
      self.Status_Return_Level = 0
      self.Hardware_Error_Status = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_H5BIB2HIHI2i9B3HiIHiBiI2H4B().pack(_x.Model_Number, _x.Firmware_Version, _x.ID, _x.Baud_Rate, _x.Return_Delay_Time, _x.Operating_Mode, _x.Moving_Threshold, _x.Temperature_Limit, _x.Max_Voltage_Limit, _x.Min_Voltage_Limit, _x.Acceleration_Limit, _x.Torque_Limit, _x.Velocity_Limit, _x.Max_Position_Limit, _x.Min_Position_Limit, _x.External_Port_Mode_1, _x.External_Port_Mode_2, _x.External_Port_Mode_3, _x.External_Port_Mode_4, _x.Shutdown, _x.Torque_Enable, _x.LED_RED, _x.LED_GREEN, _x.LED_BLUE, _x.Velocity_I_Gain, _x.Velocity_P_Gain, _x.Position_P_Gain, _x.Goal_Position, _x.Goal_Velocity, _x.Goal_Torque, _x.Goal_Acceleration, _x.Moving, _x.Present_Position, _x.Present_Velocity, _x.Present_Current, _x.Present_Input_Voltage, _x.Present_Temperature, _x.Registered_Instruction, _x.Status_Return_Level, _x.Hardware_Error_Status))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 80
      (_x.Model_Number, _x.Firmware_Version, _x.ID, _x.Baud_Rate, _x.Return_Delay_Time, _x.Operating_Mode, _x.Moving_Threshold, _x.Temperature_Limit, _x.Max_Voltage_Limit, _x.Min_Voltage_Limit, _x.Acceleration_Limit, _x.Torque_Limit, _x.Velocity_Limit, _x.Max_Position_Limit, _x.Min_Position_Limit, _x.External_Port_Mode_1, _x.External_Port_Mode_2, _x.External_Port_Mode_3, _x.External_Port_Mode_4, _x.Shutdown, _x.Torque_Enable, _x.LED_RED, _x.LED_GREEN, _x.LED_BLUE, _x.Velocity_I_Gain, _x.Velocity_P_Gain, _x.Position_P_Gain, _x.Goal_Position, _x.Goal_Velocity, _x.Goal_Torque, _x.Goal_Acceleration, _x.Moving, _x.Present_Position, _x.Present_Velocity, _x.Present_Current, _x.Present_Input_Voltage, _x.Present_Temperature, _x.Registered_Instruction, _x.Status_Return_Level, _x.Hardware_Error_Status,) = _get_struct_H5BIB2HIHI2i9B3HiIHiBiI2H4B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_H5BIB2HIHI2i9B3HiIHiBiI2H4B().pack(_x.Model_Number, _x.Firmware_Version, _x.ID, _x.Baud_Rate, _x.Return_Delay_Time, _x.Operating_Mode, _x.Moving_Threshold, _x.Temperature_Limit, _x.Max_Voltage_Limit, _x.Min_Voltage_Limit, _x.Acceleration_Limit, _x.Torque_Limit, _x.Velocity_Limit, _x.Max_Position_Limit, _x.Min_Position_Limit, _x.External_Port_Mode_1, _x.External_Port_Mode_2, _x.External_Port_Mode_3, _x.External_Port_Mode_4, _x.Shutdown, _x.Torque_Enable, _x.LED_RED, _x.LED_GREEN, _x.LED_BLUE, _x.Velocity_I_Gain, _x.Velocity_P_Gain, _x.Position_P_Gain, _x.Goal_Position, _x.Goal_Velocity, _x.Goal_Torque, _x.Goal_Acceleration, _x.Moving, _x.Present_Position, _x.Present_Velocity, _x.Present_Current, _x.Present_Input_Voltage, _x.Present_Temperature, _x.Registered_Instruction, _x.Status_Return_Level, _x.Hardware_Error_Status))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 80
      (_x.Model_Number, _x.Firmware_Version, _x.ID, _x.Baud_Rate, _x.Return_Delay_Time, _x.Operating_Mode, _x.Moving_Threshold, _x.Temperature_Limit, _x.Max_Voltage_Limit, _x.Min_Voltage_Limit, _x.Acceleration_Limit, _x.Torque_Limit, _x.Velocity_Limit, _x.Max_Position_Limit, _x.Min_Position_Limit, _x.External_Port_Mode_1, _x.External_Port_Mode_2, _x.External_Port_Mode_3, _x.External_Port_Mode_4, _x.Shutdown, _x.Torque_Enable, _x.LED_RED, _x.LED_GREEN, _x.LED_BLUE, _x.Velocity_I_Gain, _x.Velocity_P_Gain, _x.Position_P_Gain, _x.Goal_Position, _x.Goal_Velocity, _x.Goal_Torque, _x.Goal_Acceleration, _x.Moving, _x.Present_Position, _x.Present_Velocity, _x.Present_Current, _x.Present_Input_Voltage, _x.Present_Temperature, _x.Registered_Instruction, _x.Status_Return_Level, _x.Hardware_Error_Status,) = _get_struct_H5BIB2HIHI2i9B3HiIHiBiI2H4B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_H5BIB2HIHI2i9B3HiIHiBiI2H4B = None
def _get_struct_H5BIB2HIHI2i9B3HiIHiBiI2H4B():
    global _struct_H5BIB2HIHI2i9B3HiIHiBiI2H4B
    if _struct_H5BIB2HIHI2i9B3HiIHiBiI2H4B is None:
        _struct_H5BIB2HIHI2i9B3HiIHiBiI2H4B = struct.Struct("<H5BIB2HIHI2i9B3HiIHiBiI2H4B")
    return _struct_H5BIB2HIHI2i9B3HiIHiBiI2H4B
