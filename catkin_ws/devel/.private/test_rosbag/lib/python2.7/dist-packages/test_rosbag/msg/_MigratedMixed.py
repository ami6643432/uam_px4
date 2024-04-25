# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from test_rosbag/MigratedMixed.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import test_rosbag.msg

class MigratedMixed(genpy.Message):
  _md5sum = "f5edc5b4f4c64456755b51dacf2f13c8"
  _type = "test_rosbag/MigratedMixed"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header             header
MigratedImplicit   field1 #(34, 16.32, "kjljene", (58.2, "aldfkja 17", 82))
int32              field2 #59

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: test_rosbag/MigratedImplicit
Header  header
MigratedExplicit field4 #(58.2 "aldfkja 17", 82)
string  field3 #"kljene"
float32 field2 #16.32
int32   field1 #34



================================================================================
MSG: test_rosbag/MigratedExplicit
Header  header
float32 field2 #58.2
string  combo_field3 #"aldfkja 17"
int32   field4 #82
"""
  __slots__ = ['header','field1','field2']
  _slot_types = ['std_msgs/Header','test_rosbag/MigratedImplicit','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,field1,field2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MigratedMixed, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.field1 is None:
        self.field1 = test_rosbag.msg.MigratedImplicit()
      if self.field2 is None:
        self.field2 = 0
    else:
      self.header = std_msgs.msg.Header()
      self.field1 = test_rosbag.msg.MigratedImplicit()
      self.field2 = 0

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.field1.header.seq, _x.field1.header.stamp.secs, _x.field1.header.stamp.nsecs))
      _x = self.field1.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.field1.field4.header.seq, _x.field1.field4.header.stamp.secs, _x.field1.field4.header.stamp.nsecs))
      _x = self.field1.field4.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.field1.field4.field2
      buff.write(_get_struct_f().pack(_x))
      _x = self.field1.field4.combo_field3
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.field1.field4.field4
      buff.write(_get_struct_i().pack(_x))
      _x = self.field1.field3
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_f2i().pack(_x.field1.field2, _x.field1.field1, _x.field2))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.field1 is None:
        self.field1 = test_rosbag.msg.MigratedImplicit()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.field1.header.seq, _x.field1.header.stamp.secs, _x.field1.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field1.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.field1.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.field1.field4.header.seq, _x.field1.field4.header.stamp.secs, _x.field1.field4.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field1.field4.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.field1.field4.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.field1.field4.field2,) = _get_struct_f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field1.field4.combo_field3 = str[start:end].decode('utf-8')
      else:
        self.field1.field4.combo_field3 = str[start:end]
      start = end
      end += 4
      (self.field1.field4.field4,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field1.field3 = str[start:end].decode('utf-8')
      else:
        self.field1.field3 = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.field1.field2, _x.field1.field1, _x.field2,) = _get_struct_f2i().unpack(str[start:end])
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.field1.header.seq, _x.field1.header.stamp.secs, _x.field1.header.stamp.nsecs))
      _x = self.field1.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.field1.field4.header.seq, _x.field1.field4.header.stamp.secs, _x.field1.field4.header.stamp.nsecs))
      _x = self.field1.field4.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.field1.field4.field2
      buff.write(_get_struct_f().pack(_x))
      _x = self.field1.field4.combo_field3
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.field1.field4.field4
      buff.write(_get_struct_i().pack(_x))
      _x = self.field1.field3
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_f2i().pack(_x.field1.field2, _x.field1.field1, _x.field2))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.field1 is None:
        self.field1 = test_rosbag.msg.MigratedImplicit()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.field1.header.seq, _x.field1.header.stamp.secs, _x.field1.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field1.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.field1.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.field1.field4.header.seq, _x.field1.field4.header.stamp.secs, _x.field1.field4.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field1.field4.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.field1.field4.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.field1.field4.field2,) = _get_struct_f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field1.field4.combo_field3 = str[start:end].decode('utf-8')
      else:
        self.field1.field4.combo_field3 = str[start:end]
      start = end
      end += 4
      (self.field1.field4.field4,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.field1.field3 = str[start:end].decode('utf-8')
      else:
        self.field1.field3 = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.field1.field2, _x.field1.field1, _x.field2,) = _get_struct_f2i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_f = None
def _get_struct_f():
    global _struct_f
    if _struct_f is None:
        _struct_f = struct.Struct("<f")
    return _struct_f
_struct_f2i = None
def _get_struct_f2i():
    global _struct_f2i
    if _struct_f2i is None:
        _struct_f2i = struct.Struct("<f2i")
    return _struct_f2i
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i