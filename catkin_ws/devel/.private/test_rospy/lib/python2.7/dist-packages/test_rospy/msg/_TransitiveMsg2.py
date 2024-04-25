# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from test_rospy/TransitiveMsg2.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import test_rosmaster.msg

class TransitiveMsg2(genpy.Message):
  _md5sum = "eb1fa3c8b51b0e31f74e89c2eecc441e"
  _type = "test_rospy/TransitiveMsg2"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """test_rosmaster/Composite data

================================================================================
MSG: test_rosmaster/Composite
# composite message. required for testing import calculation in generators
CompositeA a
CompositeB b

================================================================================
MSG: test_rosmaster/CompositeA
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: test_rosmaster/CompositeB
# copy of geometry_msgs/Point for testing
float64 x
float64 y
float64 z
"""
  __slots__ = ['data']
  _slot_types = ['test_rosmaster/Composite']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TransitiveMsg2, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.data is None:
        self.data = test_rosmaster.msg.Composite()
    else:
      self.data = test_rosmaster.msg.Composite()

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
      buff.write(_get_struct_7d().pack(_x.data.a.x, _x.data.a.y, _x.data.a.z, _x.data.a.w, _x.data.b.x, _x.data.b.y, _x.data.b.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.data is None:
        self.data = test_rosmaster.msg.Composite()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.data.a.x, _x.data.a.y, _x.data.a.z, _x.data.a.w, _x.data.b.x, _x.data.b.y, _x.data.b.z,) = _get_struct_7d().unpack(str[start:end])
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
      buff.write(_get_struct_7d().pack(_x.data.a.x, _x.data.a.y, _x.data.a.z, _x.data.a.w, _x.data.b.x, _x.data.b.y, _x.data.b.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.data is None:
        self.data = test_rosmaster.msg.Composite()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.data.a.x, _x.data.a.y, _x.data.a.z, _x.data.a.w, _x.data.b.x, _x.data.b.y, _x.data.b.z,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
