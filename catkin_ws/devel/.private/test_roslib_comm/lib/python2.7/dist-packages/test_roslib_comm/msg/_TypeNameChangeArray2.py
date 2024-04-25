# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from test_roslib_comm/TypeNameChangeArray2.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import test_roslib_comm.msg

class TypeNameChangeArray2(genpy.Message):
  _md5sum = "31a9992534c4d020bfc4045e7dc1a786"
  _type = "test_roslib_comm/TypeNameChangeArray2"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """SameSubMsg2[] a
================================================================================
MSG: test_roslib_comm/SameSubMsg2
int32 a
float32 b
string c
uint64[10] d
float64[] e"""
  __slots__ = ['a']
  _slot_types = ['test_roslib_comm/SameSubMsg2[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       a

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TypeNameChangeArray2, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.a is None:
        self.a = []
    else:
      self.a = []

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
      length = len(self.a)
      buff.write(_struct_I.pack(length))
      for val1 in self.a:
        _x = val1
        buff.write(_get_struct_if().pack(_x.a, _x.b))
        _x = val1.c
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_get_struct_10Q().pack(*val1.d))
        length = len(val1.e)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(struct.pack(pattern, *val1.e))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.a is None:
        self.a = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.a = []
      for i in range(0, length):
        val1 = test_roslib_comm.msg.SameSubMsg2()
        _x = val1
        start = end
        end += 8
        (_x.a, _x.b,) = _get_struct_if().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.c = str[start:end].decode('utf-8')
        else:
          val1.c = str[start:end]
        start = end
        end += 80
        val1.d = _get_struct_10Q().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        end += struct.calcsize(pattern)
        val1.e = struct.unpack(pattern, str[start:end])
        self.a.append(val1)
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
      length = len(self.a)
      buff.write(_struct_I.pack(length))
      for val1 in self.a:
        _x = val1
        buff.write(_get_struct_if().pack(_x.a, _x.b))
        _x = val1.c
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(val1.d.tostring())
        length = len(val1.e)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(val1.e.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.a is None:
        self.a = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.a = []
      for i in range(0, length):
        val1 = test_roslib_comm.msg.SameSubMsg2()
        _x = val1
        start = end
        end += 8
        (_x.a, _x.b,) = _get_struct_if().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.c = str[start:end].decode('utf-8')
        else:
          val1.c = str[start:end]
        start = end
        end += 80
        val1.d = numpy.frombuffer(str[start:end], dtype=numpy.uint64, count=10)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        end += struct.calcsize(pattern)
        val1.e = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
        self.a.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_10Q = None
def _get_struct_10Q():
    global _struct_10Q
    if _struct_10Q is None:
        _struct_10Q = struct.Struct("<10Q")
    return _struct_10Q
_struct_if = None
def _get_struct_if():
    global _struct_if
    if _struct_if is None:
        _struct_if = struct.Struct("<if")
    return _struct_if