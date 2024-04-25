# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from test_roslib_comm/FillSimple.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class FillSimple(genpy.Message):
  _md5sum = "da04a60d03fa22f7d301f9bd5f9a08ab"
  _type = "test_roslib_comm/FillSimple"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 i32
string str
int32[] i32_array
bool b"""
  __slots__ = ['i32','str','i32_array','b']
  _slot_types = ['int32','string','int32[]','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       i32,str,i32_array,b

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(FillSimple, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.i32 is None:
        self.i32 = 0
      if self.str is None:
        self.str = ''
      if self.i32_array is None:
        self.i32_array = []
      if self.b is None:
        self.b = False
    else:
      self.i32 = 0
      self.str = ''
      self.i32_array = []
      self.b = False

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
      _x = self.i32
      buff.write(_get_struct_i().pack(_x))
      _x = self.str
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.i32_array)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.i32_array))
      _x = self.b
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (self.i32,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.str = str[start:end].decode('utf-8')
      else:
        self.str = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.i32_array = struct.unpack(pattern, str[start:end])
      start = end
      end += 1
      (self.b,) = _get_struct_B().unpack(str[start:end])
      self.b = bool(self.b)
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
      _x = self.i32
      buff.write(_get_struct_i().pack(_x))
      _x = self.str
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.i32_array)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.i32_array.tostring())
      _x = self.b
      buff.write(_get_struct_B().pack(_x))
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
      start = end
      end += 4
      (self.i32,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.str = str[start:end].decode('utf-8')
      else:
        self.str = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.i32_array = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      start = end
      end += 1
      (self.b,) = _get_struct_B().unpack(str[start:end])
      self.b = bool(self.b)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
