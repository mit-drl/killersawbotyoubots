"""autogenerated by genpy from fleet_control/InsertToHoleRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import hole_detection.msg
import std_msgs.msg

class InsertToHoleRequest(genpy.Message):
  _md5sum = "1ae607eed7ad1773a455dffa3e4ada13"
  _type = "fleet_control/InsertToHoleRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """hole_detection/Hole hole
std_msgs/Float64 angle
string hole_name

================================================================================
MSG: hole_detection/Hole
bool found
float64 width
geometry_msgs/Point position


================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: std_msgs/Float64
float64 data
"""
  __slots__ = ['hole','angle','hole_name']
  _slot_types = ['hole_detection/Hole','std_msgs/Float64','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       hole,angle,hole_name

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(InsertToHoleRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.hole is None:
        self.hole = hole_detection.msg.Hole()
      if self.angle is None:
        self.angle = std_msgs.msg.Float64()
      if self.hole_name is None:
        self.hole_name = ''
    else:
      self.hole = hole_detection.msg.Hole()
      self.angle = std_msgs.msg.Float64()
      self.hole_name = ''

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
      buff.write(_struct_B5d.pack(_x.hole.found, _x.hole.width, _x.hole.position.x, _x.hole.position.y, _x.hole.position.z, _x.angle.data))
      _x = self.hole_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.hole is None:
        self.hole = hole_detection.msg.Hole()
      if self.angle is None:
        self.angle = std_msgs.msg.Float64()
      end = 0
      _x = self
      start = end
      end += 41
      (_x.hole.found, _x.hole.width, _x.hole.position.x, _x.hole.position.y, _x.hole.position.z, _x.angle.data,) = _struct_B5d.unpack(str[start:end])
      self.hole.found = bool(self.hole.found)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.hole_name = str[start:end].decode('utf-8')
      else:
        self.hole_name = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_B5d.pack(_x.hole.found, _x.hole.width, _x.hole.position.x, _x.hole.position.y, _x.hole.position.z, _x.angle.data))
      _x = self.hole_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.hole is None:
        self.hole = hole_detection.msg.Hole()
      if self.angle is None:
        self.angle = std_msgs.msg.Float64()
      end = 0
      _x = self
      start = end
      end += 41
      (_x.hole.found, _x.hole.width, _x.hole.position.x, _x.hole.position.y, _x.hole.position.z, _x.angle.data,) = _struct_B5d.unpack(str[start:end])
      self.hole.found = bool(self.hole.found)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.hole_name = str[start:end].decode('utf-8')
      else:
        self.hole_name = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B5d = struct.Struct("<B5d")
"""autogenerated by genpy from fleet_control/InsertToHoleResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class InsertToHoleResponse(genpy.Message):
  _md5sum = "358e233cde0c8a8bcfea4ce193f8fc15"
  _type = "fleet_control/InsertToHoleResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool success


"""
  __slots__ = ['success']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       success

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(InsertToHoleResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.success is None:
        self.success = False
    else:
      self.success = False

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
      buff.write(_struct_B.pack(self.success))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.success,) = _struct_B.unpack(str[start:end])
      self.success = bool(self.success)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(_struct_B.pack(self.success))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.success,) = _struct_B.unpack(str[start:end])
      self.success = bool(self.success)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
class InsertToHole(object):
  _type          = 'fleet_control/InsertToHole'
  _md5sum = 'e79a0046ad15ab2c2f4f4971fdf613aa'
  _request_class  = InsertToHoleRequest
  _response_class = InsertToHoleResponse
