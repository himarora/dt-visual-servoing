# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ground_projection/GetImageCoordRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class GetImageCoordRequest(genpy.Message):
  _md5sum = "cb06b1906fc6f1f5910a7d2012f835c0"
  _type = "ground_projection/GetImageCoordRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
geometry_msgs/Point gp

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['gp']
  _slot_types = ['geometry_msgs/Point']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       gp

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetImageCoordRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.gp is None:
        self.gp = geometry_msgs.msg.Point()
    else:
      self.gp = geometry_msgs.msg.Point()

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
      buff.write(_get_struct_3d().pack(_x.gp.x, _x.gp.y, _x.gp.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.gp is None:
        self.gp = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.gp.x, _x.gp.y, _x.gp.z,) = _get_struct_3d().unpack(str[start:end])
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
      buff.write(_get_struct_3d().pack(_x.gp.x, _x.gp.y, _x.gp.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.gp is None:
        self.gp = geometry_msgs.msg.Point()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.gp.x, _x.gp.y, _x.gp.z,) = _get_struct_3d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ground_projection/GetImageCoordResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import duckietown_msgs.msg

class GetImageCoordResponse(genpy.Message):
  _md5sum = "7e079b9787496ba75117334836e96c45"
  _type = "ground_projection/GetImageCoordResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """

duckietown_msgs/Vector2D normalized_uv

================================================================================
MSG: duckietown_msgs/Vector2D
float32 x
float32 y
"""
  __slots__ = ['normalized_uv']
  _slot_types = ['duckietown_msgs/Vector2D']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       normalized_uv

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetImageCoordResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.normalized_uv is None:
        self.normalized_uv = duckietown_msgs.msg.Vector2D()
    else:
      self.normalized_uv = duckietown_msgs.msg.Vector2D()

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
      buff.write(_get_struct_2f().pack(_x.normalized_uv.x, _x.normalized_uv.y))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.normalized_uv is None:
        self.normalized_uv = duckietown_msgs.msg.Vector2D()
      end = 0
      _x = self
      start = end
      end += 8
      (_x.normalized_uv.x, _x.normalized_uv.y,) = _get_struct_2f().unpack(str[start:end])
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
      buff.write(_get_struct_2f().pack(_x.normalized_uv.x, _x.normalized_uv.y))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.normalized_uv is None:
        self.normalized_uv = duckietown_msgs.msg.Vector2D()
      end = 0
      _x = self
      start = end
      end += 8
      (_x.normalized_uv.x, _x.normalized_uv.y,) = _get_struct_2f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2f = None
def _get_struct_2f():
    global _struct_2f
    if _struct_2f is None:
        _struct_2f = struct.Struct("<2f")
    return _struct_2f
class GetImageCoord(object):
  _type          = 'ground_projection/GetImageCoord'
  _md5sum = '590f09a60b9d5d3e1b4384278b6e2b2f'
  _request_class  = GetImageCoordRequest
  _response_class = GetImageCoordResponse
