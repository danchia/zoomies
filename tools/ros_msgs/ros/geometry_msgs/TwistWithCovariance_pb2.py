# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ros/geometry_msgs/TwistWithCovariance.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ros.geometry_msgs import Twist_pb2 as ros_dot_geometry__msgs_dot_Twist__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n+ros/geometry_msgs/TwistWithCovariance.proto\x12\x11ros.geometry_msgs\x1a\x1dros/geometry_msgs/Twist.proto\"R\n\x13TwistWithCovariance\x12\'\n\x05twist\x18\x01 \x01(\x0b\x32\x18.ros.geometry_msgs.Twist\x12\x12\n\ncovariance\x18\x02 \x03(\x01\x62\x06proto3')



_TWISTWITHCOVARIANCE = DESCRIPTOR.message_types_by_name['TwistWithCovariance']
TwistWithCovariance = _reflection.GeneratedProtocolMessageType('TwistWithCovariance', (_message.Message,), {
  'DESCRIPTOR' : _TWISTWITHCOVARIANCE,
  '__module__' : 'ros.geometry_msgs.TwistWithCovariance_pb2'
  # @@protoc_insertion_point(class_scope:ros.geometry_msgs.TwistWithCovariance)
  })
_sym_db.RegisterMessage(TwistWithCovariance)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _TWISTWITHCOVARIANCE._serialized_start=97
  _TWISTWITHCOVARIANCE._serialized_end=179
# @@protoc_insertion_point(module_scope)
