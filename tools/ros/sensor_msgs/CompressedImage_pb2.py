# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ros/sensor_msgs/CompressedImage.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ros.std_msgs import Header_pb2 as ros_dot_std__msgs_dot_Header__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n%ros/sensor_msgs/CompressedImage.proto\x12\x0fros.sensor_msgs\x1a\x19ros/std_msgs/Header.proto\"U\n\x0f\x43ompressedImage\x12$\n\x06header\x18\x01 \x01(\x0b\x32\x14.ros.std_msgs.Header\x12\x0e\n\x06\x66ormat\x18\x02 \x01(\t\x12\x0c\n\x04\x64\x61ta\x18\x03 \x01(\x0c\x62\x06proto3')



_COMPRESSEDIMAGE = DESCRIPTOR.message_types_by_name['CompressedImage']
CompressedImage = _reflection.GeneratedProtocolMessageType('CompressedImage', (_message.Message,), {
  'DESCRIPTOR' : _COMPRESSEDIMAGE,
  '__module__' : 'ros.sensor_msgs.CompressedImage_pb2'
  # @@protoc_insertion_point(class_scope:ros.sensor_msgs.CompressedImage)
  })
_sym_db.RegisterMessage(CompressedImage)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _COMPRESSEDIMAGE._serialized_start=85
  _COMPRESSEDIMAGE._serialized_end=170
# @@protoc_insertion_point(module_scope)
