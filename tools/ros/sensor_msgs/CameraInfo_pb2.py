# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ros/sensor_msgs/CameraInfo.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ros.sensor_msgs import RegionOfInterest_pb2 as ros_dot_sensor__msgs_dot_RegionOfInterest__pb2
from ros.std_msgs import Header_pb2 as ros_dot_std__msgs_dot_Header__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n ros/sensor_msgs/CameraInfo.proto\x12\x0fros.sensor_msgs\x1a&ros/sensor_msgs/RegionOfInterest.proto\x1a\x19ros/std_msgs/Header.proto\"\xed\x01\n\nCameraInfo\x12$\n\x06header\x18\x01 \x01(\x0b\x32\x14.ros.std_msgs.Header\x12\x0e\n\x06height\x18\x02 \x01(\r\x12\r\n\x05width\x18\x03 \x01(\r\x12\x18\n\x10\x64istortion_model\x18\x04 \x01(\t\x12\t\n\x01\x44\x18\x05 \x03(\x01\x12\t\n\x01K\x18\x06 \x03(\x01\x12\t\n\x01R\x18\x07 \x03(\x01\x12\t\n\x01P\x18\x08 \x03(\x01\x12\x11\n\tbinning_x\x18\t \x01(\r\x12\x11\n\tbinning_y\x18\n \x01(\r\x12.\n\x03roi\x18\x0b \x01(\x0b\x32!.ros.sensor_msgs.RegionOfInterestb\x06proto3')



_CAMERAINFO = DESCRIPTOR.message_types_by_name['CameraInfo']
CameraInfo = _reflection.GeneratedProtocolMessageType('CameraInfo', (_message.Message,), {
  'DESCRIPTOR' : _CAMERAINFO,
  '__module__' : 'ros.sensor_msgs.CameraInfo_pb2'
  # @@protoc_insertion_point(class_scope:ros.sensor_msgs.CameraInfo)
  })
_sym_db.RegisterMessage(CameraInfo)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _CAMERAINFO._serialized_start=121
  _CAMERAINFO._serialized_end=358
# @@protoc_insertion_point(module_scope)
