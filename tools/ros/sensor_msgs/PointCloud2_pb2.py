# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ros/sensor_msgs/PointCloud2.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ros.sensor_msgs import PointField_pb2 as ros_dot_sensor__msgs_dot_PointField__pb2
from ros.std_msgs import Header_pb2 as ros_dot_std__msgs_dot_Header__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n!ros/sensor_msgs/PointCloud2.proto\x12\x0fros.sensor_msgs\x1a ros/sensor_msgs/PointField.proto\x1a\x19ros/std_msgs/Header.proto\"\xdb\x01\n\x0bPointCloud2\x12$\n\x06header\x18\x01 \x01(\x0b\x32\x14.ros.std_msgs.Header\x12\x0e\n\x06height\x18\x02 \x01(\r\x12\r\n\x05width\x18\x03 \x01(\r\x12+\n\x06\x66ields\x18\x04 \x03(\x0b\x32\x1b.ros.sensor_msgs.PointField\x12\x14\n\x0cis_bigendian\x18\x05 \x01(\x08\x12\x12\n\npoint_step\x18\x06 \x01(\r\x12\x10\n\x08row_step\x18\x07 \x01(\r\x12\x0c\n\x04\x64\x61ta\x18\x08 \x01(\x0c\x12\x10\n\x08is_dense\x18\t \x01(\x08\x62\x06proto3')



_POINTCLOUD2 = DESCRIPTOR.message_types_by_name['PointCloud2']
PointCloud2 = _reflection.GeneratedProtocolMessageType('PointCloud2', (_message.Message,), {
  'DESCRIPTOR' : _POINTCLOUD2,
  '__module__' : 'ros.sensor_msgs.PointCloud2_pb2'
  # @@protoc_insertion_point(class_scope:ros.sensor_msgs.PointCloud2)
  })
_sym_db.RegisterMessage(PointCloud2)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _POINTCLOUD2._serialized_start=116
  _POINTCLOUD2._serialized_end=335
# @@protoc_insertion_point(module_scope)
