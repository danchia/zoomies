# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ros/sensor_msgs/JointState.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ros.std_msgs import Header_pb2 as ros_dot_std__msgs_dot_Header__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n ros/sensor_msgs/JointState.proto\x12\x0fros.sensor_msgs\x1a\x19ros/std_msgs/Header.proto\"t\n\nJointState\x12$\n\x06header\x18\x01 \x01(\x0b\x32\x14.ros.std_msgs.Header\x12\x0c\n\x04name\x18\x02 \x03(\t\x12\x10\n\x08position\x18\x03 \x03(\x01\x12\x10\n\x08velocity\x18\x04 \x03(\x01\x12\x0e\n\x06\x65\x66\x66ort\x18\x05 \x03(\x01\x62\x06proto3')



_JOINTSTATE = DESCRIPTOR.message_types_by_name['JointState']
JointState = _reflection.GeneratedProtocolMessageType('JointState', (_message.Message,), {
  'DESCRIPTOR' : _JOINTSTATE,
  '__module__' : 'ros.sensor_msgs.JointState_pb2'
  # @@protoc_insertion_point(class_scope:ros.sensor_msgs.JointState)
  })
_sym_db.RegisterMessage(JointState)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _JOINTSTATE._serialized_start=80
  _JOINTSTATE._serialized_end=196
# @@protoc_insertion_point(module_scope)
