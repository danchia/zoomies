# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ros/std_msgs/Header.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ros import builtins_pb2 as ros_dot_builtins__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x19ros/std_msgs/Header.proto\x12\x0cros.std_msgs\x1a\x12ros/builtins.proto\"A\n\x06Header\x12\x0b\n\x03seq\x18\x01 \x01(\r\x12\x18\n\x05stamp\x18\x02 \x01(\x0b\x32\t.ros.Time\x12\x10\n\x08\x66rame_id\x18\x03 \x01(\tb\x06proto3')



_HEADER = DESCRIPTOR.message_types_by_name['Header']
Header = _reflection.GeneratedProtocolMessageType('Header', (_message.Message,), {
  'DESCRIPTOR' : _HEADER,
  '__module__' : 'ros.std_msgs.Header_pb2'
  # @@protoc_insertion_point(class_scope:ros.std_msgs.Header)
  })
_sym_db.RegisterMessage(Header)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _HEADER._serialized_start=63
  _HEADER._serialized_end=128
# @@protoc_insertion_point(module_scope)
