# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ros/actionlib_msgs/GoalStatus.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from ros.actionlib_msgs import GoalID_pb2 as ros_dot_actionlib__msgs_dot_GoalID__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n#ros/actionlib_msgs/GoalStatus.proto\x12\x12ros.actionlib_msgs\x1a\x1fros/actionlib_msgs/GoalID.proto\"W\n\nGoalStatus\x12+\n\x07goal_id\x18\x01 \x01(\x0b\x32\x1a.ros.actionlib_msgs.GoalID\x12\x0e\n\x06status\x18\x02 \x01(\x05\x12\x0c\n\x04text\x18\x03 \x01(\tb\x06proto3')



_GOALSTATUS = DESCRIPTOR.message_types_by_name['GoalStatus']
GoalStatus = _reflection.GeneratedProtocolMessageType('GoalStatus', (_message.Message,), {
  'DESCRIPTOR' : _GOALSTATUS,
  '__module__' : 'ros.actionlib_msgs.GoalStatus_pb2'
  # @@protoc_insertion_point(class_scope:ros.actionlib_msgs.GoalStatus)
  })
_sym_db.RegisterMessage(GoalStatus)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _GOALSTATUS._serialized_start=92
  _GOALSTATUS._serialized_end=179
# @@protoc_insertion_point(module_scope)
