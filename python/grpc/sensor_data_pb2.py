# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# NO CHECKED-IN PROTOBUF GENCODE
# source: python/grpc/sensor_data.proto
# Protobuf Python Version: 5.29.0
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import runtime_version as _runtime_version
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
_runtime_version.ValidateProtobufRuntimeVersion(
    _runtime_version.Domain.PUBLIC,
    5,
    29,
    0,
    '',
    'python/grpc/sensor_data.proto'
)
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x1dpython/grpc/sensor_data.proto\x12\ndatastream\"\x07\n\x05\x45mpty\"-\n\tDataPoint\x12\r\n\x05value\x18\x01 \x01(\x01\x12\x11\n\ttimestamp\x18\x02 \x01(\x03\x32H\n\x0c\x44\x61taStreamer\x12\x38\n\nStreamData\x12\x11.datastream.Empty\x1a\x15.datastream.DataPoint0\x01\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'python.grpc.sensor_data_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_EMPTY']._serialized_start=45
  _globals['_EMPTY']._serialized_end=52
  _globals['_DATAPOINT']._serialized_start=54
  _globals['_DATAPOINT']._serialized_end=99
  _globals['_DATASTREAMER']._serialized_start=101
  _globals['_DATASTREAMER']._serialized_end=173
# @@protoc_insertion_point(module_scope)
