// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/visualization_msgs/MarkerArray.proto

#include "ros/visualization_msgs/MarkerArray.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace ros {
namespace visualization_msgs {
constexpr MarkerArray::MarkerArray(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : markers_(){}
struct MarkerArrayDefaultTypeInternal {
  constexpr MarkerArrayDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~MarkerArrayDefaultTypeInternal() {}
  union {
    MarkerArray _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT MarkerArrayDefaultTypeInternal _MarkerArray_default_instance_;
}  // namespace visualization_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto = nullptr;

const uint32_t TableStruct_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::MarkerArray, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::MarkerArray, markers_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::visualization_msgs::MarkerArray)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::visualization_msgs::_MarkerArray_default_instance_),
};

const char descriptor_table_protodef_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n(ros/visualization_msgs/MarkerArray.pro"
  "to\022\026ros.visualization_msgs\032#ros/visualiz"
  "ation_msgs/Marker.proto\">\n\013MarkerArray\022/"
  "\n\007markers\030\001 \003(\0132\036.ros.visualization_msgs"
  ".Markerb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto_deps[1] = {
  &::descriptor_table_ros_2fvisualization_5fmsgs_2fMarker_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto = {
  false, false, 175, descriptor_table_protodef_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto, "ros/visualization_msgs/MarkerArray.proto", 
  &descriptor_table_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto_once, descriptor_table_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto::offsets,
  file_level_metadata_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto, file_level_enum_descriptors_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto, file_level_service_descriptors_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto_getter() {
  return &descriptor_table_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto(&descriptor_table_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto);
namespace ros {
namespace visualization_msgs {

// ===================================================================

class MarkerArray::_Internal {
 public:
};

void MarkerArray::clear_markers() {
  markers_.Clear();
}
MarkerArray::MarkerArray(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  markers_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.visualization_msgs.MarkerArray)
}
MarkerArray::MarkerArray(const MarkerArray& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      markers_(from.markers_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:ros.visualization_msgs.MarkerArray)
}

inline void MarkerArray::SharedCtor() {
}

MarkerArray::~MarkerArray() {
  // @@protoc_insertion_point(destructor:ros.visualization_msgs.MarkerArray)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void MarkerArray::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void MarkerArray::ArenaDtor(void* object) {
  MarkerArray* _this = reinterpret_cast< MarkerArray* >(object);
  (void)_this;
}
void MarkerArray::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void MarkerArray::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void MarkerArray::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.visualization_msgs.MarkerArray)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  markers_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* MarkerArray::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // repeated .ros.visualization_msgs.Marker markers = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_markers(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<10>(ptr));
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* MarkerArray::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.visualization_msgs.MarkerArray)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .ros.visualization_msgs.Marker markers = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_markers_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, this->_internal_markers(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.visualization_msgs.MarkerArray)
  return target;
}

size_t MarkerArray::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.visualization_msgs.MarkerArray)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .ros.visualization_msgs.Marker markers = 1;
  total_size += 1UL * this->_internal_markers_size();
  for (const auto& msg : this->markers_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData MarkerArray::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    MarkerArray::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*MarkerArray::GetClassData() const { return &_class_data_; }

void MarkerArray::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<MarkerArray *>(to)->MergeFrom(
      static_cast<const MarkerArray &>(from));
}


void MarkerArray::MergeFrom(const MarkerArray& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.visualization_msgs.MarkerArray)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  markers_.MergeFrom(from.markers_);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void MarkerArray::CopyFrom(const MarkerArray& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.visualization_msgs.MarkerArray)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MarkerArray::IsInitialized() const {
  return true;
}

void MarkerArray::InternalSwap(MarkerArray* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  markers_.InternalSwap(&other->markers_);
}

::PROTOBUF_NAMESPACE_ID::Metadata MarkerArray::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto_getter, &descriptor_table_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto_once,
      file_level_metadata_ros_2fvisualization_5fmsgs_2fMarkerArray_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace visualization_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::visualization_msgs::MarkerArray* Arena::CreateMaybeMessage< ::ros::visualization_msgs::MarkerArray >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::visualization_msgs::MarkerArray >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
