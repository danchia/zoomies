// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/foxglove_msgs/ImageMarkerArray.proto

#include "ros/foxglove_msgs/ImageMarkerArray.pb.h"

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
namespace foxglove_msgs {
constexpr ImageMarkerArray::ImageMarkerArray(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : markers_(){}
struct ImageMarkerArrayDefaultTypeInternal {
  constexpr ImageMarkerArrayDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~ImageMarkerArrayDefaultTypeInternal() {}
  union {
    ImageMarkerArray _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT ImageMarkerArrayDefaultTypeInternal _ImageMarkerArray_default_instance_;
}  // namespace foxglove_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto = nullptr;

const uint32_t TableStruct_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::foxglove_msgs::ImageMarkerArray, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::foxglove_msgs::ImageMarkerArray, markers_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::foxglove_msgs::ImageMarkerArray)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::foxglove_msgs::_ImageMarkerArray_default_instance_),
};

const char descriptor_table_protodef_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n(ros/foxglove_msgs/ImageMarkerArray.pro"
  "to\022\021ros.foxglove_msgs\032(ros/visualization"
  "_msgs/ImageMarker.proto\"H\n\020ImageMarkerAr"
  "ray\0224\n\007markers\030\001 \003(\0132#.ros.visualization"
  "_msgs.ImageMarkerb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto_deps[1] = {
  &::descriptor_table_ros_2fvisualization_5fmsgs_2fImageMarker_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto = {
  false, false, 185, descriptor_table_protodef_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto, "ros/foxglove_msgs/ImageMarkerArray.proto", 
  &descriptor_table_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto_once, descriptor_table_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto::offsets,
  file_level_metadata_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto, file_level_enum_descriptors_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto, file_level_service_descriptors_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto_getter() {
  return &descriptor_table_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto(&descriptor_table_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto);
namespace ros {
namespace foxglove_msgs {

// ===================================================================

class ImageMarkerArray::_Internal {
 public:
};

void ImageMarkerArray::clear_markers() {
  markers_.Clear();
}
ImageMarkerArray::ImageMarkerArray(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  markers_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.foxglove_msgs.ImageMarkerArray)
}
ImageMarkerArray::ImageMarkerArray(const ImageMarkerArray& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      markers_(from.markers_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:ros.foxglove_msgs.ImageMarkerArray)
}

inline void ImageMarkerArray::SharedCtor() {
}

ImageMarkerArray::~ImageMarkerArray() {
  // @@protoc_insertion_point(destructor:ros.foxglove_msgs.ImageMarkerArray)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void ImageMarkerArray::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void ImageMarkerArray::ArenaDtor(void* object) {
  ImageMarkerArray* _this = reinterpret_cast< ImageMarkerArray* >(object);
  (void)_this;
}
void ImageMarkerArray::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void ImageMarkerArray::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void ImageMarkerArray::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.foxglove_msgs.ImageMarkerArray)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  markers_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* ImageMarkerArray::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // repeated .ros.visualization_msgs.ImageMarker markers = 1;
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

uint8_t* ImageMarkerArray::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.foxglove_msgs.ImageMarkerArray)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .ros.visualization_msgs.ImageMarker markers = 1;
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
  // @@protoc_insertion_point(serialize_to_array_end:ros.foxglove_msgs.ImageMarkerArray)
  return target;
}

size_t ImageMarkerArray::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.foxglove_msgs.ImageMarkerArray)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .ros.visualization_msgs.ImageMarker markers = 1;
  total_size += 1UL * this->_internal_markers_size();
  for (const auto& msg : this->markers_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData ImageMarkerArray::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    ImageMarkerArray::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*ImageMarkerArray::GetClassData() const { return &_class_data_; }

void ImageMarkerArray::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<ImageMarkerArray *>(to)->MergeFrom(
      static_cast<const ImageMarkerArray &>(from));
}


void ImageMarkerArray::MergeFrom(const ImageMarkerArray& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.foxglove_msgs.ImageMarkerArray)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  markers_.MergeFrom(from.markers_);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void ImageMarkerArray::CopyFrom(const ImageMarkerArray& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.foxglove_msgs.ImageMarkerArray)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ImageMarkerArray::IsInitialized() const {
  return true;
}

void ImageMarkerArray::InternalSwap(ImageMarkerArray* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  markers_.InternalSwap(&other->markers_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ImageMarkerArray::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto_getter, &descriptor_table_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto_once,
      file_level_metadata_ros_2ffoxglove_5fmsgs_2fImageMarkerArray_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace foxglove_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::foxglove_msgs::ImageMarkerArray* Arena::CreateMaybeMessage< ::ros::foxglove_msgs::ImageMarkerArray >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::foxglove_msgs::ImageMarkerArray >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>