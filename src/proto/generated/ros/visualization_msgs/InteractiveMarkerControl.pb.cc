// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/visualization_msgs/InteractiveMarkerControl.proto

#include "ros/visualization_msgs/InteractiveMarkerControl.pb.h"

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
constexpr InteractiveMarkerControl::InteractiveMarkerControl(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : markers_()
  , name_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , description_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , orientation_(nullptr)
  , orientation_mode_(0)
  , interaction_mode_(0)
  , always_visible_(false)
  , independent_marker_orientation_(false){}
struct InteractiveMarkerControlDefaultTypeInternal {
  constexpr InteractiveMarkerControlDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~InteractiveMarkerControlDefaultTypeInternal() {}
  union {
    InteractiveMarkerControl _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT InteractiveMarkerControlDefaultTypeInternal _InteractiveMarkerControl_default_instance_;
}  // namespace visualization_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto = nullptr;

const uint32_t TableStruct_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarkerControl, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarkerControl, name_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarkerControl, orientation_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarkerControl, orientation_mode_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarkerControl, interaction_mode_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarkerControl, always_visible_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarkerControl, markers_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarkerControl, independent_marker_orientation_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarkerControl, description_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::visualization_msgs::InteractiveMarkerControl)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::visualization_msgs::_InteractiveMarkerControl_default_instance_),
};

const char descriptor_table_protodef_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n5ros/visualization_msgs/InteractiveMark"
  "erControl.proto\022\026ros.visualization_msgs\032"
  "\"ros/geometry_msgs/Quaternion.proto\032#ros"
  "/visualization_msgs/Marker.proto\"\226\002\n\030Int"
  "eractiveMarkerControl\022\014\n\004name\030\001 \001(\t\0222\n\013o"
  "rientation\030\002 \001(\0132\035.ros.geometry_msgs.Qua"
  "ternion\022\030\n\020orientation_mode\030\003 \001(\005\022\030\n\020int"
  "eraction_mode\030\004 \001(\005\022\026\n\016always_visible\030\005 "
  "\001(\010\022/\n\007markers\030\006 \003(\0132\036.ros.visualization"
  "_msgs.Marker\022&\n\036independent_marker_orien"
  "tation\030\007 \001(\010\022\023\n\013description\030\010 \001(\tb\006proto"
  "3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto_deps[2] = {
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fQuaternion_2eproto,
  &::descriptor_table_ros_2fvisualization_5fmsgs_2fMarker_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto = {
  false, false, 441, descriptor_table_protodef_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto, "ros/visualization_msgs/InteractiveMarkerControl.proto", 
  &descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto_once, descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto::offsets,
  file_level_metadata_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto, file_level_enum_descriptors_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto, file_level_service_descriptors_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto_getter() {
  return &descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto(&descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto);
namespace ros {
namespace visualization_msgs {

// ===================================================================

class InteractiveMarkerControl::_Internal {
 public:
  static const ::ros::geometry_msgs::Quaternion& orientation(const InteractiveMarkerControl* msg);
};

const ::ros::geometry_msgs::Quaternion&
InteractiveMarkerControl::_Internal::orientation(const InteractiveMarkerControl* msg) {
  return *msg->orientation_;
}
void InteractiveMarkerControl::clear_orientation() {
  if (GetArenaForAllocation() == nullptr && orientation_ != nullptr) {
    delete orientation_;
  }
  orientation_ = nullptr;
}
void InteractiveMarkerControl::clear_markers() {
  markers_.Clear();
}
InteractiveMarkerControl::InteractiveMarkerControl(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  markers_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.visualization_msgs.InteractiveMarkerControl)
}
InteractiveMarkerControl::InteractiveMarkerControl(const InteractiveMarkerControl& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      markers_(from.markers_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    name_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (!from._internal_name().empty()) {
    name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_name(), 
      GetArenaForAllocation());
  }
  description_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    description_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (!from._internal_description().empty()) {
    description_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_description(), 
      GetArenaForAllocation());
  }
  if (from._internal_has_orientation()) {
    orientation_ = new ::ros::geometry_msgs::Quaternion(*from.orientation_);
  } else {
    orientation_ = nullptr;
  }
  ::memcpy(&orientation_mode_, &from.orientation_mode_,
    static_cast<size_t>(reinterpret_cast<char*>(&independent_marker_orientation_) -
    reinterpret_cast<char*>(&orientation_mode_)) + sizeof(independent_marker_orientation_));
  // @@protoc_insertion_point(copy_constructor:ros.visualization_msgs.InteractiveMarkerControl)
}

inline void InteractiveMarkerControl::SharedCtor() {
name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  name_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
description_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  description_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&orientation_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&independent_marker_orientation_) -
    reinterpret_cast<char*>(&orientation_)) + sizeof(independent_marker_orientation_));
}

InteractiveMarkerControl::~InteractiveMarkerControl() {
  // @@protoc_insertion_point(destructor:ros.visualization_msgs.InteractiveMarkerControl)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void InteractiveMarkerControl::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  name_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  description_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete orientation_;
}

void InteractiveMarkerControl::ArenaDtor(void* object) {
  InteractiveMarkerControl* _this = reinterpret_cast< InteractiveMarkerControl* >(object);
  (void)_this;
}
void InteractiveMarkerControl::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void InteractiveMarkerControl::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void InteractiveMarkerControl::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.visualization_msgs.InteractiveMarkerControl)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  markers_.Clear();
  name_.ClearToEmpty();
  description_.ClearToEmpty();
  if (GetArenaForAllocation() == nullptr && orientation_ != nullptr) {
    delete orientation_;
  }
  orientation_ = nullptr;
  ::memset(&orientation_mode_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&independent_marker_orientation_) -
      reinterpret_cast<char*>(&orientation_mode_)) + sizeof(independent_marker_orientation_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* InteractiveMarkerControl::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // string name = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          auto str = _internal_mutable_name();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "ros.visualization_msgs.InteractiveMarkerControl.name"));
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .ros.geometry_msgs.Quaternion orientation = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_orientation(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // int32 orientation_mode = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 24)) {
          orientation_mode_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // int32 interaction_mode = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 32)) {
          interaction_mode_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // bool always_visible = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 40)) {
          always_visible_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .ros.visualization_msgs.Marker markers = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 50)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_markers(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<50>(ptr));
        } else
          goto handle_unusual;
        continue;
      // bool independent_marker_orientation = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 56)) {
          independent_marker_orientation_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // string description = 8;
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 66)) {
          auto str = _internal_mutable_description();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "ros.visualization_msgs.InteractiveMarkerControl.description"));
          CHK_(ptr);
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

uint8_t* InteractiveMarkerControl::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.visualization_msgs.InteractiveMarkerControl)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // string name = 1;
  if (!this->_internal_name().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_name().data(), static_cast<int>(this->_internal_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "ros.visualization_msgs.InteractiveMarkerControl.name");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_name(), target);
  }

  // .ros.geometry_msgs.Quaternion orientation = 2;
  if (this->_internal_has_orientation()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::orientation(this), target, stream);
  }

  // int32 orientation_mode = 3;
  if (this->_internal_orientation_mode() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(3, this->_internal_orientation_mode(), target);
  }

  // int32 interaction_mode = 4;
  if (this->_internal_interaction_mode() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(4, this->_internal_interaction_mode(), target);
  }

  // bool always_visible = 5;
  if (this->_internal_always_visible() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(5, this->_internal_always_visible(), target);
  }

  // repeated .ros.visualization_msgs.Marker markers = 6;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_markers_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(6, this->_internal_markers(i), target, stream);
  }

  // bool independent_marker_orientation = 7;
  if (this->_internal_independent_marker_orientation() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(7, this->_internal_independent_marker_orientation(), target);
  }

  // string description = 8;
  if (!this->_internal_description().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_description().data(), static_cast<int>(this->_internal_description().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "ros.visualization_msgs.InteractiveMarkerControl.description");
    target = stream->WriteStringMaybeAliased(
        8, this->_internal_description(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.visualization_msgs.InteractiveMarkerControl)
  return target;
}

size_t InteractiveMarkerControl::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.visualization_msgs.InteractiveMarkerControl)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .ros.visualization_msgs.Marker markers = 6;
  total_size += 1UL * this->_internal_markers_size();
  for (const auto& msg : this->markers_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // string name = 1;
  if (!this->_internal_name().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_name());
  }

  // string description = 8;
  if (!this->_internal_description().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_description());
  }

  // .ros.geometry_msgs.Quaternion orientation = 2;
  if (this->_internal_has_orientation()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *orientation_);
  }

  // int32 orientation_mode = 3;
  if (this->_internal_orientation_mode() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_orientation_mode());
  }

  // int32 interaction_mode = 4;
  if (this->_internal_interaction_mode() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_interaction_mode());
  }

  // bool always_visible = 5;
  if (this->_internal_always_visible() != 0) {
    total_size += 1 + 1;
  }

  // bool independent_marker_orientation = 7;
  if (this->_internal_independent_marker_orientation() != 0) {
    total_size += 1 + 1;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData InteractiveMarkerControl::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    InteractiveMarkerControl::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*InteractiveMarkerControl::GetClassData() const { return &_class_data_; }

void InteractiveMarkerControl::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<InteractiveMarkerControl *>(to)->MergeFrom(
      static_cast<const InteractiveMarkerControl &>(from));
}


void InteractiveMarkerControl::MergeFrom(const InteractiveMarkerControl& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.visualization_msgs.InteractiveMarkerControl)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  markers_.MergeFrom(from.markers_);
  if (!from._internal_name().empty()) {
    _internal_set_name(from._internal_name());
  }
  if (!from._internal_description().empty()) {
    _internal_set_description(from._internal_description());
  }
  if (from._internal_has_orientation()) {
    _internal_mutable_orientation()->::ros::geometry_msgs::Quaternion::MergeFrom(from._internal_orientation());
  }
  if (from._internal_orientation_mode() != 0) {
    _internal_set_orientation_mode(from._internal_orientation_mode());
  }
  if (from._internal_interaction_mode() != 0) {
    _internal_set_interaction_mode(from._internal_interaction_mode());
  }
  if (from._internal_always_visible() != 0) {
    _internal_set_always_visible(from._internal_always_visible());
  }
  if (from._internal_independent_marker_orientation() != 0) {
    _internal_set_independent_marker_orientation(from._internal_independent_marker_orientation());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void InteractiveMarkerControl::CopyFrom(const InteractiveMarkerControl& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.visualization_msgs.InteractiveMarkerControl)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool InteractiveMarkerControl::IsInitialized() const {
  return true;
}

void InteractiveMarkerControl::InternalSwap(InteractiveMarkerControl* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  markers_.InternalSwap(&other->markers_);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &name_, lhs_arena,
      &other->name_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &description_, lhs_arena,
      &other->description_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(InteractiveMarkerControl, independent_marker_orientation_)
      + sizeof(InteractiveMarkerControl::independent_marker_orientation_)
      - PROTOBUF_FIELD_OFFSET(InteractiveMarkerControl, orientation_)>(
          reinterpret_cast<char*>(&orientation_),
          reinterpret_cast<char*>(&other->orientation_));
}

::PROTOBUF_NAMESPACE_ID::Metadata InteractiveMarkerControl::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto_getter, &descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto_once,
      file_level_metadata_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace visualization_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::visualization_msgs::InteractiveMarkerControl* Arena::CreateMaybeMessage< ::ros::visualization_msgs::InteractiveMarkerControl >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::visualization_msgs::InteractiveMarkerControl >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
