// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/visualization_msgs/InteractiveMarker.proto

#include "ros/visualization_msgs/InteractiveMarker.pb.h"

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
constexpr InteractiveMarker::InteractiveMarker(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : menu_entries_()
  , controls_()
  , name_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , description_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , header_(nullptr)
  , pose_(nullptr)
  , scale_(0){}
struct InteractiveMarkerDefaultTypeInternal {
  constexpr InteractiveMarkerDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~InteractiveMarkerDefaultTypeInternal() {}
  union {
    InteractiveMarker _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT InteractiveMarkerDefaultTypeInternal _InteractiveMarker_default_instance_;
}  // namespace visualization_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto = nullptr;

const uint32_t TableStruct_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarker, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarker, header_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarker, pose_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarker, name_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarker, description_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarker, scale_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarker, menu_entries_),
  PROTOBUF_FIELD_OFFSET(::ros::visualization_msgs::InteractiveMarker, controls_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::visualization_msgs::InteractiveMarker)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::visualization_msgs::_InteractiveMarker_default_instance_),
};

const char descriptor_table_protodef_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n.ros/visualization_msgs/InteractiveMark"
  "er.proto\022\026ros.visualization_msgs\032\034ros/ge"
  "ometry_msgs/Pose.proto\032\031ros/std_msgs/Hea"
  "der.proto\0325ros/visualization_msgs/Intera"
  "ctiveMarkerControl.proto\032&ros/visualizat"
  "ion_msgs/MenuEntry.proto\"\217\002\n\021Interactive"
  "Marker\022$\n\006header\030\001 \001(\0132\024.ros.std_msgs.He"
  "ader\022%\n\004pose\030\002 \001(\0132\027.ros.geometry_msgs.P"
  "ose\022\014\n\004name\030\003 \001(\t\022\023\n\013description\030\004 \001(\t\022\r"
  "\n\005scale\030\005 \001(\002\0227\n\014menu_entries\030\006 \003(\0132!.ro"
  "s.visualization_msgs.MenuEntry\022B\n\010contro"
  "ls\030\007 \003(\01320.ros.visualization_msgs.Intera"
  "ctiveMarkerControlb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto_deps[4] = {
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fPose_2eproto,
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
  &::descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto,
  &::descriptor_table_ros_2fvisualization_5fmsgs_2fMenuEntry_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto = {
  false, false, 506, descriptor_table_protodef_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto, "ros/visualization_msgs/InteractiveMarker.proto", 
  &descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto_once, descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto_deps, 4, 1,
  schemas, file_default_instances, TableStruct_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto::offsets,
  file_level_metadata_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto, file_level_enum_descriptors_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto, file_level_service_descriptors_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto_getter() {
  return &descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto(&descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto);
namespace ros {
namespace visualization_msgs {

// ===================================================================

class InteractiveMarker::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const InteractiveMarker* msg);
  static const ::ros::geometry_msgs::Pose& pose(const InteractiveMarker* msg);
};

const ::ros::std_msgs::Header&
InteractiveMarker::_Internal::header(const InteractiveMarker* msg) {
  return *msg->header_;
}
const ::ros::geometry_msgs::Pose&
InteractiveMarker::_Internal::pose(const InteractiveMarker* msg) {
  return *msg->pose_;
}
void InteractiveMarker::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void InteractiveMarker::clear_pose() {
  if (GetArenaForAllocation() == nullptr && pose_ != nullptr) {
    delete pose_;
  }
  pose_ = nullptr;
}
void InteractiveMarker::clear_menu_entries() {
  menu_entries_.Clear();
}
void InteractiveMarker::clear_controls() {
  controls_.Clear();
}
InteractiveMarker::InteractiveMarker(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  menu_entries_(arena),
  controls_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.visualization_msgs.InteractiveMarker)
}
InteractiveMarker::InteractiveMarker(const InteractiveMarker& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      menu_entries_(from.menu_entries_),
      controls_(from.controls_) {
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
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_pose()) {
    pose_ = new ::ros::geometry_msgs::Pose(*from.pose_);
  } else {
    pose_ = nullptr;
  }
  scale_ = from.scale_;
  // @@protoc_insertion_point(copy_constructor:ros.visualization_msgs.InteractiveMarker)
}

inline void InteractiveMarker::SharedCtor() {
name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  name_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
description_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  description_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&scale_) -
    reinterpret_cast<char*>(&header_)) + sizeof(scale_));
}

InteractiveMarker::~InteractiveMarker() {
  // @@protoc_insertion_point(destructor:ros.visualization_msgs.InteractiveMarker)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void InteractiveMarker::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  name_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  description_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete pose_;
}

void InteractiveMarker::ArenaDtor(void* object) {
  InteractiveMarker* _this = reinterpret_cast< InteractiveMarker* >(object);
  (void)_this;
}
void InteractiveMarker::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void InteractiveMarker::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void InteractiveMarker::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.visualization_msgs.InteractiveMarker)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  menu_entries_.Clear();
  controls_.Clear();
  name_.ClearToEmpty();
  description_.ClearToEmpty();
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  if (GetArenaForAllocation() == nullptr && pose_ != nullptr) {
    delete pose_;
  }
  pose_ = nullptr;
  scale_ = 0;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* InteractiveMarker::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .ros.std_msgs.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .ros.geometry_msgs.Pose pose = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_pose(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // string name = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          auto str = _internal_mutable_name();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "ros.visualization_msgs.InteractiveMarker.name"));
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // string description = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 34)) {
          auto str = _internal_mutable_description();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "ros.visualization_msgs.InteractiveMarker.description"));
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // float scale = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 45)) {
          scale_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // repeated .ros.visualization_msgs.MenuEntry menu_entries = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 50)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_menu_entries(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<50>(ptr));
        } else
          goto handle_unusual;
        continue;
      // repeated .ros.visualization_msgs.InteractiveMarkerControl controls = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 58)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_controls(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<58>(ptr));
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

uint8_t* InteractiveMarker::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.visualization_msgs.InteractiveMarker)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // .ros.geometry_msgs.Pose pose = 2;
  if (this->_internal_has_pose()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::pose(this), target, stream);
  }

  // string name = 3;
  if (!this->_internal_name().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_name().data(), static_cast<int>(this->_internal_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "ros.visualization_msgs.InteractiveMarker.name");
    target = stream->WriteStringMaybeAliased(
        3, this->_internal_name(), target);
  }

  // string description = 4;
  if (!this->_internal_description().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_description().data(), static_cast<int>(this->_internal_description().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "ros.visualization_msgs.InteractiveMarker.description");
    target = stream->WriteStringMaybeAliased(
        4, this->_internal_description(), target);
  }

  // float scale = 5;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_scale = this->_internal_scale();
  uint32_t raw_scale;
  memcpy(&raw_scale, &tmp_scale, sizeof(tmp_scale));
  if (raw_scale != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(5, this->_internal_scale(), target);
  }

  // repeated .ros.visualization_msgs.MenuEntry menu_entries = 6;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_menu_entries_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(6, this->_internal_menu_entries(i), target, stream);
  }

  // repeated .ros.visualization_msgs.InteractiveMarkerControl controls = 7;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_controls_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(7, this->_internal_controls(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.visualization_msgs.InteractiveMarker)
  return target;
}

size_t InteractiveMarker::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.visualization_msgs.InteractiveMarker)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .ros.visualization_msgs.MenuEntry menu_entries = 6;
  total_size += 1UL * this->_internal_menu_entries_size();
  for (const auto& msg : this->menu_entries_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .ros.visualization_msgs.InteractiveMarkerControl controls = 7;
  total_size += 1UL * this->_internal_controls_size();
  for (const auto& msg : this->controls_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // string name = 3;
  if (!this->_internal_name().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_name());
  }

  // string description = 4;
  if (!this->_internal_description().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_description());
  }

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  // .ros.geometry_msgs.Pose pose = 2;
  if (this->_internal_has_pose()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *pose_);
  }

  // float scale = 5;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_scale = this->_internal_scale();
  uint32_t raw_scale;
  memcpy(&raw_scale, &tmp_scale, sizeof(tmp_scale));
  if (raw_scale != 0) {
    total_size += 1 + 4;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData InteractiveMarker::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    InteractiveMarker::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*InteractiveMarker::GetClassData() const { return &_class_data_; }

void InteractiveMarker::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<InteractiveMarker *>(to)->MergeFrom(
      static_cast<const InteractiveMarker &>(from));
}


void InteractiveMarker::MergeFrom(const InteractiveMarker& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.visualization_msgs.InteractiveMarker)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  menu_entries_.MergeFrom(from.menu_entries_);
  controls_.MergeFrom(from.controls_);
  if (!from._internal_name().empty()) {
    _internal_set_name(from._internal_name());
  }
  if (!from._internal_description().empty()) {
    _internal_set_description(from._internal_description());
  }
  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  if (from._internal_has_pose()) {
    _internal_mutable_pose()->::ros::geometry_msgs::Pose::MergeFrom(from._internal_pose());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_scale = from._internal_scale();
  uint32_t raw_scale;
  memcpy(&raw_scale, &tmp_scale, sizeof(tmp_scale));
  if (raw_scale != 0) {
    _internal_set_scale(from._internal_scale());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void InteractiveMarker::CopyFrom(const InteractiveMarker& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.visualization_msgs.InteractiveMarker)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool InteractiveMarker::IsInitialized() const {
  return true;
}

void InteractiveMarker::InternalSwap(InteractiveMarker* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  menu_entries_.InternalSwap(&other->menu_entries_);
  controls_.InternalSwap(&other->controls_);
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
      PROTOBUF_FIELD_OFFSET(InteractiveMarker, scale_)
      + sizeof(InteractiveMarker::scale_)
      - PROTOBUF_FIELD_OFFSET(InteractiveMarker, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata InteractiveMarker::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto_getter, &descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto_once,
      file_level_metadata_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace visualization_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::visualization_msgs::InteractiveMarker* Arena::CreateMaybeMessage< ::ros::visualization_msgs::InteractiveMarker >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::visualization_msgs::InteractiveMarker >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
