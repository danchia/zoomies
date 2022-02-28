// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/actionlib_msgs/GoalStatusArray.proto

#include "ros/actionlib_msgs/GoalStatusArray.pb.h"

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
namespace actionlib_msgs {
constexpr GoalStatusArray::GoalStatusArray(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : status_list_()
  , header_(nullptr){}
struct GoalStatusArrayDefaultTypeInternal {
  constexpr GoalStatusArrayDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~GoalStatusArrayDefaultTypeInternal() {}
  union {
    GoalStatusArray _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT GoalStatusArrayDefaultTypeInternal _GoalStatusArray_default_instance_;
}  // namespace actionlib_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto = nullptr;

const uint32_t TableStruct_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::actionlib_msgs::GoalStatusArray, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::actionlib_msgs::GoalStatusArray, header_),
  PROTOBUF_FIELD_OFFSET(::ros::actionlib_msgs::GoalStatusArray, status_list_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::actionlib_msgs::GoalStatusArray)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::actionlib_msgs::_GoalStatusArray_default_instance_),
};

const char descriptor_table_protodef_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n(ros/actionlib_msgs/GoalStatusArray.pro"
  "to\022\022ros.actionlib_msgs\032#ros/actionlib_ms"
  "gs/GoalStatus.proto\032\031ros/std_msgs/Header"
  ".proto\"l\n\017GoalStatusArray\022$\n\006header\030\001 \001("
  "\0132\024.ros.std_msgs.Header\0223\n\013status_list\030\002"
  " \003(\0132\036.ros.actionlib_msgs.GoalStatusb\006pr"
  "oto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto_deps[2] = {
  &::descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto,
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto = {
  false, false, 244, descriptor_table_protodef_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto, "ros/actionlib_msgs/GoalStatusArray.proto", 
  &descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto_once, descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto::offsets,
  file_level_metadata_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto, file_level_enum_descriptors_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto, file_level_service_descriptors_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto_getter() {
  return &descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto(&descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto);
namespace ros {
namespace actionlib_msgs {

// ===================================================================

class GoalStatusArray::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const GoalStatusArray* msg);
};

const ::ros::std_msgs::Header&
GoalStatusArray::_Internal::header(const GoalStatusArray* msg) {
  return *msg->header_;
}
void GoalStatusArray::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void GoalStatusArray::clear_status_list() {
  status_list_.Clear();
}
GoalStatusArray::GoalStatusArray(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  status_list_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.actionlib_msgs.GoalStatusArray)
}
GoalStatusArray::GoalStatusArray(const GoalStatusArray& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      status_list_(from.status_list_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:ros.actionlib_msgs.GoalStatusArray)
}

inline void GoalStatusArray::SharedCtor() {
header_ = nullptr;
}

GoalStatusArray::~GoalStatusArray() {
  // @@protoc_insertion_point(destructor:ros.actionlib_msgs.GoalStatusArray)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void GoalStatusArray::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
}

void GoalStatusArray::ArenaDtor(void* object) {
  GoalStatusArray* _this = reinterpret_cast< GoalStatusArray* >(object);
  (void)_this;
}
void GoalStatusArray::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void GoalStatusArray::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void GoalStatusArray::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.actionlib_msgs.GoalStatusArray)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  status_list_.Clear();
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* GoalStatusArray::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // repeated .ros.actionlib_msgs.GoalStatus status_list = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_status_list(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
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

uint8_t* GoalStatusArray::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.actionlib_msgs.GoalStatusArray)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // repeated .ros.actionlib_msgs.GoalStatus status_list = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_status_list_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, this->_internal_status_list(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.actionlib_msgs.GoalStatusArray)
  return target;
}

size_t GoalStatusArray::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.actionlib_msgs.GoalStatusArray)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .ros.actionlib_msgs.GoalStatus status_list = 2;
  total_size += 1UL * this->_internal_status_list_size();
  for (const auto& msg : this->status_list_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData GoalStatusArray::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    GoalStatusArray::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GoalStatusArray::GetClassData() const { return &_class_data_; }

void GoalStatusArray::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<GoalStatusArray *>(to)->MergeFrom(
      static_cast<const GoalStatusArray &>(from));
}


void GoalStatusArray::MergeFrom(const GoalStatusArray& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.actionlib_msgs.GoalStatusArray)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  status_list_.MergeFrom(from.status_list_);
  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void GoalStatusArray::CopyFrom(const GoalStatusArray& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.actionlib_msgs.GoalStatusArray)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool GoalStatusArray::IsInitialized() const {
  return true;
}

void GoalStatusArray::InternalSwap(GoalStatusArray* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  status_list_.InternalSwap(&other->status_list_);
  swap(header_, other->header_);
}

::PROTOBUF_NAMESPACE_ID::Metadata GoalStatusArray::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto_getter, &descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto_once,
      file_level_metadata_ros_2factionlib_5fmsgs_2fGoalStatusArray_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace actionlib_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::actionlib_msgs::GoalStatusArray* Arena::CreateMaybeMessage< ::ros::actionlib_msgs::GoalStatusArray >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::actionlib_msgs::GoalStatusArray >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>