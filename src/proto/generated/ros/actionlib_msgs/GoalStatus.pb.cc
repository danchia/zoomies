// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/actionlib_msgs/GoalStatus.proto

#include "ros/actionlib_msgs/GoalStatus.pb.h"

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
constexpr GoalStatus::GoalStatus(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : text_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , goal_id_(nullptr)
  , status_(0){}
struct GoalStatusDefaultTypeInternal {
  constexpr GoalStatusDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~GoalStatusDefaultTypeInternal() {}
  union {
    GoalStatus _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT GoalStatusDefaultTypeInternal _GoalStatus_default_instance_;
}  // namespace actionlib_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto = nullptr;

const uint32_t TableStruct_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::actionlib_msgs::GoalStatus, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::actionlib_msgs::GoalStatus, goal_id_),
  PROTOBUF_FIELD_OFFSET(::ros::actionlib_msgs::GoalStatus, status_),
  PROTOBUF_FIELD_OFFSET(::ros::actionlib_msgs::GoalStatus, text_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::actionlib_msgs::GoalStatus)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::actionlib_msgs::_GoalStatus_default_instance_),
};

const char descriptor_table_protodef_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n#ros/actionlib_msgs/GoalStatus.proto\022\022r"
  "os.actionlib_msgs\032\037ros/actionlib_msgs/Go"
  "alID.proto\"W\n\nGoalStatus\022+\n\007goal_id\030\001 \001("
  "\0132\032.ros.actionlib_msgs.GoalID\022\016\n\006status\030"
  "\002 \001(\005\022\014\n\004text\030\003 \001(\tb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto_deps[1] = {
  &::descriptor_table_ros_2factionlib_5fmsgs_2fGoalID_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto = {
  false, false, 187, descriptor_table_protodef_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto, "ros/actionlib_msgs/GoalStatus.proto", 
  &descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto_once, descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto::offsets,
  file_level_metadata_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto, file_level_enum_descriptors_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto, file_level_service_descriptors_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto_getter() {
  return &descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto(&descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto);
namespace ros {
namespace actionlib_msgs {

// ===================================================================

class GoalStatus::_Internal {
 public:
  static const ::ros::actionlib_msgs::GoalID& goal_id(const GoalStatus* msg);
};

const ::ros::actionlib_msgs::GoalID&
GoalStatus::_Internal::goal_id(const GoalStatus* msg) {
  return *msg->goal_id_;
}
void GoalStatus::clear_goal_id() {
  if (GetArenaForAllocation() == nullptr && goal_id_ != nullptr) {
    delete goal_id_;
  }
  goal_id_ = nullptr;
}
GoalStatus::GoalStatus(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.actionlib_msgs.GoalStatus)
}
GoalStatus::GoalStatus(const GoalStatus& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  text_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    text_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (!from._internal_text().empty()) {
    text_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_text(), 
      GetArenaForAllocation());
  }
  if (from._internal_has_goal_id()) {
    goal_id_ = new ::ros::actionlib_msgs::GoalID(*from.goal_id_);
  } else {
    goal_id_ = nullptr;
  }
  status_ = from.status_;
  // @@protoc_insertion_point(copy_constructor:ros.actionlib_msgs.GoalStatus)
}

inline void GoalStatus::SharedCtor() {
text_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  text_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&goal_id_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&status_) -
    reinterpret_cast<char*>(&goal_id_)) + sizeof(status_));
}

GoalStatus::~GoalStatus() {
  // @@protoc_insertion_point(destructor:ros.actionlib_msgs.GoalStatus)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void GoalStatus::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  text_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete goal_id_;
}

void GoalStatus::ArenaDtor(void* object) {
  GoalStatus* _this = reinterpret_cast< GoalStatus* >(object);
  (void)_this;
}
void GoalStatus::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void GoalStatus::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void GoalStatus::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.actionlib_msgs.GoalStatus)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  text_.ClearToEmpty();
  if (GetArenaForAllocation() == nullptr && goal_id_ != nullptr) {
    delete goal_id_;
  }
  goal_id_ = nullptr;
  status_ = 0;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* GoalStatus::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .ros.actionlib_msgs.GoalID goal_id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_goal_id(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // int32 status = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 16)) {
          status_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // string text = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          auto str = _internal_mutable_text();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "ros.actionlib_msgs.GoalStatus.text"));
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

uint8_t* GoalStatus::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.actionlib_msgs.GoalStatus)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.actionlib_msgs.GoalID goal_id = 1;
  if (this->_internal_has_goal_id()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::goal_id(this), target, stream);
  }

  // int32 status = 2;
  if (this->_internal_status() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(2, this->_internal_status(), target);
  }

  // string text = 3;
  if (!this->_internal_text().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_text().data(), static_cast<int>(this->_internal_text().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "ros.actionlib_msgs.GoalStatus.text");
    target = stream->WriteStringMaybeAliased(
        3, this->_internal_text(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.actionlib_msgs.GoalStatus)
  return target;
}

size_t GoalStatus::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.actionlib_msgs.GoalStatus)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string text = 3;
  if (!this->_internal_text().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_text());
  }

  // .ros.actionlib_msgs.GoalID goal_id = 1;
  if (this->_internal_has_goal_id()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *goal_id_);
  }

  // int32 status = 2;
  if (this->_internal_status() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_status());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData GoalStatus::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    GoalStatus::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GoalStatus::GetClassData() const { return &_class_data_; }

void GoalStatus::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<GoalStatus *>(to)->MergeFrom(
      static_cast<const GoalStatus &>(from));
}


void GoalStatus::MergeFrom(const GoalStatus& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.actionlib_msgs.GoalStatus)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (!from._internal_text().empty()) {
    _internal_set_text(from._internal_text());
  }
  if (from._internal_has_goal_id()) {
    _internal_mutable_goal_id()->::ros::actionlib_msgs::GoalID::MergeFrom(from._internal_goal_id());
  }
  if (from._internal_status() != 0) {
    _internal_set_status(from._internal_status());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void GoalStatus::CopyFrom(const GoalStatus& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.actionlib_msgs.GoalStatus)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool GoalStatus::IsInitialized() const {
  return true;
}

void GoalStatus::InternalSwap(GoalStatus* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &text_, lhs_arena,
      &other->text_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(GoalStatus, status_)
      + sizeof(GoalStatus::status_)
      - PROTOBUF_FIELD_OFFSET(GoalStatus, goal_id_)>(
          reinterpret_cast<char*>(&goal_id_),
          reinterpret_cast<char*>(&other->goal_id_));
}

::PROTOBUF_NAMESPACE_ID::Metadata GoalStatus::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto_getter, &descriptor_table_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto_once,
      file_level_metadata_ros_2factionlib_5fmsgs_2fGoalStatus_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace actionlib_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::actionlib_msgs::GoalStatus* Arena::CreateMaybeMessage< ::ros::actionlib_msgs::GoalStatus >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::actionlib_msgs::GoalStatus >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
