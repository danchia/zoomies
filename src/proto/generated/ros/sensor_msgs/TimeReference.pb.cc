// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/TimeReference.proto

#include "ros/sensor_msgs/TimeReference.pb.h"

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
namespace sensor_msgs {
constexpr TimeReference::TimeReference(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : source_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , header_(nullptr)
  , time_ref_(nullptr){}
struct TimeReferenceDefaultTypeInternal {
  constexpr TimeReferenceDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~TimeReferenceDefaultTypeInternal() {}
  union {
    TimeReference _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT TimeReferenceDefaultTypeInternal _TimeReference_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fsensor_5fmsgs_2fTimeReference_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fTimeReference_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fsensor_5fmsgs_2fTimeReference_2eproto = nullptr;

const uint32_t TableStruct_ros_2fsensor_5fmsgs_2fTimeReference_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::TimeReference, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::TimeReference, header_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::TimeReference, time_ref_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::TimeReference, source_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::sensor_msgs::TimeReference)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::sensor_msgs::_TimeReference_default_instance_),
};

const char descriptor_table_protodef_ros_2fsensor_5fmsgs_2fTimeReference_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n#ros/sensor_msgs/TimeReference.proto\022\017r"
  "os.sensor_msgs\032\022ros/builtins.proto\032\031ros/"
  "std_msgs/Header.proto\"b\n\rTimeReference\022$"
  "\n\006header\030\001 \001(\0132\024.ros.std_msgs.Header\022\033\n\010"
  "time_ref\030\002 \001(\0132\t.ros.Time\022\016\n\006source\030\003 \001("
  "\tb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fsensor_5fmsgs_2fTimeReference_2eproto_deps[2] = {
  &::descriptor_table_ros_2fbuiltins_2eproto,
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fsensor_5fmsgs_2fTimeReference_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fTimeReference_2eproto = {
  false, false, 209, descriptor_table_protodef_ros_2fsensor_5fmsgs_2fTimeReference_2eproto, "ros/sensor_msgs/TimeReference.proto", 
  &descriptor_table_ros_2fsensor_5fmsgs_2fTimeReference_2eproto_once, descriptor_table_ros_2fsensor_5fmsgs_2fTimeReference_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fsensor_5fmsgs_2fTimeReference_2eproto::offsets,
  file_level_metadata_ros_2fsensor_5fmsgs_2fTimeReference_2eproto, file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fTimeReference_2eproto, file_level_service_descriptors_ros_2fsensor_5fmsgs_2fTimeReference_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fsensor_5fmsgs_2fTimeReference_2eproto_getter() {
  return &descriptor_table_ros_2fsensor_5fmsgs_2fTimeReference_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fsensor_5fmsgs_2fTimeReference_2eproto(&descriptor_table_ros_2fsensor_5fmsgs_2fTimeReference_2eproto);
namespace ros {
namespace sensor_msgs {

// ===================================================================

class TimeReference::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const TimeReference* msg);
  static const ::ros::Time& time_ref(const TimeReference* msg);
};

const ::ros::std_msgs::Header&
TimeReference::_Internal::header(const TimeReference* msg) {
  return *msg->header_;
}
const ::ros::Time&
TimeReference::_Internal::time_ref(const TimeReference* msg) {
  return *msg->time_ref_;
}
void TimeReference::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void TimeReference::clear_time_ref() {
  if (GetArenaForAllocation() == nullptr && time_ref_ != nullptr) {
    delete time_ref_;
  }
  time_ref_ = nullptr;
}
TimeReference::TimeReference(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.sensor_msgs.TimeReference)
}
TimeReference::TimeReference(const TimeReference& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  source_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    source_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (!from._internal_source().empty()) {
    source_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_source(), 
      GetArenaForAllocation());
  }
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_time_ref()) {
    time_ref_ = new ::ros::Time(*from.time_ref_);
  } else {
    time_ref_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:ros.sensor_msgs.TimeReference)
}

inline void TimeReference::SharedCtor() {
source_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  source_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&time_ref_) -
    reinterpret_cast<char*>(&header_)) + sizeof(time_ref_));
}

TimeReference::~TimeReference() {
  // @@protoc_insertion_point(destructor:ros.sensor_msgs.TimeReference)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void TimeReference::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  source_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete time_ref_;
}

void TimeReference::ArenaDtor(void* object) {
  TimeReference* _this = reinterpret_cast< TimeReference* >(object);
  (void)_this;
}
void TimeReference::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void TimeReference::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void TimeReference::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.sensor_msgs.TimeReference)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  source_.ClearToEmpty();
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  if (GetArenaForAllocation() == nullptr && time_ref_ != nullptr) {
    delete time_ref_;
  }
  time_ref_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* TimeReference::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // .ros.Time time_ref = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_time_ref(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // string source = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          auto str = _internal_mutable_source();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "ros.sensor_msgs.TimeReference.source"));
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

uint8_t* TimeReference::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.sensor_msgs.TimeReference)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // .ros.Time time_ref = 2;
  if (this->_internal_has_time_ref()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::time_ref(this), target, stream);
  }

  // string source = 3;
  if (!this->_internal_source().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_source().data(), static_cast<int>(this->_internal_source().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "ros.sensor_msgs.TimeReference.source");
    target = stream->WriteStringMaybeAliased(
        3, this->_internal_source(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.sensor_msgs.TimeReference)
  return target;
}

size_t TimeReference::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.sensor_msgs.TimeReference)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string source = 3;
  if (!this->_internal_source().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_source());
  }

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  // .ros.Time time_ref = 2;
  if (this->_internal_has_time_ref()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *time_ref_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData TimeReference::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    TimeReference::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*TimeReference::GetClassData() const { return &_class_data_; }

void TimeReference::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<TimeReference *>(to)->MergeFrom(
      static_cast<const TimeReference &>(from));
}


void TimeReference::MergeFrom(const TimeReference& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.sensor_msgs.TimeReference)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (!from._internal_source().empty()) {
    _internal_set_source(from._internal_source());
  }
  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  if (from._internal_has_time_ref()) {
    _internal_mutable_time_ref()->::ros::Time::MergeFrom(from._internal_time_ref());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void TimeReference::CopyFrom(const TimeReference& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.sensor_msgs.TimeReference)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TimeReference::IsInitialized() const {
  return true;
}

void TimeReference::InternalSwap(TimeReference* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &source_, lhs_arena,
      &other->source_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(TimeReference, time_ref_)
      + sizeof(TimeReference::time_ref_)
      - PROTOBUF_FIELD_OFFSET(TimeReference, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata TimeReference::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fsensor_5fmsgs_2fTimeReference_2eproto_getter, &descriptor_table_ros_2fsensor_5fmsgs_2fTimeReference_2eproto_once,
      file_level_metadata_ros_2fsensor_5fmsgs_2fTimeReference_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::sensor_msgs::TimeReference* Arena::CreateMaybeMessage< ::ros::sensor_msgs::TimeReference >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::sensor_msgs::TimeReference >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
