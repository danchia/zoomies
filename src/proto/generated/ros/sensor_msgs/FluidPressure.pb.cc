// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/FluidPressure.proto

#include "ros/sensor_msgs/FluidPressure.pb.h"

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
constexpr FluidPressure::FluidPressure(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : header_(nullptr)
  , fluid_pressure_(0)
  , variance_(0){}
struct FluidPressureDefaultTypeInternal {
  constexpr FluidPressureDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~FluidPressureDefaultTypeInternal() {}
  union {
    FluidPressure _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT FluidPressureDefaultTypeInternal _FluidPressure_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto = nullptr;

const uint32_t TableStruct_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::FluidPressure, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::FluidPressure, header_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::FluidPressure, fluid_pressure_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::FluidPressure, variance_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::sensor_msgs::FluidPressure)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::sensor_msgs::_FluidPressure_default_instance_),
};

const char descriptor_table_protodef_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n#ros/sensor_msgs/FluidPressure.proto\022\017r"
  "os.sensor_msgs\032\031ros/std_msgs/Header.prot"
  "o\"_\n\rFluidPressure\022$\n\006header\030\001 \001(\0132\024.ros"
  ".std_msgs.Header\022\026\n\016fluid_pressure\030\002 \001(\001"
  "\022\020\n\010variance\030\003 \001(\001b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto_deps[1] = {
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto = {
  false, false, 186, descriptor_table_protodef_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto, "ros/sensor_msgs/FluidPressure.proto", 
  &descriptor_table_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto_once, descriptor_table_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto::offsets,
  file_level_metadata_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto, file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto, file_level_service_descriptors_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto_getter() {
  return &descriptor_table_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto(&descriptor_table_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto);
namespace ros {
namespace sensor_msgs {

// ===================================================================

class FluidPressure::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const FluidPressure* msg);
};

const ::ros::std_msgs::Header&
FluidPressure::_Internal::header(const FluidPressure* msg) {
  return *msg->header_;
}
void FluidPressure::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
FluidPressure::FluidPressure(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.sensor_msgs.FluidPressure)
}
FluidPressure::FluidPressure(const FluidPressure& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  ::memcpy(&fluid_pressure_, &from.fluid_pressure_,
    static_cast<size_t>(reinterpret_cast<char*>(&variance_) -
    reinterpret_cast<char*>(&fluid_pressure_)) + sizeof(variance_));
  // @@protoc_insertion_point(copy_constructor:ros.sensor_msgs.FluidPressure)
}

inline void FluidPressure::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&variance_) -
    reinterpret_cast<char*>(&header_)) + sizeof(variance_));
}

FluidPressure::~FluidPressure() {
  // @@protoc_insertion_point(destructor:ros.sensor_msgs.FluidPressure)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void FluidPressure::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
}

void FluidPressure::ArenaDtor(void* object) {
  FluidPressure* _this = reinterpret_cast< FluidPressure* >(object);
  (void)_this;
}
void FluidPressure::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void FluidPressure::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void FluidPressure::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.sensor_msgs.FluidPressure)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  ::memset(&fluid_pressure_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&variance_) -
      reinterpret_cast<char*>(&fluid_pressure_)) + sizeof(variance_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* FluidPressure::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // double fluid_pressure = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 17)) {
          fluid_pressure_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double variance = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 25)) {
          variance_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
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

uint8_t* FluidPressure::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.sensor_msgs.FluidPressure)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // double fluid_pressure = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_fluid_pressure = this->_internal_fluid_pressure();
  uint64_t raw_fluid_pressure;
  memcpy(&raw_fluid_pressure, &tmp_fluid_pressure, sizeof(tmp_fluid_pressure));
  if (raw_fluid_pressure != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_fluid_pressure(), target);
  }

  // double variance = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_variance = this->_internal_variance();
  uint64_t raw_variance;
  memcpy(&raw_variance, &tmp_variance, sizeof(tmp_variance));
  if (raw_variance != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_variance(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.sensor_msgs.FluidPressure)
  return target;
}

size_t FluidPressure::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.sensor_msgs.FluidPressure)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  // double fluid_pressure = 2;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_fluid_pressure = this->_internal_fluid_pressure();
  uint64_t raw_fluid_pressure;
  memcpy(&raw_fluid_pressure, &tmp_fluid_pressure, sizeof(tmp_fluid_pressure));
  if (raw_fluid_pressure != 0) {
    total_size += 1 + 8;
  }

  // double variance = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_variance = this->_internal_variance();
  uint64_t raw_variance;
  memcpy(&raw_variance, &tmp_variance, sizeof(tmp_variance));
  if (raw_variance != 0) {
    total_size += 1 + 8;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData FluidPressure::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    FluidPressure::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*FluidPressure::GetClassData() const { return &_class_data_; }

void FluidPressure::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<FluidPressure *>(to)->MergeFrom(
      static_cast<const FluidPressure &>(from));
}


void FluidPressure::MergeFrom(const FluidPressure& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.sensor_msgs.FluidPressure)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_fluid_pressure = from._internal_fluid_pressure();
  uint64_t raw_fluid_pressure;
  memcpy(&raw_fluid_pressure, &tmp_fluid_pressure, sizeof(tmp_fluid_pressure));
  if (raw_fluid_pressure != 0) {
    _internal_set_fluid_pressure(from._internal_fluid_pressure());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_variance = from._internal_variance();
  uint64_t raw_variance;
  memcpy(&raw_variance, &tmp_variance, sizeof(tmp_variance));
  if (raw_variance != 0) {
    _internal_set_variance(from._internal_variance());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void FluidPressure::CopyFrom(const FluidPressure& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.sensor_msgs.FluidPressure)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool FluidPressure::IsInitialized() const {
  return true;
}

void FluidPressure::InternalSwap(FluidPressure* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(FluidPressure, variance_)
      + sizeof(FluidPressure::variance_)
      - PROTOBUF_FIELD_OFFSET(FluidPressure, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata FluidPressure::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto_getter, &descriptor_table_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto_once,
      file_level_metadata_ros_2fsensor_5fmsgs_2fFluidPressure_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::sensor_msgs::FluidPressure* Arena::CreateMaybeMessage< ::ros::sensor_msgs::FluidPressure >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::sensor_msgs::FluidPressure >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
