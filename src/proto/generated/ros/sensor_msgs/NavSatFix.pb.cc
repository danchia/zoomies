// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/NavSatFix.proto

#include "ros/sensor_msgs/NavSatFix.pb.h"

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
constexpr NavSatFix::NavSatFix(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : position_covariance_()
  , header_(nullptr)
  , status_(nullptr)
  , latitude_(0)
  , longitude_(0)
  , altitude_(0)
  , position_covariance_type_(0){}
struct NavSatFixDefaultTypeInternal {
  constexpr NavSatFixDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~NavSatFixDefaultTypeInternal() {}
  union {
    NavSatFix _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT NavSatFixDefaultTypeInternal _NavSatFix_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto = nullptr;

const uint32_t TableStruct_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::NavSatFix, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::NavSatFix, header_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::NavSatFix, status_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::NavSatFix, latitude_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::NavSatFix, longitude_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::NavSatFix, altitude_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::NavSatFix, position_covariance_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::NavSatFix, position_covariance_type_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::sensor_msgs::NavSatFix)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::sensor_msgs::_NavSatFix_default_instance_),
};

const char descriptor_table_protodef_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\037ros/sensor_msgs/NavSatFix.proto\022\017ros.s"
  "ensor_msgs\032\"ros/sensor_msgs/NavSatStatus"
  ".proto\032\031ros/std_msgs/Header.proto\"\326\001\n\tNa"
  "vSatFix\022$\n\006header\030\001 \001(\0132\024.ros.std_msgs.H"
  "eader\022-\n\006status\030\002 \001(\0132\035.ros.sensor_msgs."
  "NavSatStatus\022\020\n\010latitude\030\003 \001(\001\022\021\n\tlongit"
  "ude\030\004 \001(\001\022\020\n\010altitude\030\005 \001(\001\022\033\n\023position_"
  "covariance\030\006 \003(\001\022 \n\030position_covariance_"
  "type\030\007 \001(\005b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto_deps[2] = {
  &::descriptor_table_ros_2fsensor_5fmsgs_2fNavSatStatus_2eproto,
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto = {
  false, false, 338, descriptor_table_protodef_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto, "ros/sensor_msgs/NavSatFix.proto", 
  &descriptor_table_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto_once, descriptor_table_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto::offsets,
  file_level_metadata_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto, file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto, file_level_service_descriptors_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto_getter() {
  return &descriptor_table_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto(&descriptor_table_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto);
namespace ros {
namespace sensor_msgs {

// ===================================================================

class NavSatFix::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const NavSatFix* msg);
  static const ::ros::sensor_msgs::NavSatStatus& status(const NavSatFix* msg);
};

const ::ros::std_msgs::Header&
NavSatFix::_Internal::header(const NavSatFix* msg) {
  return *msg->header_;
}
const ::ros::sensor_msgs::NavSatStatus&
NavSatFix::_Internal::status(const NavSatFix* msg) {
  return *msg->status_;
}
void NavSatFix::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void NavSatFix::clear_status() {
  if (GetArenaForAllocation() == nullptr && status_ != nullptr) {
    delete status_;
  }
  status_ = nullptr;
}
NavSatFix::NavSatFix(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  position_covariance_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.sensor_msgs.NavSatFix)
}
NavSatFix::NavSatFix(const NavSatFix& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      position_covariance_(from.position_covariance_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_status()) {
    status_ = new ::ros::sensor_msgs::NavSatStatus(*from.status_);
  } else {
    status_ = nullptr;
  }
  ::memcpy(&latitude_, &from.latitude_,
    static_cast<size_t>(reinterpret_cast<char*>(&position_covariance_type_) -
    reinterpret_cast<char*>(&latitude_)) + sizeof(position_covariance_type_));
  // @@protoc_insertion_point(copy_constructor:ros.sensor_msgs.NavSatFix)
}

inline void NavSatFix::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&position_covariance_type_) -
    reinterpret_cast<char*>(&header_)) + sizeof(position_covariance_type_));
}

NavSatFix::~NavSatFix() {
  // @@protoc_insertion_point(destructor:ros.sensor_msgs.NavSatFix)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void NavSatFix::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete status_;
}

void NavSatFix::ArenaDtor(void* object) {
  NavSatFix* _this = reinterpret_cast< NavSatFix* >(object);
  (void)_this;
}
void NavSatFix::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void NavSatFix::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void NavSatFix::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.sensor_msgs.NavSatFix)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  position_covariance_.Clear();
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  if (GetArenaForAllocation() == nullptr && status_ != nullptr) {
    delete status_;
  }
  status_ = nullptr;
  ::memset(&latitude_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&position_covariance_type_) -
      reinterpret_cast<char*>(&latitude_)) + sizeof(position_covariance_type_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* NavSatFix::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // .ros.sensor_msgs.NavSatStatus status = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_status(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // double latitude = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 25)) {
          latitude_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double longitude = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 33)) {
          longitude_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // double altitude = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 41)) {
          altitude_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // repeated double position_covariance = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 50)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_position_covariance(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 49) {
          _internal_add_position_covariance(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // int32 position_covariance_type = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 56)) {
          position_covariance_type_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
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

uint8_t* NavSatFix::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.sensor_msgs.NavSatFix)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // .ros.sensor_msgs.NavSatStatus status = 2;
  if (this->_internal_has_status()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::status(this), target, stream);
  }

  // double latitude = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_latitude = this->_internal_latitude();
  uint64_t raw_latitude;
  memcpy(&raw_latitude, &tmp_latitude, sizeof(tmp_latitude));
  if (raw_latitude != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_latitude(), target);
  }

  // double longitude = 4;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_longitude = this->_internal_longitude();
  uint64_t raw_longitude;
  memcpy(&raw_longitude, &tmp_longitude, sizeof(tmp_longitude));
  if (raw_longitude != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_longitude(), target);
  }

  // double altitude = 5;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_altitude = this->_internal_altitude();
  uint64_t raw_altitude;
  memcpy(&raw_altitude, &tmp_altitude, sizeof(tmp_altitude));
  if (raw_altitude != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(5, this->_internal_altitude(), target);
  }

  // repeated double position_covariance = 6;
  if (this->_internal_position_covariance_size() > 0) {
    target = stream->WriteFixedPacked(6, _internal_position_covariance(), target);
  }

  // int32 position_covariance_type = 7;
  if (this->_internal_position_covariance_type() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(7, this->_internal_position_covariance_type(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.sensor_msgs.NavSatFix)
  return target;
}

size_t NavSatFix::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.sensor_msgs.NavSatFix)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated double position_covariance = 6;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_position_covariance_size());
    size_t data_size = 8UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
            static_cast<int32_t>(data_size));
    }
    total_size += data_size;
  }

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  // .ros.sensor_msgs.NavSatStatus status = 2;
  if (this->_internal_has_status()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *status_);
  }

  // double latitude = 3;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_latitude = this->_internal_latitude();
  uint64_t raw_latitude;
  memcpy(&raw_latitude, &tmp_latitude, sizeof(tmp_latitude));
  if (raw_latitude != 0) {
    total_size += 1 + 8;
  }

  // double longitude = 4;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_longitude = this->_internal_longitude();
  uint64_t raw_longitude;
  memcpy(&raw_longitude, &tmp_longitude, sizeof(tmp_longitude));
  if (raw_longitude != 0) {
    total_size += 1 + 8;
  }

  // double altitude = 5;
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_altitude = this->_internal_altitude();
  uint64_t raw_altitude;
  memcpy(&raw_altitude, &tmp_altitude, sizeof(tmp_altitude));
  if (raw_altitude != 0) {
    total_size += 1 + 8;
  }

  // int32 position_covariance_type = 7;
  if (this->_internal_position_covariance_type() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32SizePlusOne(this->_internal_position_covariance_type());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData NavSatFix::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    NavSatFix::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*NavSatFix::GetClassData() const { return &_class_data_; }

void NavSatFix::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<NavSatFix *>(to)->MergeFrom(
      static_cast<const NavSatFix &>(from));
}


void NavSatFix::MergeFrom(const NavSatFix& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.sensor_msgs.NavSatFix)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  position_covariance_.MergeFrom(from.position_covariance_);
  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  if (from._internal_has_status()) {
    _internal_mutable_status()->::ros::sensor_msgs::NavSatStatus::MergeFrom(from._internal_status());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_latitude = from._internal_latitude();
  uint64_t raw_latitude;
  memcpy(&raw_latitude, &tmp_latitude, sizeof(tmp_latitude));
  if (raw_latitude != 0) {
    _internal_set_latitude(from._internal_latitude());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_longitude = from._internal_longitude();
  uint64_t raw_longitude;
  memcpy(&raw_longitude, &tmp_longitude, sizeof(tmp_longitude));
  if (raw_longitude != 0) {
    _internal_set_longitude(from._internal_longitude());
  }
  static_assert(sizeof(uint64_t) == sizeof(double), "Code assumes uint64_t and double are the same size.");
  double tmp_altitude = from._internal_altitude();
  uint64_t raw_altitude;
  memcpy(&raw_altitude, &tmp_altitude, sizeof(tmp_altitude));
  if (raw_altitude != 0) {
    _internal_set_altitude(from._internal_altitude());
  }
  if (from._internal_position_covariance_type() != 0) {
    _internal_set_position_covariance_type(from._internal_position_covariance_type());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void NavSatFix::CopyFrom(const NavSatFix& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.sensor_msgs.NavSatFix)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool NavSatFix::IsInitialized() const {
  return true;
}

void NavSatFix::InternalSwap(NavSatFix* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  position_covariance_.InternalSwap(&other->position_covariance_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(NavSatFix, position_covariance_type_)
      + sizeof(NavSatFix::position_covariance_type_)
      - PROTOBUF_FIELD_OFFSET(NavSatFix, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata NavSatFix::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto_getter, &descriptor_table_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto_once,
      file_level_metadata_ros_2fsensor_5fmsgs_2fNavSatFix_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::sensor_msgs::NavSatFix* Arena::CreateMaybeMessage< ::ros::sensor_msgs::NavSatFix >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::sensor_msgs::NavSatFix >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>