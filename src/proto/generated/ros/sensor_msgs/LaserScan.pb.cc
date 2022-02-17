// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/LaserScan.proto

#include "ros/sensor_msgs/LaserScan.pb.h"

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
constexpr LaserScan::LaserScan(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : ranges_()
  , intensities_()
  , header_(nullptr)
  , angle_min_(0)
  , angle_max_(0)
  , angle_increment_(0)
  , time_increment_(0)
  , scan_time_(0)
  , range_min_(0)
  , range_max_(0){}
struct LaserScanDefaultTypeInternal {
  constexpr LaserScanDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~LaserScanDefaultTypeInternal() {}
  union {
    LaserScan _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT LaserScanDefaultTypeInternal _LaserScan_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fsensor_5fmsgs_2fLaserScan_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fLaserScan_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fsensor_5fmsgs_2fLaserScan_2eproto = nullptr;

const uint32_t TableStruct_ros_2fsensor_5fmsgs_2fLaserScan_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::LaserScan, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::LaserScan, header_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::LaserScan, angle_min_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::LaserScan, angle_max_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::LaserScan, angle_increment_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::LaserScan, time_increment_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::LaserScan, scan_time_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::LaserScan, range_min_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::LaserScan, range_max_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::LaserScan, ranges_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::LaserScan, intensities_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::sensor_msgs::LaserScan)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::sensor_msgs::_LaserScan_default_instance_),
};

const char descriptor_table_protodef_ros_2fsensor_5fmsgs_2fLaserScan_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\037ros/sensor_msgs/LaserScan.proto\022\017ros.s"
  "ensor_msgs\032\031ros/std_msgs/Header.proto\"\346\001"
  "\n\tLaserScan\022$\n\006header\030\001 \001(\0132\024.ros.std_ms"
  "gs.Header\022\021\n\tangle_min\030\002 \001(\002\022\021\n\tangle_ma"
  "x\030\003 \001(\002\022\027\n\017angle_increment\030\004 \001(\002\022\026\n\016time"
  "_increment\030\005 \001(\002\022\021\n\tscan_time\030\006 \001(\002\022\021\n\tr"
  "ange_min\030\007 \001(\002\022\021\n\trange_max\030\010 \001(\002\022\016\n\006ran"
  "ges\030\t \003(\002\022\023\n\013intensities\030\n \003(\002b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fsensor_5fmsgs_2fLaserScan_2eproto_deps[1] = {
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fsensor_5fmsgs_2fLaserScan_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fLaserScan_2eproto = {
  false, false, 318, descriptor_table_protodef_ros_2fsensor_5fmsgs_2fLaserScan_2eproto, "ros/sensor_msgs/LaserScan.proto", 
  &descriptor_table_ros_2fsensor_5fmsgs_2fLaserScan_2eproto_once, descriptor_table_ros_2fsensor_5fmsgs_2fLaserScan_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_ros_2fsensor_5fmsgs_2fLaserScan_2eproto::offsets,
  file_level_metadata_ros_2fsensor_5fmsgs_2fLaserScan_2eproto, file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fLaserScan_2eproto, file_level_service_descriptors_ros_2fsensor_5fmsgs_2fLaserScan_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fsensor_5fmsgs_2fLaserScan_2eproto_getter() {
  return &descriptor_table_ros_2fsensor_5fmsgs_2fLaserScan_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fsensor_5fmsgs_2fLaserScan_2eproto(&descriptor_table_ros_2fsensor_5fmsgs_2fLaserScan_2eproto);
namespace ros {
namespace sensor_msgs {

// ===================================================================

class LaserScan::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const LaserScan* msg);
};

const ::ros::std_msgs::Header&
LaserScan::_Internal::header(const LaserScan* msg) {
  return *msg->header_;
}
void LaserScan::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
LaserScan::LaserScan(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  ranges_(arena),
  intensities_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.sensor_msgs.LaserScan)
}
LaserScan::LaserScan(const LaserScan& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      ranges_(from.ranges_),
      intensities_(from.intensities_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  ::memcpy(&angle_min_, &from.angle_min_,
    static_cast<size_t>(reinterpret_cast<char*>(&range_max_) -
    reinterpret_cast<char*>(&angle_min_)) + sizeof(range_max_));
  // @@protoc_insertion_point(copy_constructor:ros.sensor_msgs.LaserScan)
}

inline void LaserScan::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&range_max_) -
    reinterpret_cast<char*>(&header_)) + sizeof(range_max_));
}

LaserScan::~LaserScan() {
  // @@protoc_insertion_point(destructor:ros.sensor_msgs.LaserScan)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void LaserScan::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
}

void LaserScan::ArenaDtor(void* object) {
  LaserScan* _this = reinterpret_cast< LaserScan* >(object);
  (void)_this;
}
void LaserScan::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void LaserScan::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void LaserScan::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.sensor_msgs.LaserScan)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ranges_.Clear();
  intensities_.Clear();
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  ::memset(&angle_min_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&range_max_) -
      reinterpret_cast<char*>(&angle_min_)) + sizeof(range_max_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* LaserScan::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // float angle_min = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 21)) {
          angle_min_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // float angle_max = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 29)) {
          angle_max_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // float angle_increment = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 37)) {
          angle_increment_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // float time_increment = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 45)) {
          time_increment_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // float scan_time = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 53)) {
          scan_time_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // float range_min = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 61)) {
          range_min_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // float range_max = 8;
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 69)) {
          range_max_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // repeated float ranges = 9;
      case 9:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 74)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedFloatParser(_internal_mutable_ranges(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 77) {
          _internal_add_ranges(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr));
          ptr += sizeof(float);
        } else
          goto handle_unusual;
        continue;
      // repeated float intensities = 10;
      case 10:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 82)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedFloatParser(_internal_mutable_intensities(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 85) {
          _internal_add_intensities(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr));
          ptr += sizeof(float);
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

uint8_t* LaserScan::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.sensor_msgs.LaserScan)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // float angle_min = 2;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_angle_min = this->_internal_angle_min();
  uint32_t raw_angle_min;
  memcpy(&raw_angle_min, &tmp_angle_min, sizeof(tmp_angle_min));
  if (raw_angle_min != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(2, this->_internal_angle_min(), target);
  }

  // float angle_max = 3;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_angle_max = this->_internal_angle_max();
  uint32_t raw_angle_max;
  memcpy(&raw_angle_max, &tmp_angle_max, sizeof(tmp_angle_max));
  if (raw_angle_max != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(3, this->_internal_angle_max(), target);
  }

  // float angle_increment = 4;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_angle_increment = this->_internal_angle_increment();
  uint32_t raw_angle_increment;
  memcpy(&raw_angle_increment, &tmp_angle_increment, sizeof(tmp_angle_increment));
  if (raw_angle_increment != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(4, this->_internal_angle_increment(), target);
  }

  // float time_increment = 5;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_time_increment = this->_internal_time_increment();
  uint32_t raw_time_increment;
  memcpy(&raw_time_increment, &tmp_time_increment, sizeof(tmp_time_increment));
  if (raw_time_increment != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(5, this->_internal_time_increment(), target);
  }

  // float scan_time = 6;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_scan_time = this->_internal_scan_time();
  uint32_t raw_scan_time;
  memcpy(&raw_scan_time, &tmp_scan_time, sizeof(tmp_scan_time));
  if (raw_scan_time != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(6, this->_internal_scan_time(), target);
  }

  // float range_min = 7;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_range_min = this->_internal_range_min();
  uint32_t raw_range_min;
  memcpy(&raw_range_min, &tmp_range_min, sizeof(tmp_range_min));
  if (raw_range_min != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(7, this->_internal_range_min(), target);
  }

  // float range_max = 8;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_range_max = this->_internal_range_max();
  uint32_t raw_range_max;
  memcpy(&raw_range_max, &tmp_range_max, sizeof(tmp_range_max));
  if (raw_range_max != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(8, this->_internal_range_max(), target);
  }

  // repeated float ranges = 9;
  if (this->_internal_ranges_size() > 0) {
    target = stream->WriteFixedPacked(9, _internal_ranges(), target);
  }

  // repeated float intensities = 10;
  if (this->_internal_intensities_size() > 0) {
    target = stream->WriteFixedPacked(10, _internal_intensities(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.sensor_msgs.LaserScan)
  return target;
}

size_t LaserScan::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.sensor_msgs.LaserScan)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated float ranges = 9;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_ranges_size());
    size_t data_size = 4UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
            static_cast<int32_t>(data_size));
    }
    total_size += data_size;
  }

  // repeated float intensities = 10;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_intensities_size());
    size_t data_size = 4UL * count;
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

  // float angle_min = 2;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_angle_min = this->_internal_angle_min();
  uint32_t raw_angle_min;
  memcpy(&raw_angle_min, &tmp_angle_min, sizeof(tmp_angle_min));
  if (raw_angle_min != 0) {
    total_size += 1 + 4;
  }

  // float angle_max = 3;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_angle_max = this->_internal_angle_max();
  uint32_t raw_angle_max;
  memcpy(&raw_angle_max, &tmp_angle_max, sizeof(tmp_angle_max));
  if (raw_angle_max != 0) {
    total_size += 1 + 4;
  }

  // float angle_increment = 4;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_angle_increment = this->_internal_angle_increment();
  uint32_t raw_angle_increment;
  memcpy(&raw_angle_increment, &tmp_angle_increment, sizeof(tmp_angle_increment));
  if (raw_angle_increment != 0) {
    total_size += 1 + 4;
  }

  // float time_increment = 5;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_time_increment = this->_internal_time_increment();
  uint32_t raw_time_increment;
  memcpy(&raw_time_increment, &tmp_time_increment, sizeof(tmp_time_increment));
  if (raw_time_increment != 0) {
    total_size += 1 + 4;
  }

  // float scan_time = 6;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_scan_time = this->_internal_scan_time();
  uint32_t raw_scan_time;
  memcpy(&raw_scan_time, &tmp_scan_time, sizeof(tmp_scan_time));
  if (raw_scan_time != 0) {
    total_size += 1 + 4;
  }

  // float range_min = 7;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_range_min = this->_internal_range_min();
  uint32_t raw_range_min;
  memcpy(&raw_range_min, &tmp_range_min, sizeof(tmp_range_min));
  if (raw_range_min != 0) {
    total_size += 1 + 4;
  }

  // float range_max = 8;
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_range_max = this->_internal_range_max();
  uint32_t raw_range_max;
  memcpy(&raw_range_max, &tmp_range_max, sizeof(tmp_range_max));
  if (raw_range_max != 0) {
    total_size += 1 + 4;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData LaserScan::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    LaserScan::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*LaserScan::GetClassData() const { return &_class_data_; }

void LaserScan::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<LaserScan *>(to)->MergeFrom(
      static_cast<const LaserScan &>(from));
}


void LaserScan::MergeFrom(const LaserScan& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.sensor_msgs.LaserScan)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  ranges_.MergeFrom(from.ranges_);
  intensities_.MergeFrom(from.intensities_);
  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_angle_min = from._internal_angle_min();
  uint32_t raw_angle_min;
  memcpy(&raw_angle_min, &tmp_angle_min, sizeof(tmp_angle_min));
  if (raw_angle_min != 0) {
    _internal_set_angle_min(from._internal_angle_min());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_angle_max = from._internal_angle_max();
  uint32_t raw_angle_max;
  memcpy(&raw_angle_max, &tmp_angle_max, sizeof(tmp_angle_max));
  if (raw_angle_max != 0) {
    _internal_set_angle_max(from._internal_angle_max());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_angle_increment = from._internal_angle_increment();
  uint32_t raw_angle_increment;
  memcpy(&raw_angle_increment, &tmp_angle_increment, sizeof(tmp_angle_increment));
  if (raw_angle_increment != 0) {
    _internal_set_angle_increment(from._internal_angle_increment());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_time_increment = from._internal_time_increment();
  uint32_t raw_time_increment;
  memcpy(&raw_time_increment, &tmp_time_increment, sizeof(tmp_time_increment));
  if (raw_time_increment != 0) {
    _internal_set_time_increment(from._internal_time_increment());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_scan_time = from._internal_scan_time();
  uint32_t raw_scan_time;
  memcpy(&raw_scan_time, &tmp_scan_time, sizeof(tmp_scan_time));
  if (raw_scan_time != 0) {
    _internal_set_scan_time(from._internal_scan_time());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_range_min = from._internal_range_min();
  uint32_t raw_range_min;
  memcpy(&raw_range_min, &tmp_range_min, sizeof(tmp_range_min));
  if (raw_range_min != 0) {
    _internal_set_range_min(from._internal_range_min());
  }
  static_assert(sizeof(uint32_t) == sizeof(float), "Code assumes uint32_t and float are the same size.");
  float tmp_range_max = from._internal_range_max();
  uint32_t raw_range_max;
  memcpy(&raw_range_max, &tmp_range_max, sizeof(tmp_range_max));
  if (raw_range_max != 0) {
    _internal_set_range_max(from._internal_range_max());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void LaserScan::CopyFrom(const LaserScan& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.sensor_msgs.LaserScan)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LaserScan::IsInitialized() const {
  return true;
}

void LaserScan::InternalSwap(LaserScan* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ranges_.InternalSwap(&other->ranges_);
  intensities_.InternalSwap(&other->intensities_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(LaserScan, range_max_)
      + sizeof(LaserScan::range_max_)
      - PROTOBUF_FIELD_OFFSET(LaserScan, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata LaserScan::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fsensor_5fmsgs_2fLaserScan_2eproto_getter, &descriptor_table_ros_2fsensor_5fmsgs_2fLaserScan_2eproto_once,
      file_level_metadata_ros_2fsensor_5fmsgs_2fLaserScan_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::sensor_msgs::LaserScan* Arena::CreateMaybeMessage< ::ros::sensor_msgs::LaserScan >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::sensor_msgs::LaserScan >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
