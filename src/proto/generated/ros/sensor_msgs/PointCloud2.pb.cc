// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/PointCloud2.proto

#include "ros/sensor_msgs/PointCloud2.pb.h"

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
constexpr PointCloud2::PointCloud2(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : fields_()
  , data_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , header_(nullptr)
  , height_(0u)
  , width_(0u)
  , point_step_(0u)
  , row_step_(0u)
  , is_bigendian_(false)
  , is_dense_(false){}
struct PointCloud2DefaultTypeInternal {
  constexpr PointCloud2DefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~PointCloud2DefaultTypeInternal() {}
  union {
    PointCloud2 _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PointCloud2DefaultTypeInternal _PointCloud2_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto = nullptr;

const uint32_t TableStruct_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::PointCloud2, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::PointCloud2, header_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::PointCloud2, height_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::PointCloud2, width_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::PointCloud2, fields_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::PointCloud2, is_bigendian_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::PointCloud2, point_step_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::PointCloud2, row_step_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::PointCloud2, data_),
  PROTOBUF_FIELD_OFFSET(::ros::sensor_msgs::PointCloud2, is_dense_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::sensor_msgs::PointCloud2)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::sensor_msgs::_PointCloud2_default_instance_),
};

const char descriptor_table_protodef_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n!ros/sensor_msgs/PointCloud2.proto\022\017ros"
  ".sensor_msgs\032 ros/sensor_msgs/PointField"
  ".proto\032\031ros/std_msgs/Header.proto\"\333\001\n\013Po"
  "intCloud2\022$\n\006header\030\001 \001(\0132\024.ros.std_msgs"
  ".Header\022\016\n\006height\030\002 \001(\r\022\r\n\005width\030\003 \001(\r\022+"
  "\n\006fields\030\004 \003(\0132\033.ros.sensor_msgs.PointFi"
  "eld\022\024\n\014is_bigendian\030\005 \001(\010\022\022\n\npoint_step\030"
  "\006 \001(\r\022\020\n\010row_step\030\007 \001(\r\022\014\n\004data\030\010 \001(\014\022\020\n"
  "\010is_dense\030\t \001(\010b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto_deps[2] = {
  &::descriptor_table_ros_2fsensor_5fmsgs_2fPointField_2eproto,
  &::descriptor_table_ros_2fstd_5fmsgs_2fHeader_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto = {
  false, false, 343, descriptor_table_protodef_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto, "ros/sensor_msgs/PointCloud2.proto", 
  &descriptor_table_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto_once, descriptor_table_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto::offsets,
  file_level_metadata_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto, file_level_enum_descriptors_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto, file_level_service_descriptors_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto_getter() {
  return &descriptor_table_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto(&descriptor_table_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto);
namespace ros {
namespace sensor_msgs {

// ===================================================================

class PointCloud2::_Internal {
 public:
  static const ::ros::std_msgs::Header& header(const PointCloud2* msg);
};

const ::ros::std_msgs::Header&
PointCloud2::_Internal::header(const PointCloud2* msg) {
  return *msg->header_;
}
void PointCloud2::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void PointCloud2::clear_fields() {
  fields_.Clear();
}
PointCloud2::PointCloud2(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  fields_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.sensor_msgs.PointCloud2)
}
PointCloud2::PointCloud2(const PointCloud2& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      fields_(from.fields_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  data_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    data_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (!from._internal_data().empty()) {
    data_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_data(), 
      GetArenaForAllocation());
  }
  if (from._internal_has_header()) {
    header_ = new ::ros::std_msgs::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  ::memcpy(&height_, &from.height_,
    static_cast<size_t>(reinterpret_cast<char*>(&is_dense_) -
    reinterpret_cast<char*>(&height_)) + sizeof(is_dense_));
  // @@protoc_insertion_point(copy_constructor:ros.sensor_msgs.PointCloud2)
}

inline void PointCloud2::SharedCtor() {
data_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  data_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&is_dense_) -
    reinterpret_cast<char*>(&header_)) + sizeof(is_dense_));
}

PointCloud2::~PointCloud2() {
  // @@protoc_insertion_point(destructor:ros.sensor_msgs.PointCloud2)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void PointCloud2::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  data_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete header_;
}

void PointCloud2::ArenaDtor(void* object) {
  PointCloud2* _this = reinterpret_cast< PointCloud2* >(object);
  (void)_this;
}
void PointCloud2::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void PointCloud2::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void PointCloud2::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.sensor_msgs.PointCloud2)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  fields_.Clear();
  data_.ClearToEmpty();
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  ::memset(&height_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&is_dense_) -
      reinterpret_cast<char*>(&height_)) + sizeof(is_dense_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* PointCloud2::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
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
      // uint32 height = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 16)) {
          height_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // uint32 width = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 24)) {
          width_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .ros.sensor_msgs.PointField fields = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 34)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_fields(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<34>(ptr));
        } else
          goto handle_unusual;
        continue;
      // bool is_bigendian = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 40)) {
          is_bigendian_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // uint32 point_step = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 48)) {
          point_step_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // uint32 row_step = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 56)) {
          row_step_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // bytes data = 8;
      case 8:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 66)) {
          auto str = _internal_mutable_data();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // bool is_dense = 9;
      case 9:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 72)) {
          is_dense_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
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

uint8_t* PointCloud2::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.sensor_msgs.PointCloud2)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::header(this), target, stream);
  }

  // uint32 height = 2;
  if (this->_internal_height() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(2, this->_internal_height(), target);
  }

  // uint32 width = 3;
  if (this->_internal_width() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(3, this->_internal_width(), target);
  }

  // repeated .ros.sensor_msgs.PointField fields = 4;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_fields_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(4, this->_internal_fields(i), target, stream);
  }

  // bool is_bigendian = 5;
  if (this->_internal_is_bigendian() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(5, this->_internal_is_bigendian(), target);
  }

  // uint32 point_step = 6;
  if (this->_internal_point_step() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(6, this->_internal_point_step(), target);
  }

  // uint32 row_step = 7;
  if (this->_internal_row_step() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(7, this->_internal_row_step(), target);
  }

  // bytes data = 8;
  if (!this->_internal_data().empty()) {
    target = stream->WriteBytesMaybeAliased(
        8, this->_internal_data(), target);
  }

  // bool is_dense = 9;
  if (this->_internal_is_dense() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(9, this->_internal_is_dense(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.sensor_msgs.PointCloud2)
  return target;
}

size_t PointCloud2::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.sensor_msgs.PointCloud2)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .ros.sensor_msgs.PointField fields = 4;
  total_size += 1UL * this->_internal_fields_size();
  for (const auto& msg : this->fields_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // bytes data = 8;
  if (!this->_internal_data().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::BytesSize(
        this->_internal_data());
  }

  // .ros.std_msgs.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  // uint32 height = 2;
  if (this->_internal_height() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32SizePlusOne(this->_internal_height());
  }

  // uint32 width = 3;
  if (this->_internal_width() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32SizePlusOne(this->_internal_width());
  }

  // uint32 point_step = 6;
  if (this->_internal_point_step() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32SizePlusOne(this->_internal_point_step());
  }

  // uint32 row_step = 7;
  if (this->_internal_row_step() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32SizePlusOne(this->_internal_row_step());
  }

  // bool is_bigendian = 5;
  if (this->_internal_is_bigendian() != 0) {
    total_size += 1 + 1;
  }

  // bool is_dense = 9;
  if (this->_internal_is_dense() != 0) {
    total_size += 1 + 1;
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData PointCloud2::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    PointCloud2::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*PointCloud2::GetClassData() const { return &_class_data_; }

void PointCloud2::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<PointCloud2 *>(to)->MergeFrom(
      static_cast<const PointCloud2 &>(from));
}


void PointCloud2::MergeFrom(const PointCloud2& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.sensor_msgs.PointCloud2)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  fields_.MergeFrom(from.fields_);
  if (!from._internal_data().empty()) {
    _internal_set_data(from._internal_data());
  }
  if (from._internal_has_header()) {
    _internal_mutable_header()->::ros::std_msgs::Header::MergeFrom(from._internal_header());
  }
  if (from._internal_height() != 0) {
    _internal_set_height(from._internal_height());
  }
  if (from._internal_width() != 0) {
    _internal_set_width(from._internal_width());
  }
  if (from._internal_point_step() != 0) {
    _internal_set_point_step(from._internal_point_step());
  }
  if (from._internal_row_step() != 0) {
    _internal_set_row_step(from._internal_row_step());
  }
  if (from._internal_is_bigendian() != 0) {
    _internal_set_is_bigendian(from._internal_is_bigendian());
  }
  if (from._internal_is_dense() != 0) {
    _internal_set_is_dense(from._internal_is_dense());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void PointCloud2::CopyFrom(const PointCloud2& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.sensor_msgs.PointCloud2)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PointCloud2::IsInitialized() const {
  return true;
}

void PointCloud2::InternalSwap(PointCloud2* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  fields_.InternalSwap(&other->fields_);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &data_, lhs_arena,
      &other->data_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(PointCloud2, is_dense_)
      + sizeof(PointCloud2::is_dense_)
      - PROTOBUF_FIELD_OFFSET(PointCloud2, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata PointCloud2::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto_getter, &descriptor_table_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto_once,
      file_level_metadata_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::sensor_msgs::PointCloud2* Arena::CreateMaybeMessage< ::ros::sensor_msgs::PointCloud2 >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::sensor_msgs::PointCloud2 >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
