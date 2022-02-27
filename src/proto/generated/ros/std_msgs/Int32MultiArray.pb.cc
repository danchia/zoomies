// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/std_msgs/Int32MultiArray.proto

#include "ros/std_msgs/Int32MultiArray.pb.h"

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
namespace std_msgs {
constexpr Int32MultiArray::Int32MultiArray(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : data_()
  , _data_cached_byte_size_(0)
  , layout_(nullptr){}
struct Int32MultiArrayDefaultTypeInternal {
  constexpr Int32MultiArrayDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~Int32MultiArrayDefaultTypeInternal() {}
  union {
    Int32MultiArray _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT Int32MultiArrayDefaultTypeInternal _Int32MultiArray_default_instance_;
}  // namespace std_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto = nullptr;

const uint32_t TableStruct_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::std_msgs::Int32MultiArray, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::std_msgs::Int32MultiArray, layout_),
  PROTOBUF_FIELD_OFFSET(::ros::std_msgs::Int32MultiArray, data_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::std_msgs::Int32MultiArray)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::std_msgs::_Int32MultiArray_default_instance_),
};

const char descriptor_table_protodef_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\"ros/std_msgs/Int32MultiArray.proto\022\014ro"
  "s.std_msgs\032#ros/std_msgs/MultiArrayLayou"
  "t.proto\"O\n\017Int32MultiArray\022.\n\006layout\030\001 \001"
  "(\0132\036.ros.std_msgs.MultiArrayLayout\022\014\n\004da"
  "ta\030\002 \003(\005b\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto_deps[1] = {
  &::descriptor_table_ros_2fstd_5fmsgs_2fMultiArrayLayout_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto = {
  false, false, 176, descriptor_table_protodef_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto, "ros/std_msgs/Int32MultiArray.proto", 
  &descriptor_table_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto_once, descriptor_table_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto_deps, 1, 1,
  schemas, file_default_instances, TableStruct_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto::offsets,
  file_level_metadata_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto, file_level_enum_descriptors_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto, file_level_service_descriptors_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto_getter() {
  return &descriptor_table_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto(&descriptor_table_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto);
namespace ros {
namespace std_msgs {

// ===================================================================

class Int32MultiArray::_Internal {
 public:
  static const ::ros::std_msgs::MultiArrayLayout& layout(const Int32MultiArray* msg);
};

const ::ros::std_msgs::MultiArrayLayout&
Int32MultiArray::_Internal::layout(const Int32MultiArray* msg) {
  return *msg->layout_;
}
void Int32MultiArray::clear_layout() {
  if (GetArenaForAllocation() == nullptr && layout_ != nullptr) {
    delete layout_;
  }
  layout_ = nullptr;
}
Int32MultiArray::Int32MultiArray(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  data_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.std_msgs.Int32MultiArray)
}
Int32MultiArray::Int32MultiArray(const Int32MultiArray& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      data_(from.data_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_layout()) {
    layout_ = new ::ros::std_msgs::MultiArrayLayout(*from.layout_);
  } else {
    layout_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:ros.std_msgs.Int32MultiArray)
}

inline void Int32MultiArray::SharedCtor() {
layout_ = nullptr;
}

Int32MultiArray::~Int32MultiArray() {
  // @@protoc_insertion_point(destructor:ros.std_msgs.Int32MultiArray)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Int32MultiArray::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete layout_;
}

void Int32MultiArray::ArenaDtor(void* object) {
  Int32MultiArray* _this = reinterpret_cast< Int32MultiArray* >(object);
  (void)_this;
}
void Int32MultiArray::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Int32MultiArray::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Int32MultiArray::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.std_msgs.Int32MultiArray)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  data_.Clear();
  if (GetArenaForAllocation() == nullptr && layout_ != nullptr) {
    delete layout_;
  }
  layout_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Int32MultiArray::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .ros.std_msgs.MultiArrayLayout layout = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_layout(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated int32 data = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedInt32Parser(_internal_mutable_data(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 16) {
          _internal_add_data(::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr));
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

uint8_t* Int32MultiArray::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.std_msgs.Int32MultiArray)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .ros.std_msgs.MultiArrayLayout layout = 1;
  if (this->_internal_has_layout()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        1, _Internal::layout(this), target, stream);
  }

  // repeated int32 data = 2;
  {
    int byte_size = _data_cached_byte_size_.load(std::memory_order_relaxed);
    if (byte_size > 0) {
      target = stream->WriteInt32Packed(
          2, _internal_data(), byte_size, target);
    }
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.std_msgs.Int32MultiArray)
  return target;
}

size_t Int32MultiArray::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.std_msgs.Int32MultiArray)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated int32 data = 2;
  {
    size_t data_size = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      Int32Size(this->data_);
    if (data_size > 0) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
            static_cast<int32_t>(data_size));
    }
    int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(data_size);
    _data_cached_byte_size_.store(cached_size,
                                    std::memory_order_relaxed);
    total_size += data_size;
  }

  // .ros.std_msgs.MultiArrayLayout layout = 1;
  if (this->_internal_has_layout()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *layout_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Int32MultiArray::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Int32MultiArray::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Int32MultiArray::GetClassData() const { return &_class_data_; }

void Int32MultiArray::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Int32MultiArray *>(to)->MergeFrom(
      static_cast<const Int32MultiArray &>(from));
}


void Int32MultiArray::MergeFrom(const Int32MultiArray& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.std_msgs.Int32MultiArray)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  data_.MergeFrom(from.data_);
  if (from._internal_has_layout()) {
    _internal_mutable_layout()->::ros::std_msgs::MultiArrayLayout::MergeFrom(from._internal_layout());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Int32MultiArray::CopyFrom(const Int32MultiArray& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.std_msgs.Int32MultiArray)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Int32MultiArray::IsInitialized() const {
  return true;
}

void Int32MultiArray::InternalSwap(Int32MultiArray* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  data_.InternalSwap(&other->data_);
  swap(layout_, other->layout_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Int32MultiArray::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto_getter, &descriptor_table_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto_once,
      file_level_metadata_ros_2fstd_5fmsgs_2fInt32MultiArray_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace std_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::std_msgs::Int32MultiArray* Arena::CreateMaybeMessage< ::ros::std_msgs::Int32MultiArray >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::std_msgs::Int32MultiArray >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
