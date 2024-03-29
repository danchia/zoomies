// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/PointCloud2.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3019000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3019001 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "ros/sensor_msgs/PointField.pb.h"
#include "ros/std_msgs/Header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto;
namespace ros {
namespace sensor_msgs {
class PointCloud2;
struct PointCloud2DefaultTypeInternal;
extern PointCloud2DefaultTypeInternal _PointCloud2_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::sensor_msgs::PointCloud2* Arena::CreateMaybeMessage<::ros::sensor_msgs::PointCloud2>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace sensor_msgs {

// ===================================================================

class PointCloud2 final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.sensor_msgs.PointCloud2) */ {
 public:
  inline PointCloud2() : PointCloud2(nullptr) {}
  ~PointCloud2() override;
  explicit constexpr PointCloud2(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  PointCloud2(const PointCloud2& from);
  PointCloud2(PointCloud2&& from) noexcept
    : PointCloud2() {
    *this = ::std::move(from);
  }

  inline PointCloud2& operator=(const PointCloud2& from) {
    CopyFrom(from);
    return *this;
  }
  inline PointCloud2& operator=(PointCloud2&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const PointCloud2& default_instance() {
    return *internal_default_instance();
  }
  static inline const PointCloud2* internal_default_instance() {
    return reinterpret_cast<const PointCloud2*>(
               &_PointCloud2_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PointCloud2& a, PointCloud2& b) {
    a.Swap(&b);
  }
  inline void Swap(PointCloud2* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(PointCloud2* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  PointCloud2* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<PointCloud2>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const PointCloud2& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const PointCloud2& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(PointCloud2* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.sensor_msgs.PointCloud2";
  }
  protected:
  explicit PointCloud2(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kFieldsFieldNumber = 4,
    kDataFieldNumber = 8,
    kHeaderFieldNumber = 1,
    kHeightFieldNumber = 2,
    kWidthFieldNumber = 3,
    kPointStepFieldNumber = 6,
    kRowStepFieldNumber = 7,
    kIsBigendianFieldNumber = 5,
    kIsDenseFieldNumber = 9,
  };
  // repeated .ros.sensor_msgs.PointField fields = 4;
  int fields_size() const;
  private:
  int _internal_fields_size() const;
  public:
  void clear_fields();
  ::ros::sensor_msgs::PointField* mutable_fields(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::sensor_msgs::PointField >*
      mutable_fields();
  private:
  const ::ros::sensor_msgs::PointField& _internal_fields(int index) const;
  ::ros::sensor_msgs::PointField* _internal_add_fields();
  public:
  const ::ros::sensor_msgs::PointField& fields(int index) const;
  ::ros::sensor_msgs::PointField* add_fields();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::sensor_msgs::PointField >&
      fields() const;

  // bytes data = 8;
  void clear_data();
  const std::string& data() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_data(ArgT0&& arg0, ArgT... args);
  std::string* mutable_data();
  PROTOBUF_NODISCARD std::string* release_data();
  void set_allocated_data(std::string* data);
  private:
  const std::string& _internal_data() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_data(const std::string& value);
  std::string* _internal_mutable_data();
  public:

  // .ros.std_msgs.Header header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::ros::std_msgs::Header& header() const;
  PROTOBUF_NODISCARD ::ros::std_msgs::Header* release_header();
  ::ros::std_msgs::Header* mutable_header();
  void set_allocated_header(::ros::std_msgs::Header* header);
  private:
  const ::ros::std_msgs::Header& _internal_header() const;
  ::ros::std_msgs::Header* _internal_mutable_header();
  public:
  void unsafe_arena_set_allocated_header(
      ::ros::std_msgs::Header* header);
  ::ros::std_msgs::Header* unsafe_arena_release_header();

  // uint32 height = 2;
  void clear_height();
  uint32_t height() const;
  void set_height(uint32_t value);
  private:
  uint32_t _internal_height() const;
  void _internal_set_height(uint32_t value);
  public:

  // uint32 width = 3;
  void clear_width();
  uint32_t width() const;
  void set_width(uint32_t value);
  private:
  uint32_t _internal_width() const;
  void _internal_set_width(uint32_t value);
  public:

  // uint32 point_step = 6;
  void clear_point_step();
  uint32_t point_step() const;
  void set_point_step(uint32_t value);
  private:
  uint32_t _internal_point_step() const;
  void _internal_set_point_step(uint32_t value);
  public:

  // uint32 row_step = 7;
  void clear_row_step();
  uint32_t row_step() const;
  void set_row_step(uint32_t value);
  private:
  uint32_t _internal_row_step() const;
  void _internal_set_row_step(uint32_t value);
  public:

  // bool is_bigendian = 5;
  void clear_is_bigendian();
  bool is_bigendian() const;
  void set_is_bigendian(bool value);
  private:
  bool _internal_is_bigendian() const;
  void _internal_set_is_bigendian(bool value);
  public:

  // bool is_dense = 9;
  void clear_is_dense();
  bool is_dense() const;
  void set_is_dense(bool value);
  private:
  bool _internal_is_dense() const;
  void _internal_set_is_dense(bool value);
  public:

  // @@protoc_insertion_point(class_scope:ros.sensor_msgs.PointCloud2)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::sensor_msgs::PointField > fields_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr data_;
  ::ros::std_msgs::Header* header_;
  uint32_t height_;
  uint32_t width_;
  uint32_t point_step_;
  uint32_t row_step_;
  bool is_bigendian_;
  bool is_dense_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PointCloud2

// .ros.std_msgs.Header header = 1;
inline bool PointCloud2::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool PointCloud2::has_header() const {
  return _internal_has_header();
}
inline const ::ros::std_msgs::Header& PointCloud2::_internal_header() const {
  const ::ros::std_msgs::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::std_msgs::Header&>(
      ::ros::std_msgs::_Header_default_instance_);
}
inline const ::ros::std_msgs::Header& PointCloud2::header() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointCloud2.header)
  return _internal_header();
}
inline void PointCloud2::unsafe_arena_set_allocated_header(
    ::ros::std_msgs::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.sensor_msgs.PointCloud2.header)
}
inline ::ros::std_msgs::Header* PointCloud2::release_header() {
  
  ::ros::std_msgs::Header* temp = header_;
  header_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::ros::std_msgs::Header* PointCloud2::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:ros.sensor_msgs.PointCloud2.header)
  
  ::ros::std_msgs::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::ros::std_msgs::Header* PointCloud2::_internal_mutable_header() {
  
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::std_msgs::Header>(GetArenaForAllocation());
    header_ = p;
  }
  return header_;
}
inline ::ros::std_msgs::Header* PointCloud2::mutable_header() {
  ::ros::std_msgs::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.PointCloud2.header)
  return _msg;
}
inline void PointCloud2::set_allocated_header(::ros::std_msgs::Header* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header));
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:ros.sensor_msgs.PointCloud2.header)
}

// uint32 height = 2;
inline void PointCloud2::clear_height() {
  height_ = 0u;
}
inline uint32_t PointCloud2::_internal_height() const {
  return height_;
}
inline uint32_t PointCloud2::height() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointCloud2.height)
  return _internal_height();
}
inline void PointCloud2::_internal_set_height(uint32_t value) {
  
  height_ = value;
}
inline void PointCloud2::set_height(uint32_t value) {
  _internal_set_height(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.PointCloud2.height)
}

// uint32 width = 3;
inline void PointCloud2::clear_width() {
  width_ = 0u;
}
inline uint32_t PointCloud2::_internal_width() const {
  return width_;
}
inline uint32_t PointCloud2::width() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointCloud2.width)
  return _internal_width();
}
inline void PointCloud2::_internal_set_width(uint32_t value) {
  
  width_ = value;
}
inline void PointCloud2::set_width(uint32_t value) {
  _internal_set_width(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.PointCloud2.width)
}

// repeated .ros.sensor_msgs.PointField fields = 4;
inline int PointCloud2::_internal_fields_size() const {
  return fields_.size();
}
inline int PointCloud2::fields_size() const {
  return _internal_fields_size();
}
inline ::ros::sensor_msgs::PointField* PointCloud2::mutable_fields(int index) {
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.PointCloud2.fields)
  return fields_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::sensor_msgs::PointField >*
PointCloud2::mutable_fields() {
  // @@protoc_insertion_point(field_mutable_list:ros.sensor_msgs.PointCloud2.fields)
  return &fields_;
}
inline const ::ros::sensor_msgs::PointField& PointCloud2::_internal_fields(int index) const {
  return fields_.Get(index);
}
inline const ::ros::sensor_msgs::PointField& PointCloud2::fields(int index) const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointCloud2.fields)
  return _internal_fields(index);
}
inline ::ros::sensor_msgs::PointField* PointCloud2::_internal_add_fields() {
  return fields_.Add();
}
inline ::ros::sensor_msgs::PointField* PointCloud2::add_fields() {
  ::ros::sensor_msgs::PointField* _add = _internal_add_fields();
  // @@protoc_insertion_point(field_add:ros.sensor_msgs.PointCloud2.fields)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::sensor_msgs::PointField >&
PointCloud2::fields() const {
  // @@protoc_insertion_point(field_list:ros.sensor_msgs.PointCloud2.fields)
  return fields_;
}

// bool is_bigendian = 5;
inline void PointCloud2::clear_is_bigendian() {
  is_bigendian_ = false;
}
inline bool PointCloud2::_internal_is_bigendian() const {
  return is_bigendian_;
}
inline bool PointCloud2::is_bigendian() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointCloud2.is_bigendian)
  return _internal_is_bigendian();
}
inline void PointCloud2::_internal_set_is_bigendian(bool value) {
  
  is_bigendian_ = value;
}
inline void PointCloud2::set_is_bigendian(bool value) {
  _internal_set_is_bigendian(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.PointCloud2.is_bigendian)
}

// uint32 point_step = 6;
inline void PointCloud2::clear_point_step() {
  point_step_ = 0u;
}
inline uint32_t PointCloud2::_internal_point_step() const {
  return point_step_;
}
inline uint32_t PointCloud2::point_step() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointCloud2.point_step)
  return _internal_point_step();
}
inline void PointCloud2::_internal_set_point_step(uint32_t value) {
  
  point_step_ = value;
}
inline void PointCloud2::set_point_step(uint32_t value) {
  _internal_set_point_step(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.PointCloud2.point_step)
}

// uint32 row_step = 7;
inline void PointCloud2::clear_row_step() {
  row_step_ = 0u;
}
inline uint32_t PointCloud2::_internal_row_step() const {
  return row_step_;
}
inline uint32_t PointCloud2::row_step() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointCloud2.row_step)
  return _internal_row_step();
}
inline void PointCloud2::_internal_set_row_step(uint32_t value) {
  
  row_step_ = value;
}
inline void PointCloud2::set_row_step(uint32_t value) {
  _internal_set_row_step(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.PointCloud2.row_step)
}

// bytes data = 8;
inline void PointCloud2::clear_data() {
  data_.ClearToEmpty();
}
inline const std::string& PointCloud2::data() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointCloud2.data)
  return _internal_data();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void PointCloud2::set_data(ArgT0&& arg0, ArgT... args) {
 
 data_.SetBytes(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.PointCloud2.data)
}
inline std::string* PointCloud2::mutable_data() {
  std::string* _s = _internal_mutable_data();
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.PointCloud2.data)
  return _s;
}
inline const std::string& PointCloud2::_internal_data() const {
  return data_.Get();
}
inline void PointCloud2::_internal_set_data(const std::string& value) {
  
  data_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* PointCloud2::_internal_mutable_data() {
  
  return data_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* PointCloud2::release_data() {
  // @@protoc_insertion_point(field_release:ros.sensor_msgs.PointCloud2.data)
  return data_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void PointCloud2::set_allocated_data(std::string* data) {
  if (data != nullptr) {
    
  } else {
    
  }
  data_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), data,
      GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (data_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    data_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:ros.sensor_msgs.PointCloud2.data)
}

// bool is_dense = 9;
inline void PointCloud2::clear_is_dense() {
  is_dense_ = false;
}
inline bool PointCloud2::_internal_is_dense() const {
  return is_dense_;
}
inline bool PointCloud2::is_dense() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointCloud2.is_dense)
  return _internal_is_dense();
}
inline void PointCloud2::_internal_set_is_dense(bool value) {
  
  is_dense_ = value;
}
inline void PointCloud2::set_is_dense(bool value) {
  _internal_set_is_dense(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.PointCloud2.is_dense)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace sensor_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fPointCloud2_2eproto
