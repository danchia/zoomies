// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/PointField.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fPointField_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fPointField_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fsensor_5fmsgs_2fPointField_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fsensor_5fmsgs_2fPointField_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fPointField_2eproto;
namespace ros {
namespace sensor_msgs {
class PointField;
struct PointFieldDefaultTypeInternal;
extern PointFieldDefaultTypeInternal _PointField_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::sensor_msgs::PointField* Arena::CreateMaybeMessage<::ros::sensor_msgs::PointField>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace sensor_msgs {

// ===================================================================

class PointField final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.sensor_msgs.PointField) */ {
 public:
  inline PointField() : PointField(nullptr) {}
  ~PointField() override;
  explicit constexpr PointField(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  PointField(const PointField& from);
  PointField(PointField&& from) noexcept
    : PointField() {
    *this = ::std::move(from);
  }

  inline PointField& operator=(const PointField& from) {
    CopyFrom(from);
    return *this;
  }
  inline PointField& operator=(PointField&& from) noexcept {
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
  static const PointField& default_instance() {
    return *internal_default_instance();
  }
  static inline const PointField* internal_default_instance() {
    return reinterpret_cast<const PointField*>(
               &_PointField_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PointField& a, PointField& b) {
    a.Swap(&b);
  }
  inline void Swap(PointField* other) {
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
  void UnsafeArenaSwap(PointField* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  PointField* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<PointField>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const PointField& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const PointField& from);
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
  void InternalSwap(PointField* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.sensor_msgs.PointField";
  }
  protected:
  explicit PointField(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kNameFieldNumber = 1,
    kOffsetFieldNumber = 2,
    kDatatypeFieldNumber = 3,
    kCountFieldNumber = 4,
  };
  // string name = 1;
  void clear_name();
  const std::string& name() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_name(ArgT0&& arg0, ArgT... args);
  std::string* mutable_name();
  PROTOBUF_NODISCARD std::string* release_name();
  void set_allocated_name(std::string* name);
  private:
  const std::string& _internal_name() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_name(const std::string& value);
  std::string* _internal_mutable_name();
  public:

  // uint32 offset = 2;
  void clear_offset();
  uint32_t offset() const;
  void set_offset(uint32_t value);
  private:
  uint32_t _internal_offset() const;
  void _internal_set_offset(uint32_t value);
  public:

  // int32 datatype = 3;
  void clear_datatype();
  int32_t datatype() const;
  void set_datatype(int32_t value);
  private:
  int32_t _internal_datatype() const;
  void _internal_set_datatype(int32_t value);
  public:

  // uint32 count = 4;
  void clear_count();
  uint32_t count() const;
  void set_count(uint32_t value);
  private:
  uint32_t _internal_count() const;
  void _internal_set_count(uint32_t value);
  public:

  // @@protoc_insertion_point(class_scope:ros.sensor_msgs.PointField)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr name_;
  uint32_t offset_;
  int32_t datatype_;
  uint32_t count_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fsensor_5fmsgs_2fPointField_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PointField

// string name = 1;
inline void PointField::clear_name() {
  name_.ClearToEmpty();
}
inline const std::string& PointField::name() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointField.name)
  return _internal_name();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void PointField::set_name(ArgT0&& arg0, ArgT... args) {
 
 name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.PointField.name)
}
inline std::string* PointField::mutable_name() {
  std::string* _s = _internal_mutable_name();
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.PointField.name)
  return _s;
}
inline const std::string& PointField::_internal_name() const {
  return name_.Get();
}
inline void PointField::_internal_set_name(const std::string& value) {
  
  name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* PointField::_internal_mutable_name() {
  
  return name_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* PointField::release_name() {
  // @@protoc_insertion_point(field_release:ros.sensor_msgs.PointField.name)
  return name_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void PointField::set_allocated_name(std::string* name) {
  if (name != nullptr) {
    
  } else {
    
  }
  name_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), name,
      GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (name_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    name_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:ros.sensor_msgs.PointField.name)
}

// uint32 offset = 2;
inline void PointField::clear_offset() {
  offset_ = 0u;
}
inline uint32_t PointField::_internal_offset() const {
  return offset_;
}
inline uint32_t PointField::offset() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointField.offset)
  return _internal_offset();
}
inline void PointField::_internal_set_offset(uint32_t value) {
  
  offset_ = value;
}
inline void PointField::set_offset(uint32_t value) {
  _internal_set_offset(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.PointField.offset)
}

// int32 datatype = 3;
inline void PointField::clear_datatype() {
  datatype_ = 0;
}
inline int32_t PointField::_internal_datatype() const {
  return datatype_;
}
inline int32_t PointField::datatype() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointField.datatype)
  return _internal_datatype();
}
inline void PointField::_internal_set_datatype(int32_t value) {
  
  datatype_ = value;
}
inline void PointField::set_datatype(int32_t value) {
  _internal_set_datatype(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.PointField.datatype)
}

// uint32 count = 4;
inline void PointField::clear_count() {
  count_ = 0u;
}
inline uint32_t PointField::_internal_count() const {
  return count_;
}
inline uint32_t PointField::count() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.PointField.count)
  return _internal_count();
}
inline void PointField::_internal_set_count(uint32_t value) {
  
  count_ = value;
}
inline void PointField::set_count(uint32_t value) {
  _internal_set_count(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.PointField.count)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace sensor_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fPointField_2eproto
