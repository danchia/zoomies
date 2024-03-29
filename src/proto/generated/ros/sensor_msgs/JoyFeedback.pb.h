// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/JoyFeedback.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fJoyFeedback_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fJoyFeedback_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ros_2fsensor_5fmsgs_2fJoyFeedback_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fsensor_5fmsgs_2fJoyFeedback_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fJoyFeedback_2eproto;
namespace ros {
namespace sensor_msgs {
class JoyFeedback;
struct JoyFeedbackDefaultTypeInternal;
extern JoyFeedbackDefaultTypeInternal _JoyFeedback_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::sensor_msgs::JoyFeedback* Arena::CreateMaybeMessage<::ros::sensor_msgs::JoyFeedback>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace sensor_msgs {

// ===================================================================

class JoyFeedback final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.sensor_msgs.JoyFeedback) */ {
 public:
  inline JoyFeedback() : JoyFeedback(nullptr) {}
  ~JoyFeedback() override;
  explicit constexpr JoyFeedback(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  JoyFeedback(const JoyFeedback& from);
  JoyFeedback(JoyFeedback&& from) noexcept
    : JoyFeedback() {
    *this = ::std::move(from);
  }

  inline JoyFeedback& operator=(const JoyFeedback& from) {
    CopyFrom(from);
    return *this;
  }
  inline JoyFeedback& operator=(JoyFeedback&& from) noexcept {
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
  static const JoyFeedback& default_instance() {
    return *internal_default_instance();
  }
  static inline const JoyFeedback* internal_default_instance() {
    return reinterpret_cast<const JoyFeedback*>(
               &_JoyFeedback_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(JoyFeedback& a, JoyFeedback& b) {
    a.Swap(&b);
  }
  inline void Swap(JoyFeedback* other) {
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
  void UnsafeArenaSwap(JoyFeedback* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  JoyFeedback* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<JoyFeedback>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const JoyFeedback& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const JoyFeedback& from);
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
  void InternalSwap(JoyFeedback* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.sensor_msgs.JoyFeedback";
  }
  protected:
  explicit JoyFeedback(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kTypeFieldNumber = 1,
    kIdFieldNumber = 2,
    kIntensityFieldNumber = 3,
  };
  // int32 type = 1;
  void clear_type();
  int32_t type() const;
  void set_type(int32_t value);
  private:
  int32_t _internal_type() const;
  void _internal_set_type(int32_t value);
  public:

  // int32 id = 2;
  void clear_id();
  int32_t id() const;
  void set_id(int32_t value);
  private:
  int32_t _internal_id() const;
  void _internal_set_id(int32_t value);
  public:

  // float intensity = 3;
  void clear_intensity();
  float intensity() const;
  void set_intensity(float value);
  private:
  float _internal_intensity() const;
  void _internal_set_intensity(float value);
  public:

  // @@protoc_insertion_point(class_scope:ros.sensor_msgs.JoyFeedback)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  int32_t type_;
  int32_t id_;
  float intensity_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fsensor_5fmsgs_2fJoyFeedback_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// JoyFeedback

// int32 type = 1;
inline void JoyFeedback::clear_type() {
  type_ = 0;
}
inline int32_t JoyFeedback::_internal_type() const {
  return type_;
}
inline int32_t JoyFeedback::type() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.JoyFeedback.type)
  return _internal_type();
}
inline void JoyFeedback::_internal_set_type(int32_t value) {
  
  type_ = value;
}
inline void JoyFeedback::set_type(int32_t value) {
  _internal_set_type(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.JoyFeedback.type)
}

// int32 id = 2;
inline void JoyFeedback::clear_id() {
  id_ = 0;
}
inline int32_t JoyFeedback::_internal_id() const {
  return id_;
}
inline int32_t JoyFeedback::id() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.JoyFeedback.id)
  return _internal_id();
}
inline void JoyFeedback::_internal_set_id(int32_t value) {
  
  id_ = value;
}
inline void JoyFeedback::set_id(int32_t value) {
  _internal_set_id(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.JoyFeedback.id)
}

// float intensity = 3;
inline void JoyFeedback::clear_intensity() {
  intensity_ = 0;
}
inline float JoyFeedback::_internal_intensity() const {
  return intensity_;
}
inline float JoyFeedback::intensity() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.JoyFeedback.intensity)
  return _internal_intensity();
}
inline void JoyFeedback::_internal_set_intensity(float value) {
  
  intensity_ = value;
}
inline void JoyFeedback::set_intensity(float value) {
  _internal_set_intensity(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.JoyFeedback.intensity)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace sensor_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fJoyFeedback_2eproto
