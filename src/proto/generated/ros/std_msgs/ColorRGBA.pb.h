// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/std_msgs/ColorRGBA.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fstd_5fmsgs_2fColorRGBA_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fstd_5fmsgs_2fColorRGBA_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ros_2fstd_5fmsgs_2fColorRGBA_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fstd_5fmsgs_2fColorRGBA_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fstd_5fmsgs_2fColorRGBA_2eproto;
namespace ros {
namespace std_msgs {
class ColorRGBA;
struct ColorRGBADefaultTypeInternal;
extern ColorRGBADefaultTypeInternal _ColorRGBA_default_instance_;
}  // namespace std_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::std_msgs::ColorRGBA* Arena::CreateMaybeMessage<::ros::std_msgs::ColorRGBA>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace std_msgs {

// ===================================================================

class ColorRGBA final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.std_msgs.ColorRGBA) */ {
 public:
  inline ColorRGBA() : ColorRGBA(nullptr) {}
  ~ColorRGBA() override;
  explicit constexpr ColorRGBA(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  ColorRGBA(const ColorRGBA& from);
  ColorRGBA(ColorRGBA&& from) noexcept
    : ColorRGBA() {
    *this = ::std::move(from);
  }

  inline ColorRGBA& operator=(const ColorRGBA& from) {
    CopyFrom(from);
    return *this;
  }
  inline ColorRGBA& operator=(ColorRGBA&& from) noexcept {
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
  static const ColorRGBA& default_instance() {
    return *internal_default_instance();
  }
  static inline const ColorRGBA* internal_default_instance() {
    return reinterpret_cast<const ColorRGBA*>(
               &_ColorRGBA_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ColorRGBA& a, ColorRGBA& b) {
    a.Swap(&b);
  }
  inline void Swap(ColorRGBA* other) {
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
  void UnsafeArenaSwap(ColorRGBA* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  ColorRGBA* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<ColorRGBA>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const ColorRGBA& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const ColorRGBA& from);
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
  void InternalSwap(ColorRGBA* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.std_msgs.ColorRGBA";
  }
  protected:
  explicit ColorRGBA(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kRFieldNumber = 1,
    kGFieldNumber = 2,
    kBFieldNumber = 3,
    kAFieldNumber = 4,
  };
  // float r = 1;
  void clear_r();
  float r() const;
  void set_r(float value);
  private:
  float _internal_r() const;
  void _internal_set_r(float value);
  public:

  // float g = 2;
  void clear_g();
  float g() const;
  void set_g(float value);
  private:
  float _internal_g() const;
  void _internal_set_g(float value);
  public:

  // float b = 3;
  void clear_b();
  float b() const;
  void set_b(float value);
  private:
  float _internal_b() const;
  void _internal_set_b(float value);
  public:

  // float a = 4;
  void clear_a();
  float a() const;
  void set_a(float value);
  private:
  float _internal_a() const;
  void _internal_set_a(float value);
  public:

  // @@protoc_insertion_point(class_scope:ros.std_msgs.ColorRGBA)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  float r_;
  float g_;
  float b_;
  float a_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fstd_5fmsgs_2fColorRGBA_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ColorRGBA

// float r = 1;
inline void ColorRGBA::clear_r() {
  r_ = 0;
}
inline float ColorRGBA::_internal_r() const {
  return r_;
}
inline float ColorRGBA::r() const {
  // @@protoc_insertion_point(field_get:ros.std_msgs.ColorRGBA.r)
  return _internal_r();
}
inline void ColorRGBA::_internal_set_r(float value) {
  
  r_ = value;
}
inline void ColorRGBA::set_r(float value) {
  _internal_set_r(value);
  // @@protoc_insertion_point(field_set:ros.std_msgs.ColorRGBA.r)
}

// float g = 2;
inline void ColorRGBA::clear_g() {
  g_ = 0;
}
inline float ColorRGBA::_internal_g() const {
  return g_;
}
inline float ColorRGBA::g() const {
  // @@protoc_insertion_point(field_get:ros.std_msgs.ColorRGBA.g)
  return _internal_g();
}
inline void ColorRGBA::_internal_set_g(float value) {
  
  g_ = value;
}
inline void ColorRGBA::set_g(float value) {
  _internal_set_g(value);
  // @@protoc_insertion_point(field_set:ros.std_msgs.ColorRGBA.g)
}

// float b = 3;
inline void ColorRGBA::clear_b() {
  b_ = 0;
}
inline float ColorRGBA::_internal_b() const {
  return b_;
}
inline float ColorRGBA::b() const {
  // @@protoc_insertion_point(field_get:ros.std_msgs.ColorRGBA.b)
  return _internal_b();
}
inline void ColorRGBA::_internal_set_b(float value) {
  
  b_ = value;
}
inline void ColorRGBA::set_b(float value) {
  _internal_set_b(value);
  // @@protoc_insertion_point(field_set:ros.std_msgs.ColorRGBA.b)
}

// float a = 4;
inline void ColorRGBA::clear_a() {
  a_ = 0;
}
inline float ColorRGBA::_internal_a() const {
  return a_;
}
inline float ColorRGBA::a() const {
  // @@protoc_insertion_point(field_get:ros.std_msgs.ColorRGBA.a)
  return _internal_a();
}
inline void ColorRGBA::_internal_set_a(float value) {
  
  a_ = value;
}
inline void ColorRGBA::set_a(float value) {
  _internal_set_a(value);
  // @@protoc_insertion_point(field_set:ros.std_msgs.ColorRGBA.a)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace std_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fstd_5fmsgs_2fColorRGBA_2eproto