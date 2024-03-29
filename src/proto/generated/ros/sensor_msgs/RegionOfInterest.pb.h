// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/RegionOfInterest.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fRegionOfInterest_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fRegionOfInterest_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ros_2fsensor_5fmsgs_2fRegionOfInterest_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fsensor_5fmsgs_2fRegionOfInterest_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fRegionOfInterest_2eproto;
namespace ros {
namespace sensor_msgs {
class RegionOfInterest;
struct RegionOfInterestDefaultTypeInternal;
extern RegionOfInterestDefaultTypeInternal _RegionOfInterest_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::sensor_msgs::RegionOfInterest* Arena::CreateMaybeMessage<::ros::sensor_msgs::RegionOfInterest>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace sensor_msgs {

// ===================================================================

class RegionOfInterest final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.sensor_msgs.RegionOfInterest) */ {
 public:
  inline RegionOfInterest() : RegionOfInterest(nullptr) {}
  ~RegionOfInterest() override;
  explicit constexpr RegionOfInterest(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  RegionOfInterest(const RegionOfInterest& from);
  RegionOfInterest(RegionOfInterest&& from) noexcept
    : RegionOfInterest() {
    *this = ::std::move(from);
  }

  inline RegionOfInterest& operator=(const RegionOfInterest& from) {
    CopyFrom(from);
    return *this;
  }
  inline RegionOfInterest& operator=(RegionOfInterest&& from) noexcept {
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
  static const RegionOfInterest& default_instance() {
    return *internal_default_instance();
  }
  static inline const RegionOfInterest* internal_default_instance() {
    return reinterpret_cast<const RegionOfInterest*>(
               &_RegionOfInterest_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(RegionOfInterest& a, RegionOfInterest& b) {
    a.Swap(&b);
  }
  inline void Swap(RegionOfInterest* other) {
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
  void UnsafeArenaSwap(RegionOfInterest* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  RegionOfInterest* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<RegionOfInterest>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const RegionOfInterest& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const RegionOfInterest& from);
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
  void InternalSwap(RegionOfInterest* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.sensor_msgs.RegionOfInterest";
  }
  protected:
  explicit RegionOfInterest(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kXOffsetFieldNumber = 1,
    kYOffsetFieldNumber = 2,
    kHeightFieldNumber = 3,
    kWidthFieldNumber = 4,
    kDoRectifyFieldNumber = 5,
  };
  // uint32 x_offset = 1;
  void clear_x_offset();
  uint32_t x_offset() const;
  void set_x_offset(uint32_t value);
  private:
  uint32_t _internal_x_offset() const;
  void _internal_set_x_offset(uint32_t value);
  public:

  // uint32 y_offset = 2;
  void clear_y_offset();
  uint32_t y_offset() const;
  void set_y_offset(uint32_t value);
  private:
  uint32_t _internal_y_offset() const;
  void _internal_set_y_offset(uint32_t value);
  public:

  // uint32 height = 3;
  void clear_height();
  uint32_t height() const;
  void set_height(uint32_t value);
  private:
  uint32_t _internal_height() const;
  void _internal_set_height(uint32_t value);
  public:

  // uint32 width = 4;
  void clear_width();
  uint32_t width() const;
  void set_width(uint32_t value);
  private:
  uint32_t _internal_width() const;
  void _internal_set_width(uint32_t value);
  public:

  // bool do_rectify = 5;
  void clear_do_rectify();
  bool do_rectify() const;
  void set_do_rectify(bool value);
  private:
  bool _internal_do_rectify() const;
  void _internal_set_do_rectify(bool value);
  public:

  // @@protoc_insertion_point(class_scope:ros.sensor_msgs.RegionOfInterest)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  uint32_t x_offset_;
  uint32_t y_offset_;
  uint32_t height_;
  uint32_t width_;
  bool do_rectify_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fsensor_5fmsgs_2fRegionOfInterest_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RegionOfInterest

// uint32 x_offset = 1;
inline void RegionOfInterest::clear_x_offset() {
  x_offset_ = 0u;
}
inline uint32_t RegionOfInterest::_internal_x_offset() const {
  return x_offset_;
}
inline uint32_t RegionOfInterest::x_offset() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.RegionOfInterest.x_offset)
  return _internal_x_offset();
}
inline void RegionOfInterest::_internal_set_x_offset(uint32_t value) {
  
  x_offset_ = value;
}
inline void RegionOfInterest::set_x_offset(uint32_t value) {
  _internal_set_x_offset(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.RegionOfInterest.x_offset)
}

// uint32 y_offset = 2;
inline void RegionOfInterest::clear_y_offset() {
  y_offset_ = 0u;
}
inline uint32_t RegionOfInterest::_internal_y_offset() const {
  return y_offset_;
}
inline uint32_t RegionOfInterest::y_offset() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.RegionOfInterest.y_offset)
  return _internal_y_offset();
}
inline void RegionOfInterest::_internal_set_y_offset(uint32_t value) {
  
  y_offset_ = value;
}
inline void RegionOfInterest::set_y_offset(uint32_t value) {
  _internal_set_y_offset(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.RegionOfInterest.y_offset)
}

// uint32 height = 3;
inline void RegionOfInterest::clear_height() {
  height_ = 0u;
}
inline uint32_t RegionOfInterest::_internal_height() const {
  return height_;
}
inline uint32_t RegionOfInterest::height() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.RegionOfInterest.height)
  return _internal_height();
}
inline void RegionOfInterest::_internal_set_height(uint32_t value) {
  
  height_ = value;
}
inline void RegionOfInterest::set_height(uint32_t value) {
  _internal_set_height(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.RegionOfInterest.height)
}

// uint32 width = 4;
inline void RegionOfInterest::clear_width() {
  width_ = 0u;
}
inline uint32_t RegionOfInterest::_internal_width() const {
  return width_;
}
inline uint32_t RegionOfInterest::width() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.RegionOfInterest.width)
  return _internal_width();
}
inline void RegionOfInterest::_internal_set_width(uint32_t value) {
  
  width_ = value;
}
inline void RegionOfInterest::set_width(uint32_t value) {
  _internal_set_width(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.RegionOfInterest.width)
}

// bool do_rectify = 5;
inline void RegionOfInterest::clear_do_rectify() {
  do_rectify_ = false;
}
inline bool RegionOfInterest::_internal_do_rectify() const {
  return do_rectify_;
}
inline bool RegionOfInterest::do_rectify() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.RegionOfInterest.do_rectify)
  return _internal_do_rectify();
}
inline void RegionOfInterest::_internal_set_do_rectify(bool value) {
  
  do_rectify_ = value;
}
inline void RegionOfInterest::set_do_rectify(bool value) {
  _internal_set_do_rectify(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.RegionOfInterest.do_rectify)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace sensor_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fRegionOfInterest_2eproto
