// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/geometry_msgs/PolygonStamped.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fPolygonStamped_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fPolygonStamped_2eproto

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
#include "ros/geometry_msgs/Polygon.pb.h"
#include "ros/std_msgs/Header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fgeometry_5fmsgs_2fPolygonStamped_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fgeometry_5fmsgs_2fPolygonStamped_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fgeometry_5fmsgs_2fPolygonStamped_2eproto;
namespace ros {
namespace geometry_msgs {
class PolygonStamped;
struct PolygonStampedDefaultTypeInternal;
extern PolygonStampedDefaultTypeInternal _PolygonStamped_default_instance_;
}  // namespace geometry_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::geometry_msgs::PolygonStamped* Arena::CreateMaybeMessage<::ros::geometry_msgs::PolygonStamped>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace geometry_msgs {

// ===================================================================

class PolygonStamped final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.geometry_msgs.PolygonStamped) */ {
 public:
  inline PolygonStamped() : PolygonStamped(nullptr) {}
  ~PolygonStamped() override;
  explicit constexpr PolygonStamped(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  PolygonStamped(const PolygonStamped& from);
  PolygonStamped(PolygonStamped&& from) noexcept
    : PolygonStamped() {
    *this = ::std::move(from);
  }

  inline PolygonStamped& operator=(const PolygonStamped& from) {
    CopyFrom(from);
    return *this;
  }
  inline PolygonStamped& operator=(PolygonStamped&& from) noexcept {
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
  static const PolygonStamped& default_instance() {
    return *internal_default_instance();
  }
  static inline const PolygonStamped* internal_default_instance() {
    return reinterpret_cast<const PolygonStamped*>(
               &_PolygonStamped_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PolygonStamped& a, PolygonStamped& b) {
    a.Swap(&b);
  }
  inline void Swap(PolygonStamped* other) {
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
  void UnsafeArenaSwap(PolygonStamped* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  PolygonStamped* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<PolygonStamped>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const PolygonStamped& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const PolygonStamped& from);
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
  void InternalSwap(PolygonStamped* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.geometry_msgs.PolygonStamped";
  }
  protected:
  explicit PolygonStamped(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kHeaderFieldNumber = 1,
    kPolygonFieldNumber = 2,
  };
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

  // .ros.geometry_msgs.Polygon polygon = 2;
  bool has_polygon() const;
  private:
  bool _internal_has_polygon() const;
  public:
  void clear_polygon();
  const ::ros::geometry_msgs::Polygon& polygon() const;
  PROTOBUF_NODISCARD ::ros::geometry_msgs::Polygon* release_polygon();
  ::ros::geometry_msgs::Polygon* mutable_polygon();
  void set_allocated_polygon(::ros::geometry_msgs::Polygon* polygon);
  private:
  const ::ros::geometry_msgs::Polygon& _internal_polygon() const;
  ::ros::geometry_msgs::Polygon* _internal_mutable_polygon();
  public:
  void unsafe_arena_set_allocated_polygon(
      ::ros::geometry_msgs::Polygon* polygon);
  ::ros::geometry_msgs::Polygon* unsafe_arena_release_polygon();

  // @@protoc_insertion_point(class_scope:ros.geometry_msgs.PolygonStamped)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::ros::std_msgs::Header* header_;
  ::ros::geometry_msgs::Polygon* polygon_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fgeometry_5fmsgs_2fPolygonStamped_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PolygonStamped

// .ros.std_msgs.Header header = 1;
inline bool PolygonStamped::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool PolygonStamped::has_header() const {
  return _internal_has_header();
}
inline const ::ros::std_msgs::Header& PolygonStamped::_internal_header() const {
  const ::ros::std_msgs::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::std_msgs::Header&>(
      ::ros::std_msgs::_Header_default_instance_);
}
inline const ::ros::std_msgs::Header& PolygonStamped::header() const {
  // @@protoc_insertion_point(field_get:ros.geometry_msgs.PolygonStamped.header)
  return _internal_header();
}
inline void PolygonStamped::unsafe_arena_set_allocated_header(
    ::ros::std_msgs::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.geometry_msgs.PolygonStamped.header)
}
inline ::ros::std_msgs::Header* PolygonStamped::release_header() {
  
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
inline ::ros::std_msgs::Header* PolygonStamped::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:ros.geometry_msgs.PolygonStamped.header)
  
  ::ros::std_msgs::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::ros::std_msgs::Header* PolygonStamped::_internal_mutable_header() {
  
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::std_msgs::Header>(GetArenaForAllocation());
    header_ = p;
  }
  return header_;
}
inline ::ros::std_msgs::Header* PolygonStamped::mutable_header() {
  ::ros::std_msgs::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:ros.geometry_msgs.PolygonStamped.header)
  return _msg;
}
inline void PolygonStamped::set_allocated_header(::ros::std_msgs::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:ros.geometry_msgs.PolygonStamped.header)
}

// .ros.geometry_msgs.Polygon polygon = 2;
inline bool PolygonStamped::_internal_has_polygon() const {
  return this != internal_default_instance() && polygon_ != nullptr;
}
inline bool PolygonStamped::has_polygon() const {
  return _internal_has_polygon();
}
inline const ::ros::geometry_msgs::Polygon& PolygonStamped::_internal_polygon() const {
  const ::ros::geometry_msgs::Polygon* p = polygon_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::geometry_msgs::Polygon&>(
      ::ros::geometry_msgs::_Polygon_default_instance_);
}
inline const ::ros::geometry_msgs::Polygon& PolygonStamped::polygon() const {
  // @@protoc_insertion_point(field_get:ros.geometry_msgs.PolygonStamped.polygon)
  return _internal_polygon();
}
inline void PolygonStamped::unsafe_arena_set_allocated_polygon(
    ::ros::geometry_msgs::Polygon* polygon) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(polygon_);
  }
  polygon_ = polygon;
  if (polygon) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.geometry_msgs.PolygonStamped.polygon)
}
inline ::ros::geometry_msgs::Polygon* PolygonStamped::release_polygon() {
  
  ::ros::geometry_msgs::Polygon* temp = polygon_;
  polygon_ = nullptr;
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
inline ::ros::geometry_msgs::Polygon* PolygonStamped::unsafe_arena_release_polygon() {
  // @@protoc_insertion_point(field_release:ros.geometry_msgs.PolygonStamped.polygon)
  
  ::ros::geometry_msgs::Polygon* temp = polygon_;
  polygon_ = nullptr;
  return temp;
}
inline ::ros::geometry_msgs::Polygon* PolygonStamped::_internal_mutable_polygon() {
  
  if (polygon_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::geometry_msgs::Polygon>(GetArenaForAllocation());
    polygon_ = p;
  }
  return polygon_;
}
inline ::ros::geometry_msgs::Polygon* PolygonStamped::mutable_polygon() {
  ::ros::geometry_msgs::Polygon* _msg = _internal_mutable_polygon();
  // @@protoc_insertion_point(field_mutable:ros.geometry_msgs.PolygonStamped.polygon)
  return _msg;
}
inline void PolygonStamped::set_allocated_polygon(::ros::geometry_msgs::Polygon* polygon) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(polygon_);
  }
  if (polygon) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(polygon));
    if (message_arena != submessage_arena) {
      polygon = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, polygon, submessage_arena);
    }
    
  } else {
    
  }
  polygon_ = polygon;
  // @@protoc_insertion_point(field_set_allocated:ros.geometry_msgs.PolygonStamped.polygon)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace geometry_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fPolygonStamped_2eproto
