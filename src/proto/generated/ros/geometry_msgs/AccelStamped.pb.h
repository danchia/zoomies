// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/geometry_msgs/AccelStamped.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto

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
#include "ros/geometry_msgs/Accel.pb.h"
#include "ros/std_msgs/Header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto;
namespace ros {
namespace geometry_msgs {
class AccelStamped;
struct AccelStampedDefaultTypeInternal;
extern AccelStampedDefaultTypeInternal _AccelStamped_default_instance_;
}  // namespace geometry_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::geometry_msgs::AccelStamped* Arena::CreateMaybeMessage<::ros::geometry_msgs::AccelStamped>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace geometry_msgs {

// ===================================================================

class AccelStamped final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.geometry_msgs.AccelStamped) */ {
 public:
  inline AccelStamped() : AccelStamped(nullptr) {}
  ~AccelStamped() override;
  explicit constexpr AccelStamped(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  AccelStamped(const AccelStamped& from);
  AccelStamped(AccelStamped&& from) noexcept
    : AccelStamped() {
    *this = ::std::move(from);
  }

  inline AccelStamped& operator=(const AccelStamped& from) {
    CopyFrom(from);
    return *this;
  }
  inline AccelStamped& operator=(AccelStamped&& from) noexcept {
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
  static const AccelStamped& default_instance() {
    return *internal_default_instance();
  }
  static inline const AccelStamped* internal_default_instance() {
    return reinterpret_cast<const AccelStamped*>(
               &_AccelStamped_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(AccelStamped& a, AccelStamped& b) {
    a.Swap(&b);
  }
  inline void Swap(AccelStamped* other) {
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
  void UnsafeArenaSwap(AccelStamped* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  AccelStamped* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<AccelStamped>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const AccelStamped& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const AccelStamped& from);
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
  void InternalSwap(AccelStamped* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.geometry_msgs.AccelStamped";
  }
  protected:
  explicit AccelStamped(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kAccelFieldNumber = 2,
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

  // .ros.geometry_msgs.Accel accel = 2;
  bool has_accel() const;
  private:
  bool _internal_has_accel() const;
  public:
  void clear_accel();
  const ::ros::geometry_msgs::Accel& accel() const;
  PROTOBUF_NODISCARD ::ros::geometry_msgs::Accel* release_accel();
  ::ros::geometry_msgs::Accel* mutable_accel();
  void set_allocated_accel(::ros::geometry_msgs::Accel* accel);
  private:
  const ::ros::geometry_msgs::Accel& _internal_accel() const;
  ::ros::geometry_msgs::Accel* _internal_mutable_accel();
  public:
  void unsafe_arena_set_allocated_accel(
      ::ros::geometry_msgs::Accel* accel);
  ::ros::geometry_msgs::Accel* unsafe_arena_release_accel();

  // @@protoc_insertion_point(class_scope:ros.geometry_msgs.AccelStamped)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::ros::std_msgs::Header* header_;
  ::ros::geometry_msgs::Accel* accel_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// AccelStamped

// .ros.std_msgs.Header header = 1;
inline bool AccelStamped::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool AccelStamped::has_header() const {
  return _internal_has_header();
}
inline const ::ros::std_msgs::Header& AccelStamped::_internal_header() const {
  const ::ros::std_msgs::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::std_msgs::Header&>(
      ::ros::std_msgs::_Header_default_instance_);
}
inline const ::ros::std_msgs::Header& AccelStamped::header() const {
  // @@protoc_insertion_point(field_get:ros.geometry_msgs.AccelStamped.header)
  return _internal_header();
}
inline void AccelStamped::unsafe_arena_set_allocated_header(
    ::ros::std_msgs::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.geometry_msgs.AccelStamped.header)
}
inline ::ros::std_msgs::Header* AccelStamped::release_header() {
  
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
inline ::ros::std_msgs::Header* AccelStamped::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:ros.geometry_msgs.AccelStamped.header)
  
  ::ros::std_msgs::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::ros::std_msgs::Header* AccelStamped::_internal_mutable_header() {
  
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::std_msgs::Header>(GetArenaForAllocation());
    header_ = p;
  }
  return header_;
}
inline ::ros::std_msgs::Header* AccelStamped::mutable_header() {
  ::ros::std_msgs::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:ros.geometry_msgs.AccelStamped.header)
  return _msg;
}
inline void AccelStamped::set_allocated_header(::ros::std_msgs::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:ros.geometry_msgs.AccelStamped.header)
}

// .ros.geometry_msgs.Accel accel = 2;
inline bool AccelStamped::_internal_has_accel() const {
  return this != internal_default_instance() && accel_ != nullptr;
}
inline bool AccelStamped::has_accel() const {
  return _internal_has_accel();
}
inline const ::ros::geometry_msgs::Accel& AccelStamped::_internal_accel() const {
  const ::ros::geometry_msgs::Accel* p = accel_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::geometry_msgs::Accel&>(
      ::ros::geometry_msgs::_Accel_default_instance_);
}
inline const ::ros::geometry_msgs::Accel& AccelStamped::accel() const {
  // @@protoc_insertion_point(field_get:ros.geometry_msgs.AccelStamped.accel)
  return _internal_accel();
}
inline void AccelStamped::unsafe_arena_set_allocated_accel(
    ::ros::geometry_msgs::Accel* accel) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(accel_);
  }
  accel_ = accel;
  if (accel) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.geometry_msgs.AccelStamped.accel)
}
inline ::ros::geometry_msgs::Accel* AccelStamped::release_accel() {
  
  ::ros::geometry_msgs::Accel* temp = accel_;
  accel_ = nullptr;
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
inline ::ros::geometry_msgs::Accel* AccelStamped::unsafe_arena_release_accel() {
  // @@protoc_insertion_point(field_release:ros.geometry_msgs.AccelStamped.accel)
  
  ::ros::geometry_msgs::Accel* temp = accel_;
  accel_ = nullptr;
  return temp;
}
inline ::ros::geometry_msgs::Accel* AccelStamped::_internal_mutable_accel() {
  
  if (accel_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::geometry_msgs::Accel>(GetArenaForAllocation());
    accel_ = p;
  }
  return accel_;
}
inline ::ros::geometry_msgs::Accel* AccelStamped::mutable_accel() {
  ::ros::geometry_msgs::Accel* _msg = _internal_mutable_accel();
  // @@protoc_insertion_point(field_mutable:ros.geometry_msgs.AccelStamped.accel)
  return _msg;
}
inline void AccelStamped::set_allocated_accel(::ros::geometry_msgs::Accel* accel) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(accel_);
  }
  if (accel) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(accel));
    if (message_arena != submessage_arena) {
      accel = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, accel, submessage_arena);
    }
    
  } else {
    
  }
  accel_ = accel;
  // @@protoc_insertion_point(field_set_allocated:ros.geometry_msgs.AccelStamped.accel)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace geometry_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fAccelStamped_2eproto
