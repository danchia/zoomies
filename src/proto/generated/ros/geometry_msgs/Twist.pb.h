// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/geometry_msgs/Twist.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fTwist_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fTwist_2eproto

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
#include "ros/geometry_msgs/Vector3.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fgeometry_5fmsgs_2fTwist_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fgeometry_5fmsgs_2fTwist_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fgeometry_5fmsgs_2fTwist_2eproto;
namespace ros {
namespace geometry_msgs {
class Twist;
struct TwistDefaultTypeInternal;
extern TwistDefaultTypeInternal _Twist_default_instance_;
}  // namespace geometry_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::geometry_msgs::Twist* Arena::CreateMaybeMessage<::ros::geometry_msgs::Twist>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace geometry_msgs {

// ===================================================================

class Twist final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.geometry_msgs.Twist) */ {
 public:
  inline Twist() : Twist(nullptr) {}
  ~Twist() override;
  explicit constexpr Twist(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Twist(const Twist& from);
  Twist(Twist&& from) noexcept
    : Twist() {
    *this = ::std::move(from);
  }

  inline Twist& operator=(const Twist& from) {
    CopyFrom(from);
    return *this;
  }
  inline Twist& operator=(Twist&& from) noexcept {
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
  static const Twist& default_instance() {
    return *internal_default_instance();
  }
  static inline const Twist* internal_default_instance() {
    return reinterpret_cast<const Twist*>(
               &_Twist_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Twist& a, Twist& b) {
    a.Swap(&b);
  }
  inline void Swap(Twist* other) {
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
  void UnsafeArenaSwap(Twist* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Twist* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Twist>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Twist& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const Twist& from);
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
  void InternalSwap(Twist* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.geometry_msgs.Twist";
  }
  protected:
  explicit Twist(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kLinearFieldNumber = 1,
    kAngularFieldNumber = 2,
  };
  // .ros.geometry_msgs.Vector3 linear = 1;
  bool has_linear() const;
  private:
  bool _internal_has_linear() const;
  public:
  void clear_linear();
  const ::ros::geometry_msgs::Vector3& linear() const;
  PROTOBUF_NODISCARD ::ros::geometry_msgs::Vector3* release_linear();
  ::ros::geometry_msgs::Vector3* mutable_linear();
  void set_allocated_linear(::ros::geometry_msgs::Vector3* linear);
  private:
  const ::ros::geometry_msgs::Vector3& _internal_linear() const;
  ::ros::geometry_msgs::Vector3* _internal_mutable_linear();
  public:
  void unsafe_arena_set_allocated_linear(
      ::ros::geometry_msgs::Vector3* linear);
  ::ros::geometry_msgs::Vector3* unsafe_arena_release_linear();

  // .ros.geometry_msgs.Vector3 angular = 2;
  bool has_angular() const;
  private:
  bool _internal_has_angular() const;
  public:
  void clear_angular();
  const ::ros::geometry_msgs::Vector3& angular() const;
  PROTOBUF_NODISCARD ::ros::geometry_msgs::Vector3* release_angular();
  ::ros::geometry_msgs::Vector3* mutable_angular();
  void set_allocated_angular(::ros::geometry_msgs::Vector3* angular);
  private:
  const ::ros::geometry_msgs::Vector3& _internal_angular() const;
  ::ros::geometry_msgs::Vector3* _internal_mutable_angular();
  public:
  void unsafe_arena_set_allocated_angular(
      ::ros::geometry_msgs::Vector3* angular);
  ::ros::geometry_msgs::Vector3* unsafe_arena_release_angular();

  // @@protoc_insertion_point(class_scope:ros.geometry_msgs.Twist)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::ros::geometry_msgs::Vector3* linear_;
  ::ros::geometry_msgs::Vector3* angular_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fgeometry_5fmsgs_2fTwist_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Twist

// .ros.geometry_msgs.Vector3 linear = 1;
inline bool Twist::_internal_has_linear() const {
  return this != internal_default_instance() && linear_ != nullptr;
}
inline bool Twist::has_linear() const {
  return _internal_has_linear();
}
inline const ::ros::geometry_msgs::Vector3& Twist::_internal_linear() const {
  const ::ros::geometry_msgs::Vector3* p = linear_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::geometry_msgs::Vector3&>(
      ::ros::geometry_msgs::_Vector3_default_instance_);
}
inline const ::ros::geometry_msgs::Vector3& Twist::linear() const {
  // @@protoc_insertion_point(field_get:ros.geometry_msgs.Twist.linear)
  return _internal_linear();
}
inline void Twist::unsafe_arena_set_allocated_linear(
    ::ros::geometry_msgs::Vector3* linear) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(linear_);
  }
  linear_ = linear;
  if (linear) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.geometry_msgs.Twist.linear)
}
inline ::ros::geometry_msgs::Vector3* Twist::release_linear() {
  
  ::ros::geometry_msgs::Vector3* temp = linear_;
  linear_ = nullptr;
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
inline ::ros::geometry_msgs::Vector3* Twist::unsafe_arena_release_linear() {
  // @@protoc_insertion_point(field_release:ros.geometry_msgs.Twist.linear)
  
  ::ros::geometry_msgs::Vector3* temp = linear_;
  linear_ = nullptr;
  return temp;
}
inline ::ros::geometry_msgs::Vector3* Twist::_internal_mutable_linear() {
  
  if (linear_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::geometry_msgs::Vector3>(GetArenaForAllocation());
    linear_ = p;
  }
  return linear_;
}
inline ::ros::geometry_msgs::Vector3* Twist::mutable_linear() {
  ::ros::geometry_msgs::Vector3* _msg = _internal_mutable_linear();
  // @@protoc_insertion_point(field_mutable:ros.geometry_msgs.Twist.linear)
  return _msg;
}
inline void Twist::set_allocated_linear(::ros::geometry_msgs::Vector3* linear) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(linear_);
  }
  if (linear) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(linear));
    if (message_arena != submessage_arena) {
      linear = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, linear, submessage_arena);
    }
    
  } else {
    
  }
  linear_ = linear;
  // @@protoc_insertion_point(field_set_allocated:ros.geometry_msgs.Twist.linear)
}

// .ros.geometry_msgs.Vector3 angular = 2;
inline bool Twist::_internal_has_angular() const {
  return this != internal_default_instance() && angular_ != nullptr;
}
inline bool Twist::has_angular() const {
  return _internal_has_angular();
}
inline const ::ros::geometry_msgs::Vector3& Twist::_internal_angular() const {
  const ::ros::geometry_msgs::Vector3* p = angular_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::geometry_msgs::Vector3&>(
      ::ros::geometry_msgs::_Vector3_default_instance_);
}
inline const ::ros::geometry_msgs::Vector3& Twist::angular() const {
  // @@protoc_insertion_point(field_get:ros.geometry_msgs.Twist.angular)
  return _internal_angular();
}
inline void Twist::unsafe_arena_set_allocated_angular(
    ::ros::geometry_msgs::Vector3* angular) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(angular_);
  }
  angular_ = angular;
  if (angular) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.geometry_msgs.Twist.angular)
}
inline ::ros::geometry_msgs::Vector3* Twist::release_angular() {
  
  ::ros::geometry_msgs::Vector3* temp = angular_;
  angular_ = nullptr;
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
inline ::ros::geometry_msgs::Vector3* Twist::unsafe_arena_release_angular() {
  // @@protoc_insertion_point(field_release:ros.geometry_msgs.Twist.angular)
  
  ::ros::geometry_msgs::Vector3* temp = angular_;
  angular_ = nullptr;
  return temp;
}
inline ::ros::geometry_msgs::Vector3* Twist::_internal_mutable_angular() {
  
  if (angular_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::geometry_msgs::Vector3>(GetArenaForAllocation());
    angular_ = p;
  }
  return angular_;
}
inline ::ros::geometry_msgs::Vector3* Twist::mutable_angular() {
  ::ros::geometry_msgs::Vector3* _msg = _internal_mutable_angular();
  // @@protoc_insertion_point(field_mutable:ros.geometry_msgs.Twist.angular)
  return _msg;
}
inline void Twist::set_allocated_angular(::ros::geometry_msgs::Vector3* angular) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(angular_);
  }
  if (angular) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(angular));
    if (message_arena != submessage_arena) {
      angular = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, angular, submessage_arena);
    }
    
  } else {
    
  }
  angular_ = angular;
  // @@protoc_insertion_point(field_set_allocated:ros.geometry_msgs.Twist.angular)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace geometry_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fTwist_2eproto
