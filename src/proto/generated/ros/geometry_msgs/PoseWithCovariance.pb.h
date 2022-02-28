// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/geometry_msgs/PoseWithCovariance.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fPoseWithCovariance_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fPoseWithCovariance_2eproto

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
#include "ros/geometry_msgs/Pose.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fgeometry_5fmsgs_2fPoseWithCovariance_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fgeometry_5fmsgs_2fPoseWithCovariance_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fgeometry_5fmsgs_2fPoseWithCovariance_2eproto;
namespace ros {
namespace geometry_msgs {
class PoseWithCovariance;
struct PoseWithCovarianceDefaultTypeInternal;
extern PoseWithCovarianceDefaultTypeInternal _PoseWithCovariance_default_instance_;
}  // namespace geometry_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::geometry_msgs::PoseWithCovariance* Arena::CreateMaybeMessage<::ros::geometry_msgs::PoseWithCovariance>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace geometry_msgs {

// ===================================================================

class PoseWithCovariance final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.geometry_msgs.PoseWithCovariance) */ {
 public:
  inline PoseWithCovariance() : PoseWithCovariance(nullptr) {}
  ~PoseWithCovariance() override;
  explicit constexpr PoseWithCovariance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  PoseWithCovariance(const PoseWithCovariance& from);
  PoseWithCovariance(PoseWithCovariance&& from) noexcept
    : PoseWithCovariance() {
    *this = ::std::move(from);
  }

  inline PoseWithCovariance& operator=(const PoseWithCovariance& from) {
    CopyFrom(from);
    return *this;
  }
  inline PoseWithCovariance& operator=(PoseWithCovariance&& from) noexcept {
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
  static const PoseWithCovariance& default_instance() {
    return *internal_default_instance();
  }
  static inline const PoseWithCovariance* internal_default_instance() {
    return reinterpret_cast<const PoseWithCovariance*>(
               &_PoseWithCovariance_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PoseWithCovariance& a, PoseWithCovariance& b) {
    a.Swap(&b);
  }
  inline void Swap(PoseWithCovariance* other) {
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
  void UnsafeArenaSwap(PoseWithCovariance* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  PoseWithCovariance* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<PoseWithCovariance>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const PoseWithCovariance& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const PoseWithCovariance& from);
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
  void InternalSwap(PoseWithCovariance* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.geometry_msgs.PoseWithCovariance";
  }
  protected:
  explicit PoseWithCovariance(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kCovarianceFieldNumber = 2,
    kPoseFieldNumber = 1,
  };
  // repeated double covariance = 2;
  int covariance_size() const;
  private:
  int _internal_covariance_size() const;
  public:
  void clear_covariance();
  private:
  double _internal_covariance(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_covariance() const;
  void _internal_add_covariance(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_covariance();
  public:
  double covariance(int index) const;
  void set_covariance(int index, double value);
  void add_covariance(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      covariance() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_covariance();

  // .ros.geometry_msgs.Pose pose = 1;
  bool has_pose() const;
  private:
  bool _internal_has_pose() const;
  public:
  void clear_pose();
  const ::ros::geometry_msgs::Pose& pose() const;
  PROTOBUF_NODISCARD ::ros::geometry_msgs::Pose* release_pose();
  ::ros::geometry_msgs::Pose* mutable_pose();
  void set_allocated_pose(::ros::geometry_msgs::Pose* pose);
  private:
  const ::ros::geometry_msgs::Pose& _internal_pose() const;
  ::ros::geometry_msgs::Pose* _internal_mutable_pose();
  public:
  void unsafe_arena_set_allocated_pose(
      ::ros::geometry_msgs::Pose* pose);
  ::ros::geometry_msgs::Pose* unsafe_arena_release_pose();

  // @@protoc_insertion_point(class_scope:ros.geometry_msgs.PoseWithCovariance)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > covariance_;
  ::ros::geometry_msgs::Pose* pose_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fgeometry_5fmsgs_2fPoseWithCovariance_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PoseWithCovariance

// .ros.geometry_msgs.Pose pose = 1;
inline bool PoseWithCovariance::_internal_has_pose() const {
  return this != internal_default_instance() && pose_ != nullptr;
}
inline bool PoseWithCovariance::has_pose() const {
  return _internal_has_pose();
}
inline const ::ros::geometry_msgs::Pose& PoseWithCovariance::_internal_pose() const {
  const ::ros::geometry_msgs::Pose* p = pose_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::geometry_msgs::Pose&>(
      ::ros::geometry_msgs::_Pose_default_instance_);
}
inline const ::ros::geometry_msgs::Pose& PoseWithCovariance::pose() const {
  // @@protoc_insertion_point(field_get:ros.geometry_msgs.PoseWithCovariance.pose)
  return _internal_pose();
}
inline void PoseWithCovariance::unsafe_arena_set_allocated_pose(
    ::ros::geometry_msgs::Pose* pose) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(pose_);
  }
  pose_ = pose;
  if (pose) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.geometry_msgs.PoseWithCovariance.pose)
}
inline ::ros::geometry_msgs::Pose* PoseWithCovariance::release_pose() {
  
  ::ros::geometry_msgs::Pose* temp = pose_;
  pose_ = nullptr;
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
inline ::ros::geometry_msgs::Pose* PoseWithCovariance::unsafe_arena_release_pose() {
  // @@protoc_insertion_point(field_release:ros.geometry_msgs.PoseWithCovariance.pose)
  
  ::ros::geometry_msgs::Pose* temp = pose_;
  pose_ = nullptr;
  return temp;
}
inline ::ros::geometry_msgs::Pose* PoseWithCovariance::_internal_mutable_pose() {
  
  if (pose_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::geometry_msgs::Pose>(GetArenaForAllocation());
    pose_ = p;
  }
  return pose_;
}
inline ::ros::geometry_msgs::Pose* PoseWithCovariance::mutable_pose() {
  ::ros::geometry_msgs::Pose* _msg = _internal_mutable_pose();
  // @@protoc_insertion_point(field_mutable:ros.geometry_msgs.PoseWithCovariance.pose)
  return _msg;
}
inline void PoseWithCovariance::set_allocated_pose(::ros::geometry_msgs::Pose* pose) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(pose_);
  }
  if (pose) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(pose));
    if (message_arena != submessage_arena) {
      pose = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, pose, submessage_arena);
    }
    
  } else {
    
  }
  pose_ = pose;
  // @@protoc_insertion_point(field_set_allocated:ros.geometry_msgs.PoseWithCovariance.pose)
}

// repeated double covariance = 2;
inline int PoseWithCovariance::_internal_covariance_size() const {
  return covariance_.size();
}
inline int PoseWithCovariance::covariance_size() const {
  return _internal_covariance_size();
}
inline void PoseWithCovariance::clear_covariance() {
  covariance_.Clear();
}
inline double PoseWithCovariance::_internal_covariance(int index) const {
  return covariance_.Get(index);
}
inline double PoseWithCovariance::covariance(int index) const {
  // @@protoc_insertion_point(field_get:ros.geometry_msgs.PoseWithCovariance.covariance)
  return _internal_covariance(index);
}
inline void PoseWithCovariance::set_covariance(int index, double value) {
  covariance_.Set(index, value);
  // @@protoc_insertion_point(field_set:ros.geometry_msgs.PoseWithCovariance.covariance)
}
inline void PoseWithCovariance::_internal_add_covariance(double value) {
  covariance_.Add(value);
}
inline void PoseWithCovariance::add_covariance(double value) {
  _internal_add_covariance(value);
  // @@protoc_insertion_point(field_add:ros.geometry_msgs.PoseWithCovariance.covariance)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
PoseWithCovariance::_internal_covariance() const {
  return covariance_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
PoseWithCovariance::covariance() const {
  // @@protoc_insertion_point(field_list:ros.geometry_msgs.PoseWithCovariance.covariance)
  return _internal_covariance();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
PoseWithCovariance::_internal_mutable_covariance() {
  return &covariance_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
PoseWithCovariance::mutable_covariance() {
  // @@protoc_insertion_point(field_mutable_list:ros.geometry_msgs.PoseWithCovariance.covariance)
  return _internal_mutable_covariance();
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace geometry_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fgeometry_5fmsgs_2fPoseWithCovariance_2eproto