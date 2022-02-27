// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/trajectory_msgs/JointTrajectoryPoint.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2ftrajectory_5fmsgs_2fJointTrajectoryPoint_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2ftrajectory_5fmsgs_2fJointTrajectoryPoint_2eproto

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
#include "ros/builtins.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2ftrajectory_5fmsgs_2fJointTrajectoryPoint_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2ftrajectory_5fmsgs_2fJointTrajectoryPoint_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2ftrajectory_5fmsgs_2fJointTrajectoryPoint_2eproto;
namespace ros {
namespace trajectory_msgs {
class JointTrajectoryPoint;
struct JointTrajectoryPointDefaultTypeInternal;
extern JointTrajectoryPointDefaultTypeInternal _JointTrajectoryPoint_default_instance_;
}  // namespace trajectory_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::trajectory_msgs::JointTrajectoryPoint* Arena::CreateMaybeMessage<::ros::trajectory_msgs::JointTrajectoryPoint>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace trajectory_msgs {

// ===================================================================

class JointTrajectoryPoint final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.trajectory_msgs.JointTrajectoryPoint) */ {
 public:
  inline JointTrajectoryPoint() : JointTrajectoryPoint(nullptr) {}
  ~JointTrajectoryPoint() override;
  explicit constexpr JointTrajectoryPoint(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  JointTrajectoryPoint(const JointTrajectoryPoint& from);
  JointTrajectoryPoint(JointTrajectoryPoint&& from) noexcept
    : JointTrajectoryPoint() {
    *this = ::std::move(from);
  }

  inline JointTrajectoryPoint& operator=(const JointTrajectoryPoint& from) {
    CopyFrom(from);
    return *this;
  }
  inline JointTrajectoryPoint& operator=(JointTrajectoryPoint&& from) noexcept {
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
  static const JointTrajectoryPoint& default_instance() {
    return *internal_default_instance();
  }
  static inline const JointTrajectoryPoint* internal_default_instance() {
    return reinterpret_cast<const JointTrajectoryPoint*>(
               &_JointTrajectoryPoint_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(JointTrajectoryPoint& a, JointTrajectoryPoint& b) {
    a.Swap(&b);
  }
  inline void Swap(JointTrajectoryPoint* other) {
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
  void UnsafeArenaSwap(JointTrajectoryPoint* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  JointTrajectoryPoint* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<JointTrajectoryPoint>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const JointTrajectoryPoint& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const JointTrajectoryPoint& from);
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
  void InternalSwap(JointTrajectoryPoint* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.trajectory_msgs.JointTrajectoryPoint";
  }
  protected:
  explicit JointTrajectoryPoint(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kPositionsFieldNumber = 1,
    kVelocitiesFieldNumber = 2,
    kAccelerationsFieldNumber = 3,
    kEffortFieldNumber = 4,
    kTimeFromStartFieldNumber = 5,
  };
  // repeated double positions = 1;
  int positions_size() const;
  private:
  int _internal_positions_size() const;
  public:
  void clear_positions();
  private:
  double _internal_positions(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_positions() const;
  void _internal_add_positions(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_positions();
  public:
  double positions(int index) const;
  void set_positions(int index, double value);
  void add_positions(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      positions() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_positions();

  // repeated double velocities = 2;
  int velocities_size() const;
  private:
  int _internal_velocities_size() const;
  public:
  void clear_velocities();
  private:
  double _internal_velocities(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_velocities() const;
  void _internal_add_velocities(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_velocities();
  public:
  double velocities(int index) const;
  void set_velocities(int index, double value);
  void add_velocities(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      velocities() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_velocities();

  // repeated double accelerations = 3;
  int accelerations_size() const;
  private:
  int _internal_accelerations_size() const;
  public:
  void clear_accelerations();
  private:
  double _internal_accelerations(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_accelerations() const;
  void _internal_add_accelerations(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_accelerations();
  public:
  double accelerations(int index) const;
  void set_accelerations(int index, double value);
  void add_accelerations(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      accelerations() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_accelerations();

  // repeated double effort = 4;
  int effort_size() const;
  private:
  int _internal_effort_size() const;
  public:
  void clear_effort();
  private:
  double _internal_effort(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_effort() const;
  void _internal_add_effort(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_effort();
  public:
  double effort(int index) const;
  void set_effort(int index, double value);
  void add_effort(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      effort() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_effort();

  // .ros.Duration time_from_start = 5;
  bool has_time_from_start() const;
  private:
  bool _internal_has_time_from_start() const;
  public:
  void clear_time_from_start();
  const ::ros::Duration& time_from_start() const;
  PROTOBUF_NODISCARD ::ros::Duration* release_time_from_start();
  ::ros::Duration* mutable_time_from_start();
  void set_allocated_time_from_start(::ros::Duration* time_from_start);
  private:
  const ::ros::Duration& _internal_time_from_start() const;
  ::ros::Duration* _internal_mutable_time_from_start();
  public:
  void unsafe_arena_set_allocated_time_from_start(
      ::ros::Duration* time_from_start);
  ::ros::Duration* unsafe_arena_release_time_from_start();

  // @@protoc_insertion_point(class_scope:ros.trajectory_msgs.JointTrajectoryPoint)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > positions_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > velocities_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > accelerations_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > effort_;
  ::ros::Duration* time_from_start_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2ftrajectory_5fmsgs_2fJointTrajectoryPoint_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// JointTrajectoryPoint

// repeated double positions = 1;
inline int JointTrajectoryPoint::_internal_positions_size() const {
  return positions_.size();
}
inline int JointTrajectoryPoint::positions_size() const {
  return _internal_positions_size();
}
inline void JointTrajectoryPoint::clear_positions() {
  positions_.Clear();
}
inline double JointTrajectoryPoint::_internal_positions(int index) const {
  return positions_.Get(index);
}
inline double JointTrajectoryPoint::positions(int index) const {
  // @@protoc_insertion_point(field_get:ros.trajectory_msgs.JointTrajectoryPoint.positions)
  return _internal_positions(index);
}
inline void JointTrajectoryPoint::set_positions(int index, double value) {
  positions_.Set(index, value);
  // @@protoc_insertion_point(field_set:ros.trajectory_msgs.JointTrajectoryPoint.positions)
}
inline void JointTrajectoryPoint::_internal_add_positions(double value) {
  positions_.Add(value);
}
inline void JointTrajectoryPoint::add_positions(double value) {
  _internal_add_positions(value);
  // @@protoc_insertion_point(field_add:ros.trajectory_msgs.JointTrajectoryPoint.positions)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
JointTrajectoryPoint::_internal_positions() const {
  return positions_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
JointTrajectoryPoint::positions() const {
  // @@protoc_insertion_point(field_list:ros.trajectory_msgs.JointTrajectoryPoint.positions)
  return _internal_positions();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
JointTrajectoryPoint::_internal_mutable_positions() {
  return &positions_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
JointTrajectoryPoint::mutable_positions() {
  // @@protoc_insertion_point(field_mutable_list:ros.trajectory_msgs.JointTrajectoryPoint.positions)
  return _internal_mutable_positions();
}

// repeated double velocities = 2;
inline int JointTrajectoryPoint::_internal_velocities_size() const {
  return velocities_.size();
}
inline int JointTrajectoryPoint::velocities_size() const {
  return _internal_velocities_size();
}
inline void JointTrajectoryPoint::clear_velocities() {
  velocities_.Clear();
}
inline double JointTrajectoryPoint::_internal_velocities(int index) const {
  return velocities_.Get(index);
}
inline double JointTrajectoryPoint::velocities(int index) const {
  // @@protoc_insertion_point(field_get:ros.trajectory_msgs.JointTrajectoryPoint.velocities)
  return _internal_velocities(index);
}
inline void JointTrajectoryPoint::set_velocities(int index, double value) {
  velocities_.Set(index, value);
  // @@protoc_insertion_point(field_set:ros.trajectory_msgs.JointTrajectoryPoint.velocities)
}
inline void JointTrajectoryPoint::_internal_add_velocities(double value) {
  velocities_.Add(value);
}
inline void JointTrajectoryPoint::add_velocities(double value) {
  _internal_add_velocities(value);
  // @@protoc_insertion_point(field_add:ros.trajectory_msgs.JointTrajectoryPoint.velocities)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
JointTrajectoryPoint::_internal_velocities() const {
  return velocities_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
JointTrajectoryPoint::velocities() const {
  // @@protoc_insertion_point(field_list:ros.trajectory_msgs.JointTrajectoryPoint.velocities)
  return _internal_velocities();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
JointTrajectoryPoint::_internal_mutable_velocities() {
  return &velocities_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
JointTrajectoryPoint::mutable_velocities() {
  // @@protoc_insertion_point(field_mutable_list:ros.trajectory_msgs.JointTrajectoryPoint.velocities)
  return _internal_mutable_velocities();
}

// repeated double accelerations = 3;
inline int JointTrajectoryPoint::_internal_accelerations_size() const {
  return accelerations_.size();
}
inline int JointTrajectoryPoint::accelerations_size() const {
  return _internal_accelerations_size();
}
inline void JointTrajectoryPoint::clear_accelerations() {
  accelerations_.Clear();
}
inline double JointTrajectoryPoint::_internal_accelerations(int index) const {
  return accelerations_.Get(index);
}
inline double JointTrajectoryPoint::accelerations(int index) const {
  // @@protoc_insertion_point(field_get:ros.trajectory_msgs.JointTrajectoryPoint.accelerations)
  return _internal_accelerations(index);
}
inline void JointTrajectoryPoint::set_accelerations(int index, double value) {
  accelerations_.Set(index, value);
  // @@protoc_insertion_point(field_set:ros.trajectory_msgs.JointTrajectoryPoint.accelerations)
}
inline void JointTrajectoryPoint::_internal_add_accelerations(double value) {
  accelerations_.Add(value);
}
inline void JointTrajectoryPoint::add_accelerations(double value) {
  _internal_add_accelerations(value);
  // @@protoc_insertion_point(field_add:ros.trajectory_msgs.JointTrajectoryPoint.accelerations)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
JointTrajectoryPoint::_internal_accelerations() const {
  return accelerations_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
JointTrajectoryPoint::accelerations() const {
  // @@protoc_insertion_point(field_list:ros.trajectory_msgs.JointTrajectoryPoint.accelerations)
  return _internal_accelerations();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
JointTrajectoryPoint::_internal_mutable_accelerations() {
  return &accelerations_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
JointTrajectoryPoint::mutable_accelerations() {
  // @@protoc_insertion_point(field_mutable_list:ros.trajectory_msgs.JointTrajectoryPoint.accelerations)
  return _internal_mutable_accelerations();
}

// repeated double effort = 4;
inline int JointTrajectoryPoint::_internal_effort_size() const {
  return effort_.size();
}
inline int JointTrajectoryPoint::effort_size() const {
  return _internal_effort_size();
}
inline void JointTrajectoryPoint::clear_effort() {
  effort_.Clear();
}
inline double JointTrajectoryPoint::_internal_effort(int index) const {
  return effort_.Get(index);
}
inline double JointTrajectoryPoint::effort(int index) const {
  // @@protoc_insertion_point(field_get:ros.trajectory_msgs.JointTrajectoryPoint.effort)
  return _internal_effort(index);
}
inline void JointTrajectoryPoint::set_effort(int index, double value) {
  effort_.Set(index, value);
  // @@protoc_insertion_point(field_set:ros.trajectory_msgs.JointTrajectoryPoint.effort)
}
inline void JointTrajectoryPoint::_internal_add_effort(double value) {
  effort_.Add(value);
}
inline void JointTrajectoryPoint::add_effort(double value) {
  _internal_add_effort(value);
  // @@protoc_insertion_point(field_add:ros.trajectory_msgs.JointTrajectoryPoint.effort)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
JointTrajectoryPoint::_internal_effort() const {
  return effort_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
JointTrajectoryPoint::effort() const {
  // @@protoc_insertion_point(field_list:ros.trajectory_msgs.JointTrajectoryPoint.effort)
  return _internal_effort();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
JointTrajectoryPoint::_internal_mutable_effort() {
  return &effort_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
JointTrajectoryPoint::mutable_effort() {
  // @@protoc_insertion_point(field_mutable_list:ros.trajectory_msgs.JointTrajectoryPoint.effort)
  return _internal_mutable_effort();
}

// .ros.Duration time_from_start = 5;
inline bool JointTrajectoryPoint::_internal_has_time_from_start() const {
  return this != internal_default_instance() && time_from_start_ != nullptr;
}
inline bool JointTrajectoryPoint::has_time_from_start() const {
  return _internal_has_time_from_start();
}
inline const ::ros::Duration& JointTrajectoryPoint::_internal_time_from_start() const {
  const ::ros::Duration* p = time_from_start_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::Duration&>(
      ::ros::_Duration_default_instance_);
}
inline const ::ros::Duration& JointTrajectoryPoint::time_from_start() const {
  // @@protoc_insertion_point(field_get:ros.trajectory_msgs.JointTrajectoryPoint.time_from_start)
  return _internal_time_from_start();
}
inline void JointTrajectoryPoint::unsafe_arena_set_allocated_time_from_start(
    ::ros::Duration* time_from_start) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(time_from_start_);
  }
  time_from_start_ = time_from_start;
  if (time_from_start) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.trajectory_msgs.JointTrajectoryPoint.time_from_start)
}
inline ::ros::Duration* JointTrajectoryPoint::release_time_from_start() {
  
  ::ros::Duration* temp = time_from_start_;
  time_from_start_ = nullptr;
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
inline ::ros::Duration* JointTrajectoryPoint::unsafe_arena_release_time_from_start() {
  // @@protoc_insertion_point(field_release:ros.trajectory_msgs.JointTrajectoryPoint.time_from_start)
  
  ::ros::Duration* temp = time_from_start_;
  time_from_start_ = nullptr;
  return temp;
}
inline ::ros::Duration* JointTrajectoryPoint::_internal_mutable_time_from_start() {
  
  if (time_from_start_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::Duration>(GetArenaForAllocation());
    time_from_start_ = p;
  }
  return time_from_start_;
}
inline ::ros::Duration* JointTrajectoryPoint::mutable_time_from_start() {
  ::ros::Duration* _msg = _internal_mutable_time_from_start();
  // @@protoc_insertion_point(field_mutable:ros.trajectory_msgs.JointTrajectoryPoint.time_from_start)
  return _msg;
}
inline void JointTrajectoryPoint::set_allocated_time_from_start(::ros::Duration* time_from_start) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(time_from_start_);
  }
  if (time_from_start) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(time_from_start));
    if (message_arena != submessage_arena) {
      time_from_start = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, time_from_start, submessage_arena);
    }
    
  } else {
    
  }
  time_from_start_ = time_from_start;
  // @@protoc_insertion_point(field_set_allocated:ros.trajectory_msgs.JointTrajectoryPoint.time_from_start)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace trajectory_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2ftrajectory_5fmsgs_2fJointTrajectoryPoint_2eproto
