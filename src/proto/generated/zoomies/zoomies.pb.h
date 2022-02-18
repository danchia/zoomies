// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: zoomies/zoomies.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_zoomies_2fzoomies_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_zoomies_2fzoomies_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_zoomies_2fzoomies_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_zoomies_2fzoomies_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_zoomies_2fzoomies_2eproto;
namespace zoomies {
class DriverLog;
struct DriverLogDefaultTypeInternal;
extern DriverLogDefaultTypeInternal _DriverLog_default_instance_;
}  // namespace zoomies
PROTOBUF_NAMESPACE_OPEN
template<> ::zoomies::DriverLog* Arena::CreateMaybeMessage<::zoomies::DriverLog>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace zoomies {

// ===================================================================

class DriverLog final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:zoomies.DriverLog) */ {
 public:
  inline DriverLog() : DriverLog(nullptr) {}
  ~DriverLog() override;
  explicit constexpr DriverLog(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  DriverLog(const DriverLog& from);
  DriverLog(DriverLog&& from) noexcept
    : DriverLog() {
    *this = ::std::move(from);
  }

  inline DriverLog& operator=(const DriverLog& from) {
    CopyFrom(from);
    return *this;
  }
  inline DriverLog& operator=(DriverLog&& from) noexcept {
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
  static const DriverLog& default_instance() {
    return *internal_default_instance();
  }
  static inline const DriverLog* internal_default_instance() {
    return reinterpret_cast<const DriverLog*>(
               &_DriverLog_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(DriverLog& a, DriverLog& b) {
    a.Swap(&b);
  }
  inline void Swap(DriverLog* other) {
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
  void UnsafeArenaSwap(DriverLog* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  DriverLog* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<DriverLog>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const DriverLog& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const DriverLog& from);
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
  void InternalSwap(DriverLog* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "zoomies.DriverLog";
  }
  protected:
  explicit DriverLog(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kImuAccelFieldNumber = 17,
    kImuRotationFieldNumber = 18,
    kTUsFieldNumber = 1,
    kLinearVelocityFieldNumber = 2,
    kAngularVelocityFieldNumber = 3,
    kDesiredLinearVelocityFieldNumber = 4,
    kDesiredAngularVelocityFieldNumber = 5,
    kHeadingFieldNumber = 6,
    kXFieldNumber = 7,
    kYFieldNumber = 8,
    kTotalDistanceFieldNumber = 9,
    kRacingPathDistFieldNumber = 10,
    kDistDeltaFieldNumber = 11,
    kHeadingDeltaFieldNumber = 12,
    kDistStddevFieldNumber = 13,
    kHeadingStddevFieldNumber = 14,
    kEscFieldNumber = 15,
    kSteerFieldNumber = 16,
  };
  // .ros.geometry_msgs.Vector3 imu_accel = 17;
  bool has_imu_accel() const;
  private:
  bool _internal_has_imu_accel() const;
  public:
  void clear_imu_accel();
  const ::ros::geometry_msgs::Vector3& imu_accel() const;
  PROTOBUF_NODISCARD ::ros::geometry_msgs::Vector3* release_imu_accel();
  ::ros::geometry_msgs::Vector3* mutable_imu_accel();
  void set_allocated_imu_accel(::ros::geometry_msgs::Vector3* imu_accel);
  private:
  const ::ros::geometry_msgs::Vector3& _internal_imu_accel() const;
  ::ros::geometry_msgs::Vector3* _internal_mutable_imu_accel();
  public:
  void unsafe_arena_set_allocated_imu_accel(
      ::ros::geometry_msgs::Vector3* imu_accel);
  ::ros::geometry_msgs::Vector3* unsafe_arena_release_imu_accel();

  // .ros.geometry_msgs.Vector3 imu_rotation = 18;
  bool has_imu_rotation() const;
  private:
  bool _internal_has_imu_rotation() const;
  public:
  void clear_imu_rotation();
  const ::ros::geometry_msgs::Vector3& imu_rotation() const;
  PROTOBUF_NODISCARD ::ros::geometry_msgs::Vector3* release_imu_rotation();
  ::ros::geometry_msgs::Vector3* mutable_imu_rotation();
  void set_allocated_imu_rotation(::ros::geometry_msgs::Vector3* imu_rotation);
  private:
  const ::ros::geometry_msgs::Vector3& _internal_imu_rotation() const;
  ::ros::geometry_msgs::Vector3* _internal_mutable_imu_rotation();
  public:
  void unsafe_arena_set_allocated_imu_rotation(
      ::ros::geometry_msgs::Vector3* imu_rotation);
  ::ros::geometry_msgs::Vector3* unsafe_arena_release_imu_rotation();

  // fixed64 t_us = 1;
  void clear_t_us();
  uint64_t t_us() const;
  void set_t_us(uint64_t value);
  private:
  uint64_t _internal_t_us() const;
  void _internal_set_t_us(uint64_t value);
  public:

  // float linear_velocity = 2;
  void clear_linear_velocity();
  float linear_velocity() const;
  void set_linear_velocity(float value);
  private:
  float _internal_linear_velocity() const;
  void _internal_set_linear_velocity(float value);
  public:

  // float angular_velocity = 3;
  void clear_angular_velocity();
  float angular_velocity() const;
  void set_angular_velocity(float value);
  private:
  float _internal_angular_velocity() const;
  void _internal_set_angular_velocity(float value);
  public:

  // float desired_linear_velocity = 4;
  void clear_desired_linear_velocity();
  float desired_linear_velocity() const;
  void set_desired_linear_velocity(float value);
  private:
  float _internal_desired_linear_velocity() const;
  void _internal_set_desired_linear_velocity(float value);
  public:

  // float desired_angular_velocity = 5;
  void clear_desired_angular_velocity();
  float desired_angular_velocity() const;
  void set_desired_angular_velocity(float value);
  private:
  float _internal_desired_angular_velocity() const;
  void _internal_set_desired_angular_velocity(float value);
  public:

  // float heading = 6;
  void clear_heading();
  float heading() const;
  void set_heading(float value);
  private:
  float _internal_heading() const;
  void _internal_set_heading(float value);
  public:

  // float x = 7;
  void clear_x();
  float x() const;
  void set_x(float value);
  private:
  float _internal_x() const;
  void _internal_set_x(float value);
  public:

  // float y = 8;
  void clear_y();
  float y() const;
  void set_y(float value);
  private:
  float _internal_y() const;
  void _internal_set_y(float value);
  public:

  // float total_distance = 9;
  void clear_total_distance();
  float total_distance() const;
  void set_total_distance(float value);
  private:
  float _internal_total_distance() const;
  void _internal_set_total_distance(float value);
  public:

  // float racing_path_dist = 10;
  void clear_racing_path_dist();
  float racing_path_dist() const;
  void set_racing_path_dist(float value);
  private:
  float _internal_racing_path_dist() const;
  void _internal_set_racing_path_dist(float value);
  public:

  // float dist_delta = 11;
  void clear_dist_delta();
  float dist_delta() const;
  void set_dist_delta(float value);
  private:
  float _internal_dist_delta() const;
  void _internal_set_dist_delta(float value);
  public:

  // float heading_delta = 12;
  void clear_heading_delta();
  float heading_delta() const;
  void set_heading_delta(float value);
  private:
  float _internal_heading_delta() const;
  void _internal_set_heading_delta(float value);
  public:

  // float dist_stddev = 13;
  void clear_dist_stddev();
  float dist_stddev() const;
  void set_dist_stddev(float value);
  private:
  float _internal_dist_stddev() const;
  void _internal_set_dist_stddev(float value);
  public:

  // float heading_stddev = 14;
  void clear_heading_stddev();
  float heading_stddev() const;
  void set_heading_stddev(float value);
  private:
  float _internal_heading_stddev() const;
  void _internal_set_heading_stddev(float value);
  public:

  // float esc = 15;
  void clear_esc();
  float esc() const;
  void set_esc(float value);
  private:
  float _internal_esc() const;
  void _internal_set_esc(float value);
  public:

  // float steer = 16;
  void clear_steer();
  float steer() const;
  void set_steer(float value);
  private:
  float _internal_steer() const;
  void _internal_set_steer(float value);
  public:

  // @@protoc_insertion_point(class_scope:zoomies.DriverLog)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::ros::geometry_msgs::Vector3* imu_accel_;
  ::ros::geometry_msgs::Vector3* imu_rotation_;
  uint64_t t_us_;
  float linear_velocity_;
  float angular_velocity_;
  float desired_linear_velocity_;
  float desired_angular_velocity_;
  float heading_;
  float x_;
  float y_;
  float total_distance_;
  float racing_path_dist_;
  float dist_delta_;
  float heading_delta_;
  float dist_stddev_;
  float heading_stddev_;
  float esc_;
  float steer_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_zoomies_2fzoomies_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DriverLog

// fixed64 t_us = 1;
inline void DriverLog::clear_t_us() {
  t_us_ = uint64_t{0u};
}
inline uint64_t DriverLog::_internal_t_us() const {
  return t_us_;
}
inline uint64_t DriverLog::t_us() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.t_us)
  return _internal_t_us();
}
inline void DriverLog::_internal_set_t_us(uint64_t value) {
  
  t_us_ = value;
}
inline void DriverLog::set_t_us(uint64_t value) {
  _internal_set_t_us(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.t_us)
}

// float linear_velocity = 2;
inline void DriverLog::clear_linear_velocity() {
  linear_velocity_ = 0;
}
inline float DriverLog::_internal_linear_velocity() const {
  return linear_velocity_;
}
inline float DriverLog::linear_velocity() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.linear_velocity)
  return _internal_linear_velocity();
}
inline void DriverLog::_internal_set_linear_velocity(float value) {
  
  linear_velocity_ = value;
}
inline void DriverLog::set_linear_velocity(float value) {
  _internal_set_linear_velocity(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.linear_velocity)
}

// float angular_velocity = 3;
inline void DriverLog::clear_angular_velocity() {
  angular_velocity_ = 0;
}
inline float DriverLog::_internal_angular_velocity() const {
  return angular_velocity_;
}
inline float DriverLog::angular_velocity() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.angular_velocity)
  return _internal_angular_velocity();
}
inline void DriverLog::_internal_set_angular_velocity(float value) {
  
  angular_velocity_ = value;
}
inline void DriverLog::set_angular_velocity(float value) {
  _internal_set_angular_velocity(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.angular_velocity)
}

// float desired_linear_velocity = 4;
inline void DriverLog::clear_desired_linear_velocity() {
  desired_linear_velocity_ = 0;
}
inline float DriverLog::_internal_desired_linear_velocity() const {
  return desired_linear_velocity_;
}
inline float DriverLog::desired_linear_velocity() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.desired_linear_velocity)
  return _internal_desired_linear_velocity();
}
inline void DriverLog::_internal_set_desired_linear_velocity(float value) {
  
  desired_linear_velocity_ = value;
}
inline void DriverLog::set_desired_linear_velocity(float value) {
  _internal_set_desired_linear_velocity(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.desired_linear_velocity)
}

// float desired_angular_velocity = 5;
inline void DriverLog::clear_desired_angular_velocity() {
  desired_angular_velocity_ = 0;
}
inline float DriverLog::_internal_desired_angular_velocity() const {
  return desired_angular_velocity_;
}
inline float DriverLog::desired_angular_velocity() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.desired_angular_velocity)
  return _internal_desired_angular_velocity();
}
inline void DriverLog::_internal_set_desired_angular_velocity(float value) {
  
  desired_angular_velocity_ = value;
}
inline void DriverLog::set_desired_angular_velocity(float value) {
  _internal_set_desired_angular_velocity(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.desired_angular_velocity)
}

// float heading = 6;
inline void DriverLog::clear_heading() {
  heading_ = 0;
}
inline float DriverLog::_internal_heading() const {
  return heading_;
}
inline float DriverLog::heading() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.heading)
  return _internal_heading();
}
inline void DriverLog::_internal_set_heading(float value) {
  
  heading_ = value;
}
inline void DriverLog::set_heading(float value) {
  _internal_set_heading(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.heading)
}

// float x = 7;
inline void DriverLog::clear_x() {
  x_ = 0;
}
inline float DriverLog::_internal_x() const {
  return x_;
}
inline float DriverLog::x() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.x)
  return _internal_x();
}
inline void DriverLog::_internal_set_x(float value) {
  
  x_ = value;
}
inline void DriverLog::set_x(float value) {
  _internal_set_x(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.x)
}

// float y = 8;
inline void DriverLog::clear_y() {
  y_ = 0;
}
inline float DriverLog::_internal_y() const {
  return y_;
}
inline float DriverLog::y() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.y)
  return _internal_y();
}
inline void DriverLog::_internal_set_y(float value) {
  
  y_ = value;
}
inline void DriverLog::set_y(float value) {
  _internal_set_y(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.y)
}

// float total_distance = 9;
inline void DriverLog::clear_total_distance() {
  total_distance_ = 0;
}
inline float DriverLog::_internal_total_distance() const {
  return total_distance_;
}
inline float DriverLog::total_distance() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.total_distance)
  return _internal_total_distance();
}
inline void DriverLog::_internal_set_total_distance(float value) {
  
  total_distance_ = value;
}
inline void DriverLog::set_total_distance(float value) {
  _internal_set_total_distance(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.total_distance)
}

// float racing_path_dist = 10;
inline void DriverLog::clear_racing_path_dist() {
  racing_path_dist_ = 0;
}
inline float DriverLog::_internal_racing_path_dist() const {
  return racing_path_dist_;
}
inline float DriverLog::racing_path_dist() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.racing_path_dist)
  return _internal_racing_path_dist();
}
inline void DriverLog::_internal_set_racing_path_dist(float value) {
  
  racing_path_dist_ = value;
}
inline void DriverLog::set_racing_path_dist(float value) {
  _internal_set_racing_path_dist(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.racing_path_dist)
}

// float dist_delta = 11;
inline void DriverLog::clear_dist_delta() {
  dist_delta_ = 0;
}
inline float DriverLog::_internal_dist_delta() const {
  return dist_delta_;
}
inline float DriverLog::dist_delta() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.dist_delta)
  return _internal_dist_delta();
}
inline void DriverLog::_internal_set_dist_delta(float value) {
  
  dist_delta_ = value;
}
inline void DriverLog::set_dist_delta(float value) {
  _internal_set_dist_delta(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.dist_delta)
}

// float heading_delta = 12;
inline void DriverLog::clear_heading_delta() {
  heading_delta_ = 0;
}
inline float DriverLog::_internal_heading_delta() const {
  return heading_delta_;
}
inline float DriverLog::heading_delta() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.heading_delta)
  return _internal_heading_delta();
}
inline void DriverLog::_internal_set_heading_delta(float value) {
  
  heading_delta_ = value;
}
inline void DriverLog::set_heading_delta(float value) {
  _internal_set_heading_delta(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.heading_delta)
}

// float dist_stddev = 13;
inline void DriverLog::clear_dist_stddev() {
  dist_stddev_ = 0;
}
inline float DriverLog::_internal_dist_stddev() const {
  return dist_stddev_;
}
inline float DriverLog::dist_stddev() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.dist_stddev)
  return _internal_dist_stddev();
}
inline void DriverLog::_internal_set_dist_stddev(float value) {
  
  dist_stddev_ = value;
}
inline void DriverLog::set_dist_stddev(float value) {
  _internal_set_dist_stddev(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.dist_stddev)
}

// float heading_stddev = 14;
inline void DriverLog::clear_heading_stddev() {
  heading_stddev_ = 0;
}
inline float DriverLog::_internal_heading_stddev() const {
  return heading_stddev_;
}
inline float DriverLog::heading_stddev() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.heading_stddev)
  return _internal_heading_stddev();
}
inline void DriverLog::_internal_set_heading_stddev(float value) {
  
  heading_stddev_ = value;
}
inline void DriverLog::set_heading_stddev(float value) {
  _internal_set_heading_stddev(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.heading_stddev)
}

// float esc = 15;
inline void DriverLog::clear_esc() {
  esc_ = 0;
}
inline float DriverLog::_internal_esc() const {
  return esc_;
}
inline float DriverLog::esc() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.esc)
  return _internal_esc();
}
inline void DriverLog::_internal_set_esc(float value) {
  
  esc_ = value;
}
inline void DriverLog::set_esc(float value) {
  _internal_set_esc(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.esc)
}

// float steer = 16;
inline void DriverLog::clear_steer() {
  steer_ = 0;
}
inline float DriverLog::_internal_steer() const {
  return steer_;
}
inline float DriverLog::steer() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.steer)
  return _internal_steer();
}
inline void DriverLog::_internal_set_steer(float value) {
  
  steer_ = value;
}
inline void DriverLog::set_steer(float value) {
  _internal_set_steer(value);
  // @@protoc_insertion_point(field_set:zoomies.DriverLog.steer)
}

// .ros.geometry_msgs.Vector3 imu_accel = 17;
inline bool DriverLog::_internal_has_imu_accel() const {
  return this != internal_default_instance() && imu_accel_ != nullptr;
}
inline bool DriverLog::has_imu_accel() const {
  return _internal_has_imu_accel();
}
inline const ::ros::geometry_msgs::Vector3& DriverLog::_internal_imu_accel() const {
  const ::ros::geometry_msgs::Vector3* p = imu_accel_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::geometry_msgs::Vector3&>(
      ::ros::geometry_msgs::_Vector3_default_instance_);
}
inline const ::ros::geometry_msgs::Vector3& DriverLog::imu_accel() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.imu_accel)
  return _internal_imu_accel();
}
inline void DriverLog::unsafe_arena_set_allocated_imu_accel(
    ::ros::geometry_msgs::Vector3* imu_accel) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(imu_accel_);
  }
  imu_accel_ = imu_accel;
  if (imu_accel) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:zoomies.DriverLog.imu_accel)
}
inline ::ros::geometry_msgs::Vector3* DriverLog::release_imu_accel() {
  
  ::ros::geometry_msgs::Vector3* temp = imu_accel_;
  imu_accel_ = nullptr;
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
inline ::ros::geometry_msgs::Vector3* DriverLog::unsafe_arena_release_imu_accel() {
  // @@protoc_insertion_point(field_release:zoomies.DriverLog.imu_accel)
  
  ::ros::geometry_msgs::Vector3* temp = imu_accel_;
  imu_accel_ = nullptr;
  return temp;
}
inline ::ros::geometry_msgs::Vector3* DriverLog::_internal_mutable_imu_accel() {
  
  if (imu_accel_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::geometry_msgs::Vector3>(GetArenaForAllocation());
    imu_accel_ = p;
  }
  return imu_accel_;
}
inline ::ros::geometry_msgs::Vector3* DriverLog::mutable_imu_accel() {
  ::ros::geometry_msgs::Vector3* _msg = _internal_mutable_imu_accel();
  // @@protoc_insertion_point(field_mutable:zoomies.DriverLog.imu_accel)
  return _msg;
}
inline void DriverLog::set_allocated_imu_accel(::ros::geometry_msgs::Vector3* imu_accel) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(imu_accel_);
  }
  if (imu_accel) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(imu_accel));
    if (message_arena != submessage_arena) {
      imu_accel = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, imu_accel, submessage_arena);
    }
    
  } else {
    
  }
  imu_accel_ = imu_accel;
  // @@protoc_insertion_point(field_set_allocated:zoomies.DriverLog.imu_accel)
}

// .ros.geometry_msgs.Vector3 imu_rotation = 18;
inline bool DriverLog::_internal_has_imu_rotation() const {
  return this != internal_default_instance() && imu_rotation_ != nullptr;
}
inline bool DriverLog::has_imu_rotation() const {
  return _internal_has_imu_rotation();
}
inline const ::ros::geometry_msgs::Vector3& DriverLog::_internal_imu_rotation() const {
  const ::ros::geometry_msgs::Vector3* p = imu_rotation_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::geometry_msgs::Vector3&>(
      ::ros::geometry_msgs::_Vector3_default_instance_);
}
inline const ::ros::geometry_msgs::Vector3& DriverLog::imu_rotation() const {
  // @@protoc_insertion_point(field_get:zoomies.DriverLog.imu_rotation)
  return _internal_imu_rotation();
}
inline void DriverLog::unsafe_arena_set_allocated_imu_rotation(
    ::ros::geometry_msgs::Vector3* imu_rotation) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(imu_rotation_);
  }
  imu_rotation_ = imu_rotation;
  if (imu_rotation) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:zoomies.DriverLog.imu_rotation)
}
inline ::ros::geometry_msgs::Vector3* DriverLog::release_imu_rotation() {
  
  ::ros::geometry_msgs::Vector3* temp = imu_rotation_;
  imu_rotation_ = nullptr;
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
inline ::ros::geometry_msgs::Vector3* DriverLog::unsafe_arena_release_imu_rotation() {
  // @@protoc_insertion_point(field_release:zoomies.DriverLog.imu_rotation)
  
  ::ros::geometry_msgs::Vector3* temp = imu_rotation_;
  imu_rotation_ = nullptr;
  return temp;
}
inline ::ros::geometry_msgs::Vector3* DriverLog::_internal_mutable_imu_rotation() {
  
  if (imu_rotation_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::geometry_msgs::Vector3>(GetArenaForAllocation());
    imu_rotation_ = p;
  }
  return imu_rotation_;
}
inline ::ros::geometry_msgs::Vector3* DriverLog::mutable_imu_rotation() {
  ::ros::geometry_msgs::Vector3* _msg = _internal_mutable_imu_rotation();
  // @@protoc_insertion_point(field_mutable:zoomies.DriverLog.imu_rotation)
  return _msg;
}
inline void DriverLog::set_allocated_imu_rotation(::ros::geometry_msgs::Vector3* imu_rotation) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(imu_rotation_);
  }
  if (imu_rotation) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(imu_rotation));
    if (message_arena != submessage_arena) {
      imu_rotation = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, imu_rotation, submessage_arena);
    }
    
  } else {
    
  }
  imu_rotation_ = imu_rotation;
  // @@protoc_insertion_point(field_set_allocated:zoomies.DriverLog.imu_rotation)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace zoomies

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_zoomies_2fzoomies_2eproto
