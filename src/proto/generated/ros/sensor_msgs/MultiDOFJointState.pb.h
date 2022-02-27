// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/MultiDOFJointState.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto

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
#include "ros/geometry_msgs/Transform.pb.h"
#include "ros/geometry_msgs/Twist.pb.h"
#include "ros/geometry_msgs/Wrench.pb.h"
#include "ros/std_msgs/Header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto;
namespace ros {
namespace sensor_msgs {
class MultiDOFJointState;
struct MultiDOFJointStateDefaultTypeInternal;
extern MultiDOFJointStateDefaultTypeInternal _MultiDOFJointState_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::sensor_msgs::MultiDOFJointState* Arena::CreateMaybeMessage<::ros::sensor_msgs::MultiDOFJointState>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace sensor_msgs {

// ===================================================================

class MultiDOFJointState final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.sensor_msgs.MultiDOFJointState) */ {
 public:
  inline MultiDOFJointState() : MultiDOFJointState(nullptr) {}
  ~MultiDOFJointState() override;
  explicit constexpr MultiDOFJointState(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  MultiDOFJointState(const MultiDOFJointState& from);
  MultiDOFJointState(MultiDOFJointState&& from) noexcept
    : MultiDOFJointState() {
    *this = ::std::move(from);
  }

  inline MultiDOFJointState& operator=(const MultiDOFJointState& from) {
    CopyFrom(from);
    return *this;
  }
  inline MultiDOFJointState& operator=(MultiDOFJointState&& from) noexcept {
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
  static const MultiDOFJointState& default_instance() {
    return *internal_default_instance();
  }
  static inline const MultiDOFJointState* internal_default_instance() {
    return reinterpret_cast<const MultiDOFJointState*>(
               &_MultiDOFJointState_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MultiDOFJointState& a, MultiDOFJointState& b) {
    a.Swap(&b);
  }
  inline void Swap(MultiDOFJointState* other) {
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
  void UnsafeArenaSwap(MultiDOFJointState* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  MultiDOFJointState* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<MultiDOFJointState>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const MultiDOFJointState& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const MultiDOFJointState& from);
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
  void InternalSwap(MultiDOFJointState* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.sensor_msgs.MultiDOFJointState";
  }
  protected:
  explicit MultiDOFJointState(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kJointNamesFieldNumber = 2,
    kTransformsFieldNumber = 3,
    kTwistFieldNumber = 4,
    kWrenchFieldNumber = 5,
    kHeaderFieldNumber = 1,
  };
  // repeated string joint_names = 2;
  int joint_names_size() const;
  private:
  int _internal_joint_names_size() const;
  public:
  void clear_joint_names();
  const std::string& joint_names(int index) const;
  std::string* mutable_joint_names(int index);
  void set_joint_names(int index, const std::string& value);
  void set_joint_names(int index, std::string&& value);
  void set_joint_names(int index, const char* value);
  void set_joint_names(int index, const char* value, size_t size);
  std::string* add_joint_names();
  void add_joint_names(const std::string& value);
  void add_joint_names(std::string&& value);
  void add_joint_names(const char* value);
  void add_joint_names(const char* value, size_t size);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>& joint_names() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>* mutable_joint_names();
  private:
  const std::string& _internal_joint_names(int index) const;
  std::string* _internal_add_joint_names();
  public:

  // repeated .ros.geometry_msgs.Transform transforms = 3;
  int transforms_size() const;
  private:
  int _internal_transforms_size() const;
  public:
  void clear_transforms();
  ::ros::geometry_msgs::Transform* mutable_transforms(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Transform >*
      mutable_transforms();
  private:
  const ::ros::geometry_msgs::Transform& _internal_transforms(int index) const;
  ::ros::geometry_msgs::Transform* _internal_add_transforms();
  public:
  const ::ros::geometry_msgs::Transform& transforms(int index) const;
  ::ros::geometry_msgs::Transform* add_transforms();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Transform >&
      transforms() const;

  // repeated .ros.geometry_msgs.Twist twist = 4;
  int twist_size() const;
  private:
  int _internal_twist_size() const;
  public:
  void clear_twist();
  ::ros::geometry_msgs::Twist* mutable_twist(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Twist >*
      mutable_twist();
  private:
  const ::ros::geometry_msgs::Twist& _internal_twist(int index) const;
  ::ros::geometry_msgs::Twist* _internal_add_twist();
  public:
  const ::ros::geometry_msgs::Twist& twist(int index) const;
  ::ros::geometry_msgs::Twist* add_twist();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Twist >&
      twist() const;

  // repeated .ros.geometry_msgs.Wrench wrench = 5;
  int wrench_size() const;
  private:
  int _internal_wrench_size() const;
  public:
  void clear_wrench();
  ::ros::geometry_msgs::Wrench* mutable_wrench(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Wrench >*
      mutable_wrench();
  private:
  const ::ros::geometry_msgs::Wrench& _internal_wrench(int index) const;
  ::ros::geometry_msgs::Wrench* _internal_add_wrench();
  public:
  const ::ros::geometry_msgs::Wrench& wrench(int index) const;
  ::ros::geometry_msgs::Wrench* add_wrench();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Wrench >&
      wrench() const;

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

  // @@protoc_insertion_point(class_scope:ros.sensor_msgs.MultiDOFJointState)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string> joint_names_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Transform > transforms_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Twist > twist_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Wrench > wrench_;
  ::ros::std_msgs::Header* header_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MultiDOFJointState

// .ros.std_msgs.Header header = 1;
inline bool MultiDOFJointState::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool MultiDOFJointState::has_header() const {
  return _internal_has_header();
}
inline const ::ros::std_msgs::Header& MultiDOFJointState::_internal_header() const {
  const ::ros::std_msgs::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::std_msgs::Header&>(
      ::ros::std_msgs::_Header_default_instance_);
}
inline const ::ros::std_msgs::Header& MultiDOFJointState::header() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.MultiDOFJointState.header)
  return _internal_header();
}
inline void MultiDOFJointState::unsafe_arena_set_allocated_header(
    ::ros::std_msgs::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.sensor_msgs.MultiDOFJointState.header)
}
inline ::ros::std_msgs::Header* MultiDOFJointState::release_header() {
  
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
inline ::ros::std_msgs::Header* MultiDOFJointState::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:ros.sensor_msgs.MultiDOFJointState.header)
  
  ::ros::std_msgs::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::ros::std_msgs::Header* MultiDOFJointState::_internal_mutable_header() {
  
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::std_msgs::Header>(GetArenaForAllocation());
    header_ = p;
  }
  return header_;
}
inline ::ros::std_msgs::Header* MultiDOFJointState::mutable_header() {
  ::ros::std_msgs::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.MultiDOFJointState.header)
  return _msg;
}
inline void MultiDOFJointState::set_allocated_header(::ros::std_msgs::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:ros.sensor_msgs.MultiDOFJointState.header)
}

// repeated string joint_names = 2;
inline int MultiDOFJointState::_internal_joint_names_size() const {
  return joint_names_.size();
}
inline int MultiDOFJointState::joint_names_size() const {
  return _internal_joint_names_size();
}
inline void MultiDOFJointState::clear_joint_names() {
  joint_names_.Clear();
}
inline std::string* MultiDOFJointState::add_joint_names() {
  std::string* _s = _internal_add_joint_names();
  // @@protoc_insertion_point(field_add_mutable:ros.sensor_msgs.MultiDOFJointState.joint_names)
  return _s;
}
inline const std::string& MultiDOFJointState::_internal_joint_names(int index) const {
  return joint_names_.Get(index);
}
inline const std::string& MultiDOFJointState::joint_names(int index) const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.MultiDOFJointState.joint_names)
  return _internal_joint_names(index);
}
inline std::string* MultiDOFJointState::mutable_joint_names(int index) {
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.MultiDOFJointState.joint_names)
  return joint_names_.Mutable(index);
}
inline void MultiDOFJointState::set_joint_names(int index, const std::string& value) {
  joint_names_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.MultiDOFJointState.joint_names)
}
inline void MultiDOFJointState::set_joint_names(int index, std::string&& value) {
  joint_names_.Mutable(index)->assign(std::move(value));
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.MultiDOFJointState.joint_names)
}
inline void MultiDOFJointState::set_joint_names(int index, const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  joint_names_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:ros.sensor_msgs.MultiDOFJointState.joint_names)
}
inline void MultiDOFJointState::set_joint_names(int index, const char* value, size_t size) {
  joint_names_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:ros.sensor_msgs.MultiDOFJointState.joint_names)
}
inline std::string* MultiDOFJointState::_internal_add_joint_names() {
  return joint_names_.Add();
}
inline void MultiDOFJointState::add_joint_names(const std::string& value) {
  joint_names_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:ros.sensor_msgs.MultiDOFJointState.joint_names)
}
inline void MultiDOFJointState::add_joint_names(std::string&& value) {
  joint_names_.Add(std::move(value));
  // @@protoc_insertion_point(field_add:ros.sensor_msgs.MultiDOFJointState.joint_names)
}
inline void MultiDOFJointState::add_joint_names(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  joint_names_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:ros.sensor_msgs.MultiDOFJointState.joint_names)
}
inline void MultiDOFJointState::add_joint_names(const char* value, size_t size) {
  joint_names_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:ros.sensor_msgs.MultiDOFJointState.joint_names)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>&
MultiDOFJointState::joint_names() const {
  // @@protoc_insertion_point(field_list:ros.sensor_msgs.MultiDOFJointState.joint_names)
  return joint_names_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField<std::string>*
MultiDOFJointState::mutable_joint_names() {
  // @@protoc_insertion_point(field_mutable_list:ros.sensor_msgs.MultiDOFJointState.joint_names)
  return &joint_names_;
}

// repeated .ros.geometry_msgs.Transform transforms = 3;
inline int MultiDOFJointState::_internal_transforms_size() const {
  return transforms_.size();
}
inline int MultiDOFJointState::transforms_size() const {
  return _internal_transforms_size();
}
inline ::ros::geometry_msgs::Transform* MultiDOFJointState::mutable_transforms(int index) {
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.MultiDOFJointState.transforms)
  return transforms_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Transform >*
MultiDOFJointState::mutable_transforms() {
  // @@protoc_insertion_point(field_mutable_list:ros.sensor_msgs.MultiDOFJointState.transforms)
  return &transforms_;
}
inline const ::ros::geometry_msgs::Transform& MultiDOFJointState::_internal_transforms(int index) const {
  return transforms_.Get(index);
}
inline const ::ros::geometry_msgs::Transform& MultiDOFJointState::transforms(int index) const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.MultiDOFJointState.transforms)
  return _internal_transforms(index);
}
inline ::ros::geometry_msgs::Transform* MultiDOFJointState::_internal_add_transforms() {
  return transforms_.Add();
}
inline ::ros::geometry_msgs::Transform* MultiDOFJointState::add_transforms() {
  ::ros::geometry_msgs::Transform* _add = _internal_add_transforms();
  // @@protoc_insertion_point(field_add:ros.sensor_msgs.MultiDOFJointState.transforms)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Transform >&
MultiDOFJointState::transforms() const {
  // @@protoc_insertion_point(field_list:ros.sensor_msgs.MultiDOFJointState.transforms)
  return transforms_;
}

// repeated .ros.geometry_msgs.Twist twist = 4;
inline int MultiDOFJointState::_internal_twist_size() const {
  return twist_.size();
}
inline int MultiDOFJointState::twist_size() const {
  return _internal_twist_size();
}
inline ::ros::geometry_msgs::Twist* MultiDOFJointState::mutable_twist(int index) {
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.MultiDOFJointState.twist)
  return twist_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Twist >*
MultiDOFJointState::mutable_twist() {
  // @@protoc_insertion_point(field_mutable_list:ros.sensor_msgs.MultiDOFJointState.twist)
  return &twist_;
}
inline const ::ros::geometry_msgs::Twist& MultiDOFJointState::_internal_twist(int index) const {
  return twist_.Get(index);
}
inline const ::ros::geometry_msgs::Twist& MultiDOFJointState::twist(int index) const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.MultiDOFJointState.twist)
  return _internal_twist(index);
}
inline ::ros::geometry_msgs::Twist* MultiDOFJointState::_internal_add_twist() {
  return twist_.Add();
}
inline ::ros::geometry_msgs::Twist* MultiDOFJointState::add_twist() {
  ::ros::geometry_msgs::Twist* _add = _internal_add_twist();
  // @@protoc_insertion_point(field_add:ros.sensor_msgs.MultiDOFJointState.twist)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Twist >&
MultiDOFJointState::twist() const {
  // @@protoc_insertion_point(field_list:ros.sensor_msgs.MultiDOFJointState.twist)
  return twist_;
}

// repeated .ros.geometry_msgs.Wrench wrench = 5;
inline int MultiDOFJointState::_internal_wrench_size() const {
  return wrench_.size();
}
inline int MultiDOFJointState::wrench_size() const {
  return _internal_wrench_size();
}
inline ::ros::geometry_msgs::Wrench* MultiDOFJointState::mutable_wrench(int index) {
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.MultiDOFJointState.wrench)
  return wrench_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Wrench >*
MultiDOFJointState::mutable_wrench() {
  // @@protoc_insertion_point(field_mutable_list:ros.sensor_msgs.MultiDOFJointState.wrench)
  return &wrench_;
}
inline const ::ros::geometry_msgs::Wrench& MultiDOFJointState::_internal_wrench(int index) const {
  return wrench_.Get(index);
}
inline const ::ros::geometry_msgs::Wrench& MultiDOFJointState::wrench(int index) const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.MultiDOFJointState.wrench)
  return _internal_wrench(index);
}
inline ::ros::geometry_msgs::Wrench* MultiDOFJointState::_internal_add_wrench() {
  return wrench_.Add();
}
inline ::ros::geometry_msgs::Wrench* MultiDOFJointState::add_wrench() {
  ::ros::geometry_msgs::Wrench* _add = _internal_add_wrench();
  // @@protoc_insertion_point(field_add:ros.sensor_msgs.MultiDOFJointState.wrench)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::geometry_msgs::Wrench >&
MultiDOFJointState::wrench() const {
  // @@protoc_insertion_point(field_list:ros.sensor_msgs.MultiDOFJointState.wrench)
  return wrench_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace sensor_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fMultiDOFJointState_2eproto
