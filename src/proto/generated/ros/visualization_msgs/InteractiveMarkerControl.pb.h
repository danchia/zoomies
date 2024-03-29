// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/visualization_msgs/InteractiveMarkerControl.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto

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
#include "ros/geometry_msgs/Quaternion.pb.h"
#include "ros/visualization_msgs/Marker.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto;
namespace ros {
namespace visualization_msgs {
class InteractiveMarkerControl;
struct InteractiveMarkerControlDefaultTypeInternal;
extern InteractiveMarkerControlDefaultTypeInternal _InteractiveMarkerControl_default_instance_;
}  // namespace visualization_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::visualization_msgs::InteractiveMarkerControl* Arena::CreateMaybeMessage<::ros::visualization_msgs::InteractiveMarkerControl>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace visualization_msgs {

// ===================================================================

class InteractiveMarkerControl final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.visualization_msgs.InteractiveMarkerControl) */ {
 public:
  inline InteractiveMarkerControl() : InteractiveMarkerControl(nullptr) {}
  ~InteractiveMarkerControl() override;
  explicit constexpr InteractiveMarkerControl(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  InteractiveMarkerControl(const InteractiveMarkerControl& from);
  InteractiveMarkerControl(InteractiveMarkerControl&& from) noexcept
    : InteractiveMarkerControl() {
    *this = ::std::move(from);
  }

  inline InteractiveMarkerControl& operator=(const InteractiveMarkerControl& from) {
    CopyFrom(from);
    return *this;
  }
  inline InteractiveMarkerControl& operator=(InteractiveMarkerControl&& from) noexcept {
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
  static const InteractiveMarkerControl& default_instance() {
    return *internal_default_instance();
  }
  static inline const InteractiveMarkerControl* internal_default_instance() {
    return reinterpret_cast<const InteractiveMarkerControl*>(
               &_InteractiveMarkerControl_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(InteractiveMarkerControl& a, InteractiveMarkerControl& b) {
    a.Swap(&b);
  }
  inline void Swap(InteractiveMarkerControl* other) {
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
  void UnsafeArenaSwap(InteractiveMarkerControl* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  InteractiveMarkerControl* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<InteractiveMarkerControl>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const InteractiveMarkerControl& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const InteractiveMarkerControl& from);
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
  void InternalSwap(InteractiveMarkerControl* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.visualization_msgs.InteractiveMarkerControl";
  }
  protected:
  explicit InteractiveMarkerControl(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kMarkersFieldNumber = 6,
    kNameFieldNumber = 1,
    kDescriptionFieldNumber = 8,
    kOrientationFieldNumber = 2,
    kOrientationModeFieldNumber = 3,
    kInteractionModeFieldNumber = 4,
    kAlwaysVisibleFieldNumber = 5,
    kIndependentMarkerOrientationFieldNumber = 7,
  };
  // repeated .ros.visualization_msgs.Marker markers = 6;
  int markers_size() const;
  private:
  int _internal_markers_size() const;
  public:
  void clear_markers();
  ::ros::visualization_msgs::Marker* mutable_markers(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::Marker >*
      mutable_markers();
  private:
  const ::ros::visualization_msgs::Marker& _internal_markers(int index) const;
  ::ros::visualization_msgs::Marker* _internal_add_markers();
  public:
  const ::ros::visualization_msgs::Marker& markers(int index) const;
  ::ros::visualization_msgs::Marker* add_markers();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::Marker >&
      markers() const;

  // string name = 1;
  void clear_name();
  const std::string& name() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_name(ArgT0&& arg0, ArgT... args);
  std::string* mutable_name();
  PROTOBUF_NODISCARD std::string* release_name();
  void set_allocated_name(std::string* name);
  private:
  const std::string& _internal_name() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_name(const std::string& value);
  std::string* _internal_mutable_name();
  public:

  // string description = 8;
  void clear_description();
  const std::string& description() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_description(ArgT0&& arg0, ArgT... args);
  std::string* mutable_description();
  PROTOBUF_NODISCARD std::string* release_description();
  void set_allocated_description(std::string* description);
  private:
  const std::string& _internal_description() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_description(const std::string& value);
  std::string* _internal_mutable_description();
  public:

  // .ros.geometry_msgs.Quaternion orientation = 2;
  bool has_orientation() const;
  private:
  bool _internal_has_orientation() const;
  public:
  void clear_orientation();
  const ::ros::geometry_msgs::Quaternion& orientation() const;
  PROTOBUF_NODISCARD ::ros::geometry_msgs::Quaternion* release_orientation();
  ::ros::geometry_msgs::Quaternion* mutable_orientation();
  void set_allocated_orientation(::ros::geometry_msgs::Quaternion* orientation);
  private:
  const ::ros::geometry_msgs::Quaternion& _internal_orientation() const;
  ::ros::geometry_msgs::Quaternion* _internal_mutable_orientation();
  public:
  void unsafe_arena_set_allocated_orientation(
      ::ros::geometry_msgs::Quaternion* orientation);
  ::ros::geometry_msgs::Quaternion* unsafe_arena_release_orientation();

  // int32 orientation_mode = 3;
  void clear_orientation_mode();
  int32_t orientation_mode() const;
  void set_orientation_mode(int32_t value);
  private:
  int32_t _internal_orientation_mode() const;
  void _internal_set_orientation_mode(int32_t value);
  public:

  // int32 interaction_mode = 4;
  void clear_interaction_mode();
  int32_t interaction_mode() const;
  void set_interaction_mode(int32_t value);
  private:
  int32_t _internal_interaction_mode() const;
  void _internal_set_interaction_mode(int32_t value);
  public:

  // bool always_visible = 5;
  void clear_always_visible();
  bool always_visible() const;
  void set_always_visible(bool value);
  private:
  bool _internal_always_visible() const;
  void _internal_set_always_visible(bool value);
  public:

  // bool independent_marker_orientation = 7;
  void clear_independent_marker_orientation();
  bool independent_marker_orientation() const;
  void set_independent_marker_orientation(bool value);
  private:
  bool _internal_independent_marker_orientation() const;
  void _internal_set_independent_marker_orientation(bool value);
  public:

  // @@protoc_insertion_point(class_scope:ros.visualization_msgs.InteractiveMarkerControl)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::Marker > markers_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr name_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr description_;
  ::ros::geometry_msgs::Quaternion* orientation_;
  int32_t orientation_mode_;
  int32_t interaction_mode_;
  bool always_visible_;
  bool independent_marker_orientation_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// InteractiveMarkerControl

// string name = 1;
inline void InteractiveMarkerControl::clear_name() {
  name_.ClearToEmpty();
}
inline const std::string& InteractiveMarkerControl::name() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarkerControl.name)
  return _internal_name();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void InteractiveMarkerControl::set_name(ArgT0&& arg0, ArgT... args) {
 
 name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.InteractiveMarkerControl.name)
}
inline std::string* InteractiveMarkerControl::mutable_name() {
  std::string* _s = _internal_mutable_name();
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.InteractiveMarkerControl.name)
  return _s;
}
inline const std::string& InteractiveMarkerControl::_internal_name() const {
  return name_.Get();
}
inline void InteractiveMarkerControl::_internal_set_name(const std::string& value) {
  
  name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* InteractiveMarkerControl::_internal_mutable_name() {
  
  return name_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* InteractiveMarkerControl::release_name() {
  // @@protoc_insertion_point(field_release:ros.visualization_msgs.InteractiveMarkerControl.name)
  return name_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void InteractiveMarkerControl::set_allocated_name(std::string* name) {
  if (name != nullptr) {
    
  } else {
    
  }
  name_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), name,
      GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (name_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    name_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:ros.visualization_msgs.InteractiveMarkerControl.name)
}

// .ros.geometry_msgs.Quaternion orientation = 2;
inline bool InteractiveMarkerControl::_internal_has_orientation() const {
  return this != internal_default_instance() && orientation_ != nullptr;
}
inline bool InteractiveMarkerControl::has_orientation() const {
  return _internal_has_orientation();
}
inline const ::ros::geometry_msgs::Quaternion& InteractiveMarkerControl::_internal_orientation() const {
  const ::ros::geometry_msgs::Quaternion* p = orientation_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::geometry_msgs::Quaternion&>(
      ::ros::geometry_msgs::_Quaternion_default_instance_);
}
inline const ::ros::geometry_msgs::Quaternion& InteractiveMarkerControl::orientation() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarkerControl.orientation)
  return _internal_orientation();
}
inline void InteractiveMarkerControl::unsafe_arena_set_allocated_orientation(
    ::ros::geometry_msgs::Quaternion* orientation) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(orientation_);
  }
  orientation_ = orientation;
  if (orientation) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.visualization_msgs.InteractiveMarkerControl.orientation)
}
inline ::ros::geometry_msgs::Quaternion* InteractiveMarkerControl::release_orientation() {
  
  ::ros::geometry_msgs::Quaternion* temp = orientation_;
  orientation_ = nullptr;
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
inline ::ros::geometry_msgs::Quaternion* InteractiveMarkerControl::unsafe_arena_release_orientation() {
  // @@protoc_insertion_point(field_release:ros.visualization_msgs.InteractiveMarkerControl.orientation)
  
  ::ros::geometry_msgs::Quaternion* temp = orientation_;
  orientation_ = nullptr;
  return temp;
}
inline ::ros::geometry_msgs::Quaternion* InteractiveMarkerControl::_internal_mutable_orientation() {
  
  if (orientation_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::geometry_msgs::Quaternion>(GetArenaForAllocation());
    orientation_ = p;
  }
  return orientation_;
}
inline ::ros::geometry_msgs::Quaternion* InteractiveMarkerControl::mutable_orientation() {
  ::ros::geometry_msgs::Quaternion* _msg = _internal_mutable_orientation();
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.InteractiveMarkerControl.orientation)
  return _msg;
}
inline void InteractiveMarkerControl::set_allocated_orientation(::ros::geometry_msgs::Quaternion* orientation) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(orientation_);
  }
  if (orientation) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(orientation));
    if (message_arena != submessage_arena) {
      orientation = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, orientation, submessage_arena);
    }
    
  } else {
    
  }
  orientation_ = orientation;
  // @@protoc_insertion_point(field_set_allocated:ros.visualization_msgs.InteractiveMarkerControl.orientation)
}

// int32 orientation_mode = 3;
inline void InteractiveMarkerControl::clear_orientation_mode() {
  orientation_mode_ = 0;
}
inline int32_t InteractiveMarkerControl::_internal_orientation_mode() const {
  return orientation_mode_;
}
inline int32_t InteractiveMarkerControl::orientation_mode() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarkerControl.orientation_mode)
  return _internal_orientation_mode();
}
inline void InteractiveMarkerControl::_internal_set_orientation_mode(int32_t value) {
  
  orientation_mode_ = value;
}
inline void InteractiveMarkerControl::set_orientation_mode(int32_t value) {
  _internal_set_orientation_mode(value);
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.InteractiveMarkerControl.orientation_mode)
}

// int32 interaction_mode = 4;
inline void InteractiveMarkerControl::clear_interaction_mode() {
  interaction_mode_ = 0;
}
inline int32_t InteractiveMarkerControl::_internal_interaction_mode() const {
  return interaction_mode_;
}
inline int32_t InteractiveMarkerControl::interaction_mode() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarkerControl.interaction_mode)
  return _internal_interaction_mode();
}
inline void InteractiveMarkerControl::_internal_set_interaction_mode(int32_t value) {
  
  interaction_mode_ = value;
}
inline void InteractiveMarkerControl::set_interaction_mode(int32_t value) {
  _internal_set_interaction_mode(value);
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.InteractiveMarkerControl.interaction_mode)
}

// bool always_visible = 5;
inline void InteractiveMarkerControl::clear_always_visible() {
  always_visible_ = false;
}
inline bool InteractiveMarkerControl::_internal_always_visible() const {
  return always_visible_;
}
inline bool InteractiveMarkerControl::always_visible() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarkerControl.always_visible)
  return _internal_always_visible();
}
inline void InteractiveMarkerControl::_internal_set_always_visible(bool value) {
  
  always_visible_ = value;
}
inline void InteractiveMarkerControl::set_always_visible(bool value) {
  _internal_set_always_visible(value);
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.InteractiveMarkerControl.always_visible)
}

// repeated .ros.visualization_msgs.Marker markers = 6;
inline int InteractiveMarkerControl::_internal_markers_size() const {
  return markers_.size();
}
inline int InteractiveMarkerControl::markers_size() const {
  return _internal_markers_size();
}
inline ::ros::visualization_msgs::Marker* InteractiveMarkerControl::mutable_markers(int index) {
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.InteractiveMarkerControl.markers)
  return markers_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::Marker >*
InteractiveMarkerControl::mutable_markers() {
  // @@protoc_insertion_point(field_mutable_list:ros.visualization_msgs.InteractiveMarkerControl.markers)
  return &markers_;
}
inline const ::ros::visualization_msgs::Marker& InteractiveMarkerControl::_internal_markers(int index) const {
  return markers_.Get(index);
}
inline const ::ros::visualization_msgs::Marker& InteractiveMarkerControl::markers(int index) const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarkerControl.markers)
  return _internal_markers(index);
}
inline ::ros::visualization_msgs::Marker* InteractiveMarkerControl::_internal_add_markers() {
  return markers_.Add();
}
inline ::ros::visualization_msgs::Marker* InteractiveMarkerControl::add_markers() {
  ::ros::visualization_msgs::Marker* _add = _internal_add_markers();
  // @@protoc_insertion_point(field_add:ros.visualization_msgs.InteractiveMarkerControl.markers)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::Marker >&
InteractiveMarkerControl::markers() const {
  // @@protoc_insertion_point(field_list:ros.visualization_msgs.InteractiveMarkerControl.markers)
  return markers_;
}

// bool independent_marker_orientation = 7;
inline void InteractiveMarkerControl::clear_independent_marker_orientation() {
  independent_marker_orientation_ = false;
}
inline bool InteractiveMarkerControl::_internal_independent_marker_orientation() const {
  return independent_marker_orientation_;
}
inline bool InteractiveMarkerControl::independent_marker_orientation() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarkerControl.independent_marker_orientation)
  return _internal_independent_marker_orientation();
}
inline void InteractiveMarkerControl::_internal_set_independent_marker_orientation(bool value) {
  
  independent_marker_orientation_ = value;
}
inline void InteractiveMarkerControl::set_independent_marker_orientation(bool value) {
  _internal_set_independent_marker_orientation(value);
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.InteractiveMarkerControl.independent_marker_orientation)
}

// string description = 8;
inline void InteractiveMarkerControl::clear_description() {
  description_.ClearToEmpty();
}
inline const std::string& InteractiveMarkerControl::description() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarkerControl.description)
  return _internal_description();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void InteractiveMarkerControl::set_description(ArgT0&& arg0, ArgT... args) {
 
 description_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.InteractiveMarkerControl.description)
}
inline std::string* InteractiveMarkerControl::mutable_description() {
  std::string* _s = _internal_mutable_description();
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.InteractiveMarkerControl.description)
  return _s;
}
inline const std::string& InteractiveMarkerControl::_internal_description() const {
  return description_.Get();
}
inline void InteractiveMarkerControl::_internal_set_description(const std::string& value) {
  
  description_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* InteractiveMarkerControl::_internal_mutable_description() {
  
  return description_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* InteractiveMarkerControl::release_description() {
  // @@protoc_insertion_point(field_release:ros.visualization_msgs.InteractiveMarkerControl.description)
  return description_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void InteractiveMarkerControl::set_allocated_description(std::string* description) {
  if (description != nullptr) {
    
  } else {
    
  }
  description_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), description,
      GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (description_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    description_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:ros.visualization_msgs.InteractiveMarkerControl.description)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace visualization_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fvisualization_5fmsgs_2fInteractiveMarkerControl_2eproto
