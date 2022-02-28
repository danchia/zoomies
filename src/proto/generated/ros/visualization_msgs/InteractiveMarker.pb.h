// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/visualization_msgs/InteractiveMarker.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto

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
#include "ros/std_msgs/Header.pb.h"
#include "ros/visualization_msgs/InteractiveMarkerControl.pb.h"
#include "ros/visualization_msgs/MenuEntry.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto;
namespace ros {
namespace visualization_msgs {
class InteractiveMarker;
struct InteractiveMarkerDefaultTypeInternal;
extern InteractiveMarkerDefaultTypeInternal _InteractiveMarker_default_instance_;
}  // namespace visualization_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::visualization_msgs::InteractiveMarker* Arena::CreateMaybeMessage<::ros::visualization_msgs::InteractiveMarker>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace visualization_msgs {

// ===================================================================

class InteractiveMarker final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.visualization_msgs.InteractiveMarker) */ {
 public:
  inline InteractiveMarker() : InteractiveMarker(nullptr) {}
  ~InteractiveMarker() override;
  explicit constexpr InteractiveMarker(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  InteractiveMarker(const InteractiveMarker& from);
  InteractiveMarker(InteractiveMarker&& from) noexcept
    : InteractiveMarker() {
    *this = ::std::move(from);
  }

  inline InteractiveMarker& operator=(const InteractiveMarker& from) {
    CopyFrom(from);
    return *this;
  }
  inline InteractiveMarker& operator=(InteractiveMarker&& from) noexcept {
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
  static const InteractiveMarker& default_instance() {
    return *internal_default_instance();
  }
  static inline const InteractiveMarker* internal_default_instance() {
    return reinterpret_cast<const InteractiveMarker*>(
               &_InteractiveMarker_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(InteractiveMarker& a, InteractiveMarker& b) {
    a.Swap(&b);
  }
  inline void Swap(InteractiveMarker* other) {
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
  void UnsafeArenaSwap(InteractiveMarker* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  InteractiveMarker* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<InteractiveMarker>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const InteractiveMarker& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const InteractiveMarker& from);
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
  void InternalSwap(InteractiveMarker* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.visualization_msgs.InteractiveMarker";
  }
  protected:
  explicit InteractiveMarker(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kMenuEntriesFieldNumber = 6,
    kControlsFieldNumber = 7,
    kNameFieldNumber = 3,
    kDescriptionFieldNumber = 4,
    kHeaderFieldNumber = 1,
    kPoseFieldNumber = 2,
    kScaleFieldNumber = 5,
  };
  // repeated .ros.visualization_msgs.MenuEntry menu_entries = 6;
  int menu_entries_size() const;
  private:
  int _internal_menu_entries_size() const;
  public:
  void clear_menu_entries();
  ::ros::visualization_msgs::MenuEntry* mutable_menu_entries(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::MenuEntry >*
      mutable_menu_entries();
  private:
  const ::ros::visualization_msgs::MenuEntry& _internal_menu_entries(int index) const;
  ::ros::visualization_msgs::MenuEntry* _internal_add_menu_entries();
  public:
  const ::ros::visualization_msgs::MenuEntry& menu_entries(int index) const;
  ::ros::visualization_msgs::MenuEntry* add_menu_entries();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::MenuEntry >&
      menu_entries() const;

  // repeated .ros.visualization_msgs.InteractiveMarkerControl controls = 7;
  int controls_size() const;
  private:
  int _internal_controls_size() const;
  public:
  void clear_controls();
  ::ros::visualization_msgs::InteractiveMarkerControl* mutable_controls(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::InteractiveMarkerControl >*
      mutable_controls();
  private:
  const ::ros::visualization_msgs::InteractiveMarkerControl& _internal_controls(int index) const;
  ::ros::visualization_msgs::InteractiveMarkerControl* _internal_add_controls();
  public:
  const ::ros::visualization_msgs::InteractiveMarkerControl& controls(int index) const;
  ::ros::visualization_msgs::InteractiveMarkerControl* add_controls();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::InteractiveMarkerControl >&
      controls() const;

  // string name = 3;
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

  // string description = 4;
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

  // .ros.geometry_msgs.Pose pose = 2;
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

  // float scale = 5;
  void clear_scale();
  float scale() const;
  void set_scale(float value);
  private:
  float _internal_scale() const;
  void _internal_set_scale(float value);
  public:

  // @@protoc_insertion_point(class_scope:ros.visualization_msgs.InteractiveMarker)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::MenuEntry > menu_entries_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::InteractiveMarkerControl > controls_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr name_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr description_;
  ::ros::std_msgs::Header* header_;
  ::ros::geometry_msgs::Pose* pose_;
  float scale_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// InteractiveMarker

// .ros.std_msgs.Header header = 1;
inline bool InteractiveMarker::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool InteractiveMarker::has_header() const {
  return _internal_has_header();
}
inline const ::ros::std_msgs::Header& InteractiveMarker::_internal_header() const {
  const ::ros::std_msgs::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::std_msgs::Header&>(
      ::ros::std_msgs::_Header_default_instance_);
}
inline const ::ros::std_msgs::Header& InteractiveMarker::header() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarker.header)
  return _internal_header();
}
inline void InteractiveMarker::unsafe_arena_set_allocated_header(
    ::ros::std_msgs::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.visualization_msgs.InteractiveMarker.header)
}
inline ::ros::std_msgs::Header* InteractiveMarker::release_header() {
  
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
inline ::ros::std_msgs::Header* InteractiveMarker::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:ros.visualization_msgs.InteractiveMarker.header)
  
  ::ros::std_msgs::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::ros::std_msgs::Header* InteractiveMarker::_internal_mutable_header() {
  
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::std_msgs::Header>(GetArenaForAllocation());
    header_ = p;
  }
  return header_;
}
inline ::ros::std_msgs::Header* InteractiveMarker::mutable_header() {
  ::ros::std_msgs::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.InteractiveMarker.header)
  return _msg;
}
inline void InteractiveMarker::set_allocated_header(::ros::std_msgs::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:ros.visualization_msgs.InteractiveMarker.header)
}

// .ros.geometry_msgs.Pose pose = 2;
inline bool InteractiveMarker::_internal_has_pose() const {
  return this != internal_default_instance() && pose_ != nullptr;
}
inline bool InteractiveMarker::has_pose() const {
  return _internal_has_pose();
}
inline const ::ros::geometry_msgs::Pose& InteractiveMarker::_internal_pose() const {
  const ::ros::geometry_msgs::Pose* p = pose_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::geometry_msgs::Pose&>(
      ::ros::geometry_msgs::_Pose_default_instance_);
}
inline const ::ros::geometry_msgs::Pose& InteractiveMarker::pose() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarker.pose)
  return _internal_pose();
}
inline void InteractiveMarker::unsafe_arena_set_allocated_pose(
    ::ros::geometry_msgs::Pose* pose) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(pose_);
  }
  pose_ = pose;
  if (pose) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.visualization_msgs.InteractiveMarker.pose)
}
inline ::ros::geometry_msgs::Pose* InteractiveMarker::release_pose() {
  
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
inline ::ros::geometry_msgs::Pose* InteractiveMarker::unsafe_arena_release_pose() {
  // @@protoc_insertion_point(field_release:ros.visualization_msgs.InteractiveMarker.pose)
  
  ::ros::geometry_msgs::Pose* temp = pose_;
  pose_ = nullptr;
  return temp;
}
inline ::ros::geometry_msgs::Pose* InteractiveMarker::_internal_mutable_pose() {
  
  if (pose_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::geometry_msgs::Pose>(GetArenaForAllocation());
    pose_ = p;
  }
  return pose_;
}
inline ::ros::geometry_msgs::Pose* InteractiveMarker::mutable_pose() {
  ::ros::geometry_msgs::Pose* _msg = _internal_mutable_pose();
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.InteractiveMarker.pose)
  return _msg;
}
inline void InteractiveMarker::set_allocated_pose(::ros::geometry_msgs::Pose* pose) {
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
  // @@protoc_insertion_point(field_set_allocated:ros.visualization_msgs.InteractiveMarker.pose)
}

// string name = 3;
inline void InteractiveMarker::clear_name() {
  name_.ClearToEmpty();
}
inline const std::string& InteractiveMarker::name() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarker.name)
  return _internal_name();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void InteractiveMarker::set_name(ArgT0&& arg0, ArgT... args) {
 
 name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.InteractiveMarker.name)
}
inline std::string* InteractiveMarker::mutable_name() {
  std::string* _s = _internal_mutable_name();
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.InteractiveMarker.name)
  return _s;
}
inline const std::string& InteractiveMarker::_internal_name() const {
  return name_.Get();
}
inline void InteractiveMarker::_internal_set_name(const std::string& value) {
  
  name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* InteractiveMarker::_internal_mutable_name() {
  
  return name_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* InteractiveMarker::release_name() {
  // @@protoc_insertion_point(field_release:ros.visualization_msgs.InteractiveMarker.name)
  return name_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void InteractiveMarker::set_allocated_name(std::string* name) {
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
  // @@protoc_insertion_point(field_set_allocated:ros.visualization_msgs.InteractiveMarker.name)
}

// string description = 4;
inline void InteractiveMarker::clear_description() {
  description_.ClearToEmpty();
}
inline const std::string& InteractiveMarker::description() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarker.description)
  return _internal_description();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void InteractiveMarker::set_description(ArgT0&& arg0, ArgT... args) {
 
 description_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.InteractiveMarker.description)
}
inline std::string* InteractiveMarker::mutable_description() {
  std::string* _s = _internal_mutable_description();
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.InteractiveMarker.description)
  return _s;
}
inline const std::string& InteractiveMarker::_internal_description() const {
  return description_.Get();
}
inline void InteractiveMarker::_internal_set_description(const std::string& value) {
  
  description_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* InteractiveMarker::_internal_mutable_description() {
  
  return description_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* InteractiveMarker::release_description() {
  // @@protoc_insertion_point(field_release:ros.visualization_msgs.InteractiveMarker.description)
  return description_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void InteractiveMarker::set_allocated_description(std::string* description) {
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
  // @@protoc_insertion_point(field_set_allocated:ros.visualization_msgs.InteractiveMarker.description)
}

// float scale = 5;
inline void InteractiveMarker::clear_scale() {
  scale_ = 0;
}
inline float InteractiveMarker::_internal_scale() const {
  return scale_;
}
inline float InteractiveMarker::scale() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarker.scale)
  return _internal_scale();
}
inline void InteractiveMarker::_internal_set_scale(float value) {
  
  scale_ = value;
}
inline void InteractiveMarker::set_scale(float value) {
  _internal_set_scale(value);
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.InteractiveMarker.scale)
}

// repeated .ros.visualization_msgs.MenuEntry menu_entries = 6;
inline int InteractiveMarker::_internal_menu_entries_size() const {
  return menu_entries_.size();
}
inline int InteractiveMarker::menu_entries_size() const {
  return _internal_menu_entries_size();
}
inline ::ros::visualization_msgs::MenuEntry* InteractiveMarker::mutable_menu_entries(int index) {
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.InteractiveMarker.menu_entries)
  return menu_entries_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::MenuEntry >*
InteractiveMarker::mutable_menu_entries() {
  // @@protoc_insertion_point(field_mutable_list:ros.visualization_msgs.InteractiveMarker.menu_entries)
  return &menu_entries_;
}
inline const ::ros::visualization_msgs::MenuEntry& InteractiveMarker::_internal_menu_entries(int index) const {
  return menu_entries_.Get(index);
}
inline const ::ros::visualization_msgs::MenuEntry& InteractiveMarker::menu_entries(int index) const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarker.menu_entries)
  return _internal_menu_entries(index);
}
inline ::ros::visualization_msgs::MenuEntry* InteractiveMarker::_internal_add_menu_entries() {
  return menu_entries_.Add();
}
inline ::ros::visualization_msgs::MenuEntry* InteractiveMarker::add_menu_entries() {
  ::ros::visualization_msgs::MenuEntry* _add = _internal_add_menu_entries();
  // @@protoc_insertion_point(field_add:ros.visualization_msgs.InteractiveMarker.menu_entries)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::MenuEntry >&
InteractiveMarker::menu_entries() const {
  // @@protoc_insertion_point(field_list:ros.visualization_msgs.InteractiveMarker.menu_entries)
  return menu_entries_;
}

// repeated .ros.visualization_msgs.InteractiveMarkerControl controls = 7;
inline int InteractiveMarker::_internal_controls_size() const {
  return controls_.size();
}
inline int InteractiveMarker::controls_size() const {
  return _internal_controls_size();
}
inline ::ros::visualization_msgs::InteractiveMarkerControl* InteractiveMarker::mutable_controls(int index) {
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.InteractiveMarker.controls)
  return controls_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::InteractiveMarkerControl >*
InteractiveMarker::mutable_controls() {
  // @@protoc_insertion_point(field_mutable_list:ros.visualization_msgs.InteractiveMarker.controls)
  return &controls_;
}
inline const ::ros::visualization_msgs::InteractiveMarkerControl& InteractiveMarker::_internal_controls(int index) const {
  return controls_.Get(index);
}
inline const ::ros::visualization_msgs::InteractiveMarkerControl& InteractiveMarker::controls(int index) const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.InteractiveMarker.controls)
  return _internal_controls(index);
}
inline ::ros::visualization_msgs::InteractiveMarkerControl* InteractiveMarker::_internal_add_controls() {
  return controls_.Add();
}
inline ::ros::visualization_msgs::InteractiveMarkerControl* InteractiveMarker::add_controls() {
  ::ros::visualization_msgs::InteractiveMarkerControl* _add = _internal_add_controls();
  // @@protoc_insertion_point(field_add:ros.visualization_msgs.InteractiveMarker.controls)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::ros::visualization_msgs::InteractiveMarkerControl >&
InteractiveMarker::controls() const {
  // @@protoc_insertion_point(field_list:ros.visualization_msgs.InteractiveMarker.controls)
  return controls_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace visualization_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fvisualization_5fmsgs_2fInteractiveMarker_2eproto