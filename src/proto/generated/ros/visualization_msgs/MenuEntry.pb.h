// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/visualization_msgs/MenuEntry.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fvisualization_5fmsgs_2fMenuEntry_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fvisualization_5fmsgs_2fMenuEntry_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_ros_2fvisualization_5fmsgs_2fMenuEntry_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fvisualization_5fmsgs_2fMenuEntry_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fvisualization_5fmsgs_2fMenuEntry_2eproto;
namespace ros {
namespace visualization_msgs {
class MenuEntry;
struct MenuEntryDefaultTypeInternal;
extern MenuEntryDefaultTypeInternal _MenuEntry_default_instance_;
}  // namespace visualization_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::visualization_msgs::MenuEntry* Arena::CreateMaybeMessage<::ros::visualization_msgs::MenuEntry>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace visualization_msgs {

// ===================================================================

class MenuEntry final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.visualization_msgs.MenuEntry) */ {
 public:
  inline MenuEntry() : MenuEntry(nullptr) {}
  ~MenuEntry() override;
  explicit constexpr MenuEntry(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  MenuEntry(const MenuEntry& from);
  MenuEntry(MenuEntry&& from) noexcept
    : MenuEntry() {
    *this = ::std::move(from);
  }

  inline MenuEntry& operator=(const MenuEntry& from) {
    CopyFrom(from);
    return *this;
  }
  inline MenuEntry& operator=(MenuEntry&& from) noexcept {
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
  static const MenuEntry& default_instance() {
    return *internal_default_instance();
  }
  static inline const MenuEntry* internal_default_instance() {
    return reinterpret_cast<const MenuEntry*>(
               &_MenuEntry_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MenuEntry& a, MenuEntry& b) {
    a.Swap(&b);
  }
  inline void Swap(MenuEntry* other) {
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
  void UnsafeArenaSwap(MenuEntry* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  MenuEntry* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<MenuEntry>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const MenuEntry& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const MenuEntry& from);
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
  void InternalSwap(MenuEntry* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.visualization_msgs.MenuEntry";
  }
  protected:
  explicit MenuEntry(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kTitleFieldNumber = 3,
    kCommandFieldNumber = 4,
    kIdFieldNumber = 1,
    kParentIdFieldNumber = 2,
    kCommandTypeFieldNumber = 5,
  };
  // string title = 3;
  void clear_title();
  const std::string& title() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_title(ArgT0&& arg0, ArgT... args);
  std::string* mutable_title();
  PROTOBUF_NODISCARD std::string* release_title();
  void set_allocated_title(std::string* title);
  private:
  const std::string& _internal_title() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_title(const std::string& value);
  std::string* _internal_mutable_title();
  public:

  // string command = 4;
  void clear_command();
  const std::string& command() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_command(ArgT0&& arg0, ArgT... args);
  std::string* mutable_command();
  PROTOBUF_NODISCARD std::string* release_command();
  void set_allocated_command(std::string* command);
  private:
  const std::string& _internal_command() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_command(const std::string& value);
  std::string* _internal_mutable_command();
  public:

  // uint32 id = 1;
  void clear_id();
  uint32_t id() const;
  void set_id(uint32_t value);
  private:
  uint32_t _internal_id() const;
  void _internal_set_id(uint32_t value);
  public:

  // uint32 parent_id = 2;
  void clear_parent_id();
  uint32_t parent_id() const;
  void set_parent_id(uint32_t value);
  private:
  uint32_t _internal_parent_id() const;
  void _internal_set_parent_id(uint32_t value);
  public:

  // int32 command_type = 5;
  void clear_command_type();
  int32_t command_type() const;
  void set_command_type(int32_t value);
  private:
  int32_t _internal_command_type() const;
  void _internal_set_command_type(int32_t value);
  public:

  // @@protoc_insertion_point(class_scope:ros.visualization_msgs.MenuEntry)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr title_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr command_;
  uint32_t id_;
  uint32_t parent_id_;
  int32_t command_type_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fvisualization_5fmsgs_2fMenuEntry_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MenuEntry

// uint32 id = 1;
inline void MenuEntry::clear_id() {
  id_ = 0u;
}
inline uint32_t MenuEntry::_internal_id() const {
  return id_;
}
inline uint32_t MenuEntry::id() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.MenuEntry.id)
  return _internal_id();
}
inline void MenuEntry::_internal_set_id(uint32_t value) {
  
  id_ = value;
}
inline void MenuEntry::set_id(uint32_t value) {
  _internal_set_id(value);
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.MenuEntry.id)
}

// uint32 parent_id = 2;
inline void MenuEntry::clear_parent_id() {
  parent_id_ = 0u;
}
inline uint32_t MenuEntry::_internal_parent_id() const {
  return parent_id_;
}
inline uint32_t MenuEntry::parent_id() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.MenuEntry.parent_id)
  return _internal_parent_id();
}
inline void MenuEntry::_internal_set_parent_id(uint32_t value) {
  
  parent_id_ = value;
}
inline void MenuEntry::set_parent_id(uint32_t value) {
  _internal_set_parent_id(value);
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.MenuEntry.parent_id)
}

// string title = 3;
inline void MenuEntry::clear_title() {
  title_.ClearToEmpty();
}
inline const std::string& MenuEntry::title() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.MenuEntry.title)
  return _internal_title();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void MenuEntry::set_title(ArgT0&& arg0, ArgT... args) {
 
 title_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.MenuEntry.title)
}
inline std::string* MenuEntry::mutable_title() {
  std::string* _s = _internal_mutable_title();
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.MenuEntry.title)
  return _s;
}
inline const std::string& MenuEntry::_internal_title() const {
  return title_.Get();
}
inline void MenuEntry::_internal_set_title(const std::string& value) {
  
  title_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* MenuEntry::_internal_mutable_title() {
  
  return title_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* MenuEntry::release_title() {
  // @@protoc_insertion_point(field_release:ros.visualization_msgs.MenuEntry.title)
  return title_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void MenuEntry::set_allocated_title(std::string* title) {
  if (title != nullptr) {
    
  } else {
    
  }
  title_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), title,
      GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (title_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    title_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:ros.visualization_msgs.MenuEntry.title)
}

// string command = 4;
inline void MenuEntry::clear_command() {
  command_.ClearToEmpty();
}
inline const std::string& MenuEntry::command() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.MenuEntry.command)
  return _internal_command();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void MenuEntry::set_command(ArgT0&& arg0, ArgT... args) {
 
 command_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.MenuEntry.command)
}
inline std::string* MenuEntry::mutable_command() {
  std::string* _s = _internal_mutable_command();
  // @@protoc_insertion_point(field_mutable:ros.visualization_msgs.MenuEntry.command)
  return _s;
}
inline const std::string& MenuEntry::_internal_command() const {
  return command_.Get();
}
inline void MenuEntry::_internal_set_command(const std::string& value) {
  
  command_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* MenuEntry::_internal_mutable_command() {
  
  return command_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* MenuEntry::release_command() {
  // @@protoc_insertion_point(field_release:ros.visualization_msgs.MenuEntry.command)
  return command_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void MenuEntry::set_allocated_command(std::string* command) {
  if (command != nullptr) {
    
  } else {
    
  }
  command_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), command,
      GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (command_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    command_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:ros.visualization_msgs.MenuEntry.command)
}

// int32 command_type = 5;
inline void MenuEntry::clear_command_type() {
  command_type_ = 0;
}
inline int32_t MenuEntry::_internal_command_type() const {
  return command_type_;
}
inline int32_t MenuEntry::command_type() const {
  // @@protoc_insertion_point(field_get:ros.visualization_msgs.MenuEntry.command_type)
  return _internal_command_type();
}
inline void MenuEntry::_internal_set_command_type(int32_t value) {
  
  command_type_ = value;
}
inline void MenuEntry::set_command_type(int32_t value) {
  _internal_set_command_type(value);
  // @@protoc_insertion_point(field_set:ros.visualization_msgs.MenuEntry.command_type)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace visualization_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fvisualization_5fmsgs_2fMenuEntry_2eproto
