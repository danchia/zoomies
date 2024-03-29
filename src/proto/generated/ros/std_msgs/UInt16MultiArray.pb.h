// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/std_msgs/UInt16MultiArray.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fstd_5fmsgs_2fUInt16MultiArray_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fstd_5fmsgs_2fUInt16MultiArray_2eproto

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
#include "ros/std_msgs/MultiArrayLayout.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fstd_5fmsgs_2fUInt16MultiArray_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fstd_5fmsgs_2fUInt16MultiArray_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fstd_5fmsgs_2fUInt16MultiArray_2eproto;
namespace ros {
namespace std_msgs {
class UInt16MultiArray;
struct UInt16MultiArrayDefaultTypeInternal;
extern UInt16MultiArrayDefaultTypeInternal _UInt16MultiArray_default_instance_;
}  // namespace std_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::std_msgs::UInt16MultiArray* Arena::CreateMaybeMessage<::ros::std_msgs::UInt16MultiArray>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace std_msgs {

// ===================================================================

class UInt16MultiArray final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.std_msgs.UInt16MultiArray) */ {
 public:
  inline UInt16MultiArray() : UInt16MultiArray(nullptr) {}
  ~UInt16MultiArray() override;
  explicit constexpr UInt16MultiArray(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  UInt16MultiArray(const UInt16MultiArray& from);
  UInt16MultiArray(UInt16MultiArray&& from) noexcept
    : UInt16MultiArray() {
    *this = ::std::move(from);
  }

  inline UInt16MultiArray& operator=(const UInt16MultiArray& from) {
    CopyFrom(from);
    return *this;
  }
  inline UInt16MultiArray& operator=(UInt16MultiArray&& from) noexcept {
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
  static const UInt16MultiArray& default_instance() {
    return *internal_default_instance();
  }
  static inline const UInt16MultiArray* internal_default_instance() {
    return reinterpret_cast<const UInt16MultiArray*>(
               &_UInt16MultiArray_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(UInt16MultiArray& a, UInt16MultiArray& b) {
    a.Swap(&b);
  }
  inline void Swap(UInt16MultiArray* other) {
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
  void UnsafeArenaSwap(UInt16MultiArray* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  UInt16MultiArray* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<UInt16MultiArray>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const UInt16MultiArray& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const UInt16MultiArray& from);
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
  void InternalSwap(UInt16MultiArray* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.std_msgs.UInt16MultiArray";
  }
  protected:
  explicit UInt16MultiArray(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kDataFieldNumber = 2,
    kLayoutFieldNumber = 1,
  };
  // repeated int32 data = 2;
  int data_size() const;
  private:
  int _internal_data_size() const;
  public:
  void clear_data();
  private:
  int32_t _internal_data(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< int32_t >&
      _internal_data() const;
  void _internal_add_data(int32_t value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< int32_t >*
      _internal_mutable_data();
  public:
  int32_t data(int index) const;
  void set_data(int index, int32_t value);
  void add_data(int32_t value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< int32_t >&
      data() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< int32_t >*
      mutable_data();

  // .ros.std_msgs.MultiArrayLayout layout = 1;
  bool has_layout() const;
  private:
  bool _internal_has_layout() const;
  public:
  void clear_layout();
  const ::ros::std_msgs::MultiArrayLayout& layout() const;
  PROTOBUF_NODISCARD ::ros::std_msgs::MultiArrayLayout* release_layout();
  ::ros::std_msgs::MultiArrayLayout* mutable_layout();
  void set_allocated_layout(::ros::std_msgs::MultiArrayLayout* layout);
  private:
  const ::ros::std_msgs::MultiArrayLayout& _internal_layout() const;
  ::ros::std_msgs::MultiArrayLayout* _internal_mutable_layout();
  public:
  void unsafe_arena_set_allocated_layout(
      ::ros::std_msgs::MultiArrayLayout* layout);
  ::ros::std_msgs::MultiArrayLayout* unsafe_arena_release_layout();

  // @@protoc_insertion_point(class_scope:ros.std_msgs.UInt16MultiArray)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< int32_t > data_;
  mutable std::atomic<int> _data_cached_byte_size_;
  ::ros::std_msgs::MultiArrayLayout* layout_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fstd_5fmsgs_2fUInt16MultiArray_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// UInt16MultiArray

// .ros.std_msgs.MultiArrayLayout layout = 1;
inline bool UInt16MultiArray::_internal_has_layout() const {
  return this != internal_default_instance() && layout_ != nullptr;
}
inline bool UInt16MultiArray::has_layout() const {
  return _internal_has_layout();
}
inline const ::ros::std_msgs::MultiArrayLayout& UInt16MultiArray::_internal_layout() const {
  const ::ros::std_msgs::MultiArrayLayout* p = layout_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::std_msgs::MultiArrayLayout&>(
      ::ros::std_msgs::_MultiArrayLayout_default_instance_);
}
inline const ::ros::std_msgs::MultiArrayLayout& UInt16MultiArray::layout() const {
  // @@protoc_insertion_point(field_get:ros.std_msgs.UInt16MultiArray.layout)
  return _internal_layout();
}
inline void UInt16MultiArray::unsafe_arena_set_allocated_layout(
    ::ros::std_msgs::MultiArrayLayout* layout) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(layout_);
  }
  layout_ = layout;
  if (layout) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.std_msgs.UInt16MultiArray.layout)
}
inline ::ros::std_msgs::MultiArrayLayout* UInt16MultiArray::release_layout() {
  
  ::ros::std_msgs::MultiArrayLayout* temp = layout_;
  layout_ = nullptr;
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
inline ::ros::std_msgs::MultiArrayLayout* UInt16MultiArray::unsafe_arena_release_layout() {
  // @@protoc_insertion_point(field_release:ros.std_msgs.UInt16MultiArray.layout)
  
  ::ros::std_msgs::MultiArrayLayout* temp = layout_;
  layout_ = nullptr;
  return temp;
}
inline ::ros::std_msgs::MultiArrayLayout* UInt16MultiArray::_internal_mutable_layout() {
  
  if (layout_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::std_msgs::MultiArrayLayout>(GetArenaForAllocation());
    layout_ = p;
  }
  return layout_;
}
inline ::ros::std_msgs::MultiArrayLayout* UInt16MultiArray::mutable_layout() {
  ::ros::std_msgs::MultiArrayLayout* _msg = _internal_mutable_layout();
  // @@protoc_insertion_point(field_mutable:ros.std_msgs.UInt16MultiArray.layout)
  return _msg;
}
inline void UInt16MultiArray::set_allocated_layout(::ros::std_msgs::MultiArrayLayout* layout) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(layout_);
  }
  if (layout) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(layout));
    if (message_arena != submessage_arena) {
      layout = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, layout, submessage_arena);
    }
    
  } else {
    
  }
  layout_ = layout;
  // @@protoc_insertion_point(field_set_allocated:ros.std_msgs.UInt16MultiArray.layout)
}

// repeated int32 data = 2;
inline int UInt16MultiArray::_internal_data_size() const {
  return data_.size();
}
inline int UInt16MultiArray::data_size() const {
  return _internal_data_size();
}
inline void UInt16MultiArray::clear_data() {
  data_.Clear();
}
inline int32_t UInt16MultiArray::_internal_data(int index) const {
  return data_.Get(index);
}
inline int32_t UInt16MultiArray::data(int index) const {
  // @@protoc_insertion_point(field_get:ros.std_msgs.UInt16MultiArray.data)
  return _internal_data(index);
}
inline void UInt16MultiArray::set_data(int index, int32_t value) {
  data_.Set(index, value);
  // @@protoc_insertion_point(field_set:ros.std_msgs.UInt16MultiArray.data)
}
inline void UInt16MultiArray::_internal_add_data(int32_t value) {
  data_.Add(value);
}
inline void UInt16MultiArray::add_data(int32_t value) {
  _internal_add_data(value);
  // @@protoc_insertion_point(field_add:ros.std_msgs.UInt16MultiArray.data)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< int32_t >&
UInt16MultiArray::_internal_data() const {
  return data_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< int32_t >&
UInt16MultiArray::data() const {
  // @@protoc_insertion_point(field_list:ros.std_msgs.UInt16MultiArray.data)
  return _internal_data();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< int32_t >*
UInt16MultiArray::_internal_mutable_data() {
  return &data_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< int32_t >*
UInt16MultiArray::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:ros.std_msgs.UInt16MultiArray.data)
  return _internal_mutable_data();
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace std_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fstd_5fmsgs_2fUInt16MultiArray_2eproto
