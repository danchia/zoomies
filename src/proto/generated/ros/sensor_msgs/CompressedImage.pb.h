// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/sensor_msgs/CompressedImage.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fCompressedImage_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fCompressedImage_2eproto

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
#include "ros/std_msgs/Header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fsensor_5fmsgs_2fCompressedImage_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fsensor_5fmsgs_2fCompressedImage_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fsensor_5fmsgs_2fCompressedImage_2eproto;
namespace ros {
namespace sensor_msgs {
class CompressedImage;
struct CompressedImageDefaultTypeInternal;
extern CompressedImageDefaultTypeInternal _CompressedImage_default_instance_;
}  // namespace sensor_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::sensor_msgs::CompressedImage* Arena::CreateMaybeMessage<::ros::sensor_msgs::CompressedImage>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace sensor_msgs {

// ===================================================================

class CompressedImage final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.sensor_msgs.CompressedImage) */ {
 public:
  inline CompressedImage() : CompressedImage(nullptr) {}
  ~CompressedImage() override;
  explicit constexpr CompressedImage(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  CompressedImage(const CompressedImage& from);
  CompressedImage(CompressedImage&& from) noexcept
    : CompressedImage() {
    *this = ::std::move(from);
  }

  inline CompressedImage& operator=(const CompressedImage& from) {
    CopyFrom(from);
    return *this;
  }
  inline CompressedImage& operator=(CompressedImage&& from) noexcept {
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
  static const CompressedImage& default_instance() {
    return *internal_default_instance();
  }
  static inline const CompressedImage* internal_default_instance() {
    return reinterpret_cast<const CompressedImage*>(
               &_CompressedImage_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(CompressedImage& a, CompressedImage& b) {
    a.Swap(&b);
  }
  inline void Swap(CompressedImage* other) {
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
  void UnsafeArenaSwap(CompressedImage* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  CompressedImage* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<CompressedImage>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const CompressedImage& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const CompressedImage& from);
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
  void InternalSwap(CompressedImage* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.sensor_msgs.CompressedImage";
  }
  protected:
  explicit CompressedImage(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kFormatFieldNumber = 2,
    kDataFieldNumber = 3,
    kHeaderFieldNumber = 1,
  };
  // string format = 2;
  void clear_format();
  const std::string& format() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_format(ArgT0&& arg0, ArgT... args);
  std::string* mutable_format();
  PROTOBUF_NODISCARD std::string* release_format();
  void set_allocated_format(std::string* format);
  private:
  const std::string& _internal_format() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_format(const std::string& value);
  std::string* _internal_mutable_format();
  public:

  // bytes data = 3;
  void clear_data();
  const std::string& data() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_data(ArgT0&& arg0, ArgT... args);
  std::string* mutable_data();
  PROTOBUF_NODISCARD std::string* release_data();
  void set_allocated_data(std::string* data);
  private:
  const std::string& _internal_data() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_data(const std::string& value);
  std::string* _internal_mutable_data();
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

  // @@protoc_insertion_point(class_scope:ros.sensor_msgs.CompressedImage)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr format_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr data_;
  ::ros::std_msgs::Header* header_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fsensor_5fmsgs_2fCompressedImage_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// CompressedImage

// .ros.std_msgs.Header header = 1;
inline bool CompressedImage::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool CompressedImage::has_header() const {
  return _internal_has_header();
}
inline const ::ros::std_msgs::Header& CompressedImage::_internal_header() const {
  const ::ros::std_msgs::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::std_msgs::Header&>(
      ::ros::std_msgs::_Header_default_instance_);
}
inline const ::ros::std_msgs::Header& CompressedImage::header() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.CompressedImage.header)
  return _internal_header();
}
inline void CompressedImage::unsafe_arena_set_allocated_header(
    ::ros::std_msgs::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.sensor_msgs.CompressedImage.header)
}
inline ::ros::std_msgs::Header* CompressedImage::release_header() {
  
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
inline ::ros::std_msgs::Header* CompressedImage::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:ros.sensor_msgs.CompressedImage.header)
  
  ::ros::std_msgs::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::ros::std_msgs::Header* CompressedImage::_internal_mutable_header() {
  
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::std_msgs::Header>(GetArenaForAllocation());
    header_ = p;
  }
  return header_;
}
inline ::ros::std_msgs::Header* CompressedImage::mutable_header() {
  ::ros::std_msgs::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.CompressedImage.header)
  return _msg;
}
inline void CompressedImage::set_allocated_header(::ros::std_msgs::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:ros.sensor_msgs.CompressedImage.header)
}

// string format = 2;
inline void CompressedImage::clear_format() {
  format_.ClearToEmpty();
}
inline const std::string& CompressedImage::format() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.CompressedImage.format)
  return _internal_format();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void CompressedImage::set_format(ArgT0&& arg0, ArgT... args) {
 
 format_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.CompressedImage.format)
}
inline std::string* CompressedImage::mutable_format() {
  std::string* _s = _internal_mutable_format();
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.CompressedImage.format)
  return _s;
}
inline const std::string& CompressedImage::_internal_format() const {
  return format_.Get();
}
inline void CompressedImage::_internal_set_format(const std::string& value) {
  
  format_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* CompressedImage::_internal_mutable_format() {
  
  return format_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* CompressedImage::release_format() {
  // @@protoc_insertion_point(field_release:ros.sensor_msgs.CompressedImage.format)
  return format_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void CompressedImage::set_allocated_format(std::string* format) {
  if (format != nullptr) {
    
  } else {
    
  }
  format_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), format,
      GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (format_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    format_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:ros.sensor_msgs.CompressedImage.format)
}

// bytes data = 3;
inline void CompressedImage::clear_data() {
  data_.ClearToEmpty();
}
inline const std::string& CompressedImage::data() const {
  // @@protoc_insertion_point(field_get:ros.sensor_msgs.CompressedImage.data)
  return _internal_data();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void CompressedImage::set_data(ArgT0&& arg0, ArgT... args) {
 
 data_.SetBytes(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:ros.sensor_msgs.CompressedImage.data)
}
inline std::string* CompressedImage::mutable_data() {
  std::string* _s = _internal_mutable_data();
  // @@protoc_insertion_point(field_mutable:ros.sensor_msgs.CompressedImage.data)
  return _s;
}
inline const std::string& CompressedImage::_internal_data() const {
  return data_.Get();
}
inline void CompressedImage::_internal_set_data(const std::string& value) {
  
  data_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* CompressedImage::_internal_mutable_data() {
  
  return data_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* CompressedImage::release_data() {
  // @@protoc_insertion_point(field_release:ros.sensor_msgs.CompressedImage.data)
  return data_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void CompressedImage::set_allocated_data(std::string* data) {
  if (data != nullptr) {
    
  } else {
    
  }
  data_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), data,
      GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (data_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    data_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:ros.sensor_msgs.CompressedImage.data)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace sensor_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fsensor_5fmsgs_2fCompressedImage_2eproto
