// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/stereo_msgs/DisparityImage.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ros_2fstereo_5fmsgs_2fDisparityImage_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ros_2fstereo_5fmsgs_2fDisparityImage_2eproto

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
#include "ros/sensor_msgs/Image.pb.h"
#include "ros/sensor_msgs/RegionOfInterest.pb.h"
#include "ros/std_msgs/Header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ros_2fstereo_5fmsgs_2fDisparityImage_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ros_2fstereo_5fmsgs_2fDisparityImage_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fstereo_5fmsgs_2fDisparityImage_2eproto;
namespace ros {
namespace stereo_msgs {
class DisparityImage;
struct DisparityImageDefaultTypeInternal;
extern DisparityImageDefaultTypeInternal _DisparityImage_default_instance_;
}  // namespace stereo_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> ::ros::stereo_msgs::DisparityImage* Arena::CreateMaybeMessage<::ros::stereo_msgs::DisparityImage>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace ros {
namespace stereo_msgs {

// ===================================================================

class DisparityImage final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:ros.stereo_msgs.DisparityImage) */ {
 public:
  inline DisparityImage() : DisparityImage(nullptr) {}
  ~DisparityImage() override;
  explicit constexpr DisparityImage(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  DisparityImage(const DisparityImage& from);
  DisparityImage(DisparityImage&& from) noexcept
    : DisparityImage() {
    *this = ::std::move(from);
  }

  inline DisparityImage& operator=(const DisparityImage& from) {
    CopyFrom(from);
    return *this;
  }
  inline DisparityImage& operator=(DisparityImage&& from) noexcept {
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
  static const DisparityImage& default_instance() {
    return *internal_default_instance();
  }
  static inline const DisparityImage* internal_default_instance() {
    return reinterpret_cast<const DisparityImage*>(
               &_DisparityImage_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(DisparityImage& a, DisparityImage& b) {
    a.Swap(&b);
  }
  inline void Swap(DisparityImage* other) {
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
  void UnsafeArenaSwap(DisparityImage* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  DisparityImage* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<DisparityImage>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const DisparityImage& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const DisparityImage& from);
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
  void InternalSwap(DisparityImage* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "ros.stereo_msgs.DisparityImage";
  }
  protected:
  explicit DisparityImage(::PROTOBUF_NAMESPACE_ID::Arena* arena,
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
    kHeaderFieldNumber = 1,
    kImageFieldNumber = 2,
    kValidWindowFieldNumber = 5,
    kFFieldNumber = 3,
    kTFieldNumber = 4,
    kMinDisparityFieldNumber = 6,
    kMaxDisparityFieldNumber = 7,
    kDeltaDFieldNumber = 8,
  };
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

  // .ros.sensor_msgs.Image image = 2;
  bool has_image() const;
  private:
  bool _internal_has_image() const;
  public:
  void clear_image();
  const ::ros::sensor_msgs::Image& image() const;
  PROTOBUF_NODISCARD ::ros::sensor_msgs::Image* release_image();
  ::ros::sensor_msgs::Image* mutable_image();
  void set_allocated_image(::ros::sensor_msgs::Image* image);
  private:
  const ::ros::sensor_msgs::Image& _internal_image() const;
  ::ros::sensor_msgs::Image* _internal_mutable_image();
  public:
  void unsafe_arena_set_allocated_image(
      ::ros::sensor_msgs::Image* image);
  ::ros::sensor_msgs::Image* unsafe_arena_release_image();

  // .ros.sensor_msgs.RegionOfInterest valid_window = 5;
  bool has_valid_window() const;
  private:
  bool _internal_has_valid_window() const;
  public:
  void clear_valid_window();
  const ::ros::sensor_msgs::RegionOfInterest& valid_window() const;
  PROTOBUF_NODISCARD ::ros::sensor_msgs::RegionOfInterest* release_valid_window();
  ::ros::sensor_msgs::RegionOfInterest* mutable_valid_window();
  void set_allocated_valid_window(::ros::sensor_msgs::RegionOfInterest* valid_window);
  private:
  const ::ros::sensor_msgs::RegionOfInterest& _internal_valid_window() const;
  ::ros::sensor_msgs::RegionOfInterest* _internal_mutable_valid_window();
  public:
  void unsafe_arena_set_allocated_valid_window(
      ::ros::sensor_msgs::RegionOfInterest* valid_window);
  ::ros::sensor_msgs::RegionOfInterest* unsafe_arena_release_valid_window();

  // float f = 3;
  void clear_f();
  float f() const;
  void set_f(float value);
  private:
  float _internal_f() const;
  void _internal_set_f(float value);
  public:

  // float T = 4;
  void clear_t();
  float t() const;
  void set_t(float value);
  private:
  float _internal_t() const;
  void _internal_set_t(float value);
  public:

  // float min_disparity = 6;
  void clear_min_disparity();
  float min_disparity() const;
  void set_min_disparity(float value);
  private:
  float _internal_min_disparity() const;
  void _internal_set_min_disparity(float value);
  public:

  // float max_disparity = 7;
  void clear_max_disparity();
  float max_disparity() const;
  void set_max_disparity(float value);
  private:
  float _internal_max_disparity() const;
  void _internal_set_max_disparity(float value);
  public:

  // float delta_d = 8;
  void clear_delta_d();
  float delta_d() const;
  void set_delta_d(float value);
  private:
  float _internal_delta_d() const;
  void _internal_set_delta_d(float value);
  public:

  // @@protoc_insertion_point(class_scope:ros.stereo_msgs.DisparityImage)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::ros::std_msgs::Header* header_;
  ::ros::sensor_msgs::Image* image_;
  ::ros::sensor_msgs::RegionOfInterest* valid_window_;
  float f_;
  float t_;
  float min_disparity_;
  float max_disparity_;
  float delta_d_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ros_2fstereo_5fmsgs_2fDisparityImage_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DisparityImage

// .ros.std_msgs.Header header = 1;
inline bool DisparityImage::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool DisparityImage::has_header() const {
  return _internal_has_header();
}
inline const ::ros::std_msgs::Header& DisparityImage::_internal_header() const {
  const ::ros::std_msgs::Header* p = header_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::std_msgs::Header&>(
      ::ros::std_msgs::_Header_default_instance_);
}
inline const ::ros::std_msgs::Header& DisparityImage::header() const {
  // @@protoc_insertion_point(field_get:ros.stereo_msgs.DisparityImage.header)
  return _internal_header();
}
inline void DisparityImage::unsafe_arena_set_allocated_header(
    ::ros::std_msgs::Header* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.stereo_msgs.DisparityImage.header)
}
inline ::ros::std_msgs::Header* DisparityImage::release_header() {
  
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
inline ::ros::std_msgs::Header* DisparityImage::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:ros.stereo_msgs.DisparityImage.header)
  
  ::ros::std_msgs::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::ros::std_msgs::Header* DisparityImage::_internal_mutable_header() {
  
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::std_msgs::Header>(GetArenaForAllocation());
    header_ = p;
  }
  return header_;
}
inline ::ros::std_msgs::Header* DisparityImage::mutable_header() {
  ::ros::std_msgs::Header* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:ros.stereo_msgs.DisparityImage.header)
  return _msg;
}
inline void DisparityImage::set_allocated_header(::ros::std_msgs::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:ros.stereo_msgs.DisparityImage.header)
}

// .ros.sensor_msgs.Image image = 2;
inline bool DisparityImage::_internal_has_image() const {
  return this != internal_default_instance() && image_ != nullptr;
}
inline bool DisparityImage::has_image() const {
  return _internal_has_image();
}
inline const ::ros::sensor_msgs::Image& DisparityImage::_internal_image() const {
  const ::ros::sensor_msgs::Image* p = image_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::sensor_msgs::Image&>(
      ::ros::sensor_msgs::_Image_default_instance_);
}
inline const ::ros::sensor_msgs::Image& DisparityImage::image() const {
  // @@protoc_insertion_point(field_get:ros.stereo_msgs.DisparityImage.image)
  return _internal_image();
}
inline void DisparityImage::unsafe_arena_set_allocated_image(
    ::ros::sensor_msgs::Image* image) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(image_);
  }
  image_ = image;
  if (image) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.stereo_msgs.DisparityImage.image)
}
inline ::ros::sensor_msgs::Image* DisparityImage::release_image() {
  
  ::ros::sensor_msgs::Image* temp = image_;
  image_ = nullptr;
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
inline ::ros::sensor_msgs::Image* DisparityImage::unsafe_arena_release_image() {
  // @@protoc_insertion_point(field_release:ros.stereo_msgs.DisparityImage.image)
  
  ::ros::sensor_msgs::Image* temp = image_;
  image_ = nullptr;
  return temp;
}
inline ::ros::sensor_msgs::Image* DisparityImage::_internal_mutable_image() {
  
  if (image_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::sensor_msgs::Image>(GetArenaForAllocation());
    image_ = p;
  }
  return image_;
}
inline ::ros::sensor_msgs::Image* DisparityImage::mutable_image() {
  ::ros::sensor_msgs::Image* _msg = _internal_mutable_image();
  // @@protoc_insertion_point(field_mutable:ros.stereo_msgs.DisparityImage.image)
  return _msg;
}
inline void DisparityImage::set_allocated_image(::ros::sensor_msgs::Image* image) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(image_);
  }
  if (image) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(image));
    if (message_arena != submessage_arena) {
      image = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, image, submessage_arena);
    }
    
  } else {
    
  }
  image_ = image;
  // @@protoc_insertion_point(field_set_allocated:ros.stereo_msgs.DisparityImage.image)
}

// float f = 3;
inline void DisparityImage::clear_f() {
  f_ = 0;
}
inline float DisparityImage::_internal_f() const {
  return f_;
}
inline float DisparityImage::f() const {
  // @@protoc_insertion_point(field_get:ros.stereo_msgs.DisparityImage.f)
  return _internal_f();
}
inline void DisparityImage::_internal_set_f(float value) {
  
  f_ = value;
}
inline void DisparityImage::set_f(float value) {
  _internal_set_f(value);
  // @@protoc_insertion_point(field_set:ros.stereo_msgs.DisparityImage.f)
}

// float T = 4;
inline void DisparityImage::clear_t() {
  t_ = 0;
}
inline float DisparityImage::_internal_t() const {
  return t_;
}
inline float DisparityImage::t() const {
  // @@protoc_insertion_point(field_get:ros.stereo_msgs.DisparityImage.T)
  return _internal_t();
}
inline void DisparityImage::_internal_set_t(float value) {
  
  t_ = value;
}
inline void DisparityImage::set_t(float value) {
  _internal_set_t(value);
  // @@protoc_insertion_point(field_set:ros.stereo_msgs.DisparityImage.T)
}

// .ros.sensor_msgs.RegionOfInterest valid_window = 5;
inline bool DisparityImage::_internal_has_valid_window() const {
  return this != internal_default_instance() && valid_window_ != nullptr;
}
inline bool DisparityImage::has_valid_window() const {
  return _internal_has_valid_window();
}
inline const ::ros::sensor_msgs::RegionOfInterest& DisparityImage::_internal_valid_window() const {
  const ::ros::sensor_msgs::RegionOfInterest* p = valid_window_;
  return p != nullptr ? *p : reinterpret_cast<const ::ros::sensor_msgs::RegionOfInterest&>(
      ::ros::sensor_msgs::_RegionOfInterest_default_instance_);
}
inline const ::ros::sensor_msgs::RegionOfInterest& DisparityImage::valid_window() const {
  // @@protoc_insertion_point(field_get:ros.stereo_msgs.DisparityImage.valid_window)
  return _internal_valid_window();
}
inline void DisparityImage::unsafe_arena_set_allocated_valid_window(
    ::ros::sensor_msgs::RegionOfInterest* valid_window) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(valid_window_);
  }
  valid_window_ = valid_window;
  if (valid_window) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:ros.stereo_msgs.DisparityImage.valid_window)
}
inline ::ros::sensor_msgs::RegionOfInterest* DisparityImage::release_valid_window() {
  
  ::ros::sensor_msgs::RegionOfInterest* temp = valid_window_;
  valid_window_ = nullptr;
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
inline ::ros::sensor_msgs::RegionOfInterest* DisparityImage::unsafe_arena_release_valid_window() {
  // @@protoc_insertion_point(field_release:ros.stereo_msgs.DisparityImage.valid_window)
  
  ::ros::sensor_msgs::RegionOfInterest* temp = valid_window_;
  valid_window_ = nullptr;
  return temp;
}
inline ::ros::sensor_msgs::RegionOfInterest* DisparityImage::_internal_mutable_valid_window() {
  
  if (valid_window_ == nullptr) {
    auto* p = CreateMaybeMessage<::ros::sensor_msgs::RegionOfInterest>(GetArenaForAllocation());
    valid_window_ = p;
  }
  return valid_window_;
}
inline ::ros::sensor_msgs::RegionOfInterest* DisparityImage::mutable_valid_window() {
  ::ros::sensor_msgs::RegionOfInterest* _msg = _internal_mutable_valid_window();
  // @@protoc_insertion_point(field_mutable:ros.stereo_msgs.DisparityImage.valid_window)
  return _msg;
}
inline void DisparityImage::set_allocated_valid_window(::ros::sensor_msgs::RegionOfInterest* valid_window) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(valid_window_);
  }
  if (valid_window) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper<
            ::PROTOBUF_NAMESPACE_ID::MessageLite>::GetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(valid_window));
    if (message_arena != submessage_arena) {
      valid_window = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, valid_window, submessage_arena);
    }
    
  } else {
    
  }
  valid_window_ = valid_window;
  // @@protoc_insertion_point(field_set_allocated:ros.stereo_msgs.DisparityImage.valid_window)
}

// float min_disparity = 6;
inline void DisparityImage::clear_min_disparity() {
  min_disparity_ = 0;
}
inline float DisparityImage::_internal_min_disparity() const {
  return min_disparity_;
}
inline float DisparityImage::min_disparity() const {
  // @@protoc_insertion_point(field_get:ros.stereo_msgs.DisparityImage.min_disparity)
  return _internal_min_disparity();
}
inline void DisparityImage::_internal_set_min_disparity(float value) {
  
  min_disparity_ = value;
}
inline void DisparityImage::set_min_disparity(float value) {
  _internal_set_min_disparity(value);
  // @@protoc_insertion_point(field_set:ros.stereo_msgs.DisparityImage.min_disparity)
}

// float max_disparity = 7;
inline void DisparityImage::clear_max_disparity() {
  max_disparity_ = 0;
}
inline float DisparityImage::_internal_max_disparity() const {
  return max_disparity_;
}
inline float DisparityImage::max_disparity() const {
  // @@protoc_insertion_point(field_get:ros.stereo_msgs.DisparityImage.max_disparity)
  return _internal_max_disparity();
}
inline void DisparityImage::_internal_set_max_disparity(float value) {
  
  max_disparity_ = value;
}
inline void DisparityImage::set_max_disparity(float value) {
  _internal_set_max_disparity(value);
  // @@protoc_insertion_point(field_set:ros.stereo_msgs.DisparityImage.max_disparity)
}

// float delta_d = 8;
inline void DisparityImage::clear_delta_d() {
  delta_d_ = 0;
}
inline float DisparityImage::_internal_delta_d() const {
  return delta_d_;
}
inline float DisparityImage::delta_d() const {
  // @@protoc_insertion_point(field_get:ros.stereo_msgs.DisparityImage.delta_d)
  return _internal_delta_d();
}
inline void DisparityImage::_internal_set_delta_d(float value) {
  
  delta_d_ = value;
}
inline void DisparityImage::set_delta_d(float value) {
  _internal_set_delta_d(value);
  // @@protoc_insertion_point(field_set:ros.stereo_msgs.DisparityImage.delta_d)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace stereo_msgs
}  // namespace ros

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ros_2fstereo_5fmsgs_2fDisparityImage_2eproto
