// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ros/shape_msgs/Mesh.proto

#include "ros/shape_msgs/Mesh.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace ros {
namespace shape_msgs {
constexpr Mesh::Mesh(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : triangles_()
  , vertices_(){}
struct MeshDefaultTypeInternal {
  constexpr MeshDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~MeshDefaultTypeInternal() {}
  union {
    Mesh _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT MeshDefaultTypeInternal _Mesh_default_instance_;
}  // namespace shape_msgs
}  // namespace ros
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ros_2fshape_5fmsgs_2fMesh_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ros_2fshape_5fmsgs_2fMesh_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ros_2fshape_5fmsgs_2fMesh_2eproto = nullptr;

const uint32_t TableStruct_ros_2fshape_5fmsgs_2fMesh_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::ros::shape_msgs::Mesh, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::ros::shape_msgs::Mesh, triangles_),
  PROTOBUF_FIELD_OFFSET(::ros::shape_msgs::Mesh, vertices_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::ros::shape_msgs::Mesh)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::ros::shape_msgs::_Mesh_default_instance_),
};

const char descriptor_table_protodef_ros_2fshape_5fmsgs_2fMesh_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\031ros/shape_msgs/Mesh.proto\022\016ros.shape_m"
  "sgs\032\035ros/geometry_msgs/Point.proto\032!ros/"
  "shape_msgs/MeshTriangle.proto\"c\n\004Mesh\022/\n"
  "\ttriangles\030\001 \003(\0132\034.ros.shape_msgs.MeshTr"
  "iangle\022*\n\010vertices\030\002 \003(\0132\030.ros.geometry_"
  "msgs.Pointb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ros_2fshape_5fmsgs_2fMesh_2eproto_deps[2] = {
  &::descriptor_table_ros_2fgeometry_5fmsgs_2fPoint_2eproto,
  &::descriptor_table_ros_2fshape_5fmsgs_2fMeshTriangle_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ros_2fshape_5fmsgs_2fMesh_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ros_2fshape_5fmsgs_2fMesh_2eproto = {
  false, false, 218, descriptor_table_protodef_ros_2fshape_5fmsgs_2fMesh_2eproto, "ros/shape_msgs/Mesh.proto", 
  &descriptor_table_ros_2fshape_5fmsgs_2fMesh_2eproto_once, descriptor_table_ros_2fshape_5fmsgs_2fMesh_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_ros_2fshape_5fmsgs_2fMesh_2eproto::offsets,
  file_level_metadata_ros_2fshape_5fmsgs_2fMesh_2eproto, file_level_enum_descriptors_ros_2fshape_5fmsgs_2fMesh_2eproto, file_level_service_descriptors_ros_2fshape_5fmsgs_2fMesh_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_ros_2fshape_5fmsgs_2fMesh_2eproto_getter() {
  return &descriptor_table_ros_2fshape_5fmsgs_2fMesh_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_ros_2fshape_5fmsgs_2fMesh_2eproto(&descriptor_table_ros_2fshape_5fmsgs_2fMesh_2eproto);
namespace ros {
namespace shape_msgs {

// ===================================================================

class Mesh::_Internal {
 public:
};

void Mesh::clear_triangles() {
  triangles_.Clear();
}
void Mesh::clear_vertices() {
  vertices_.Clear();
}
Mesh::Mesh(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  triangles_(arena),
  vertices_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:ros.shape_msgs.Mesh)
}
Mesh::Mesh(const Mesh& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      triangles_(from.triangles_),
      vertices_(from.vertices_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:ros.shape_msgs.Mesh)
}

inline void Mesh::SharedCtor() {
}

Mesh::~Mesh() {
  // @@protoc_insertion_point(destructor:ros.shape_msgs.Mesh)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Mesh::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void Mesh::ArenaDtor(void* object) {
  Mesh* _this = reinterpret_cast< Mesh* >(object);
  (void)_this;
}
void Mesh::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Mesh::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Mesh::Clear() {
// @@protoc_insertion_point(message_clear_start:ros.shape_msgs.Mesh)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  triangles_.Clear();
  vertices_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Mesh::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // repeated .ros.shape_msgs.MeshTriangle triangles = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_triangles(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<10>(ptr));
        } else
          goto handle_unusual;
        continue;
      // repeated .ros.geometry_msgs.Point vertices = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_vertices(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* Mesh::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:ros.shape_msgs.Mesh)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .ros.shape_msgs.MeshTriangle triangles = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_triangles_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, this->_internal_triangles(i), target, stream);
  }

  // repeated .ros.geometry_msgs.Point vertices = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_vertices_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, this->_internal_vertices(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ros.shape_msgs.Mesh)
  return target;
}

size_t Mesh::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ros.shape_msgs.Mesh)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .ros.shape_msgs.MeshTriangle triangles = 1;
  total_size += 1UL * this->_internal_triangles_size();
  for (const auto& msg : this->triangles_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .ros.geometry_msgs.Point vertices = 2;
  total_size += 1UL * this->_internal_vertices_size();
  for (const auto& msg : this->vertices_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Mesh::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Mesh::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Mesh::GetClassData() const { return &_class_data_; }

void Mesh::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Mesh *>(to)->MergeFrom(
      static_cast<const Mesh &>(from));
}


void Mesh::MergeFrom(const Mesh& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ros.shape_msgs.Mesh)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  triangles_.MergeFrom(from.triangles_);
  vertices_.MergeFrom(from.vertices_);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Mesh::CopyFrom(const Mesh& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ros.shape_msgs.Mesh)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Mesh::IsInitialized() const {
  return true;
}

void Mesh::InternalSwap(Mesh* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  triangles_.InternalSwap(&other->triangles_);
  vertices_.InternalSwap(&other->vertices_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Mesh::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_ros_2fshape_5fmsgs_2fMesh_2eproto_getter, &descriptor_table_ros_2fshape_5fmsgs_2fMesh_2eproto_once,
      file_level_metadata_ros_2fshape_5fmsgs_2fMesh_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace shape_msgs
}  // namespace ros
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::ros::shape_msgs::Mesh* Arena::CreateMaybeMessage< ::ros::shape_msgs::Mesh >(Arena* arena) {
  return Arena::CreateMessageInternal< ::ros::shape_msgs::Mesh >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
