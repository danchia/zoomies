#include "hw/camera.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "spdlog/spdlog.h"

namespace {
constexpr int kVideoPort = 1;

void ControlCb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  mmal_buffer_header_release(buffer);
}

void BufCb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  Camera &camera = *(Camera *)port->userdata;
  camera.BufferCallback(port, buffer);
}

}  // namespace

Camera::Camera(uint32_t width, uint32_t height, uint32_t fps)
    : width_(width), height_(height), fps_(fps) {
  if (width_ % 32 != 0) {
    spdlog::critical("Width must be multiple of 32");
  }
  if (height_ % 16 != 0) {
    spdlog::critical("Height must be multiple of 16");
  }
}

bool Camera::Start(std::function<void(uint8_t *buf, int len)> cb) {
  cb_ = std::move(cb);
  // Inspiration from RaspiVideoYUV

  auto status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera_);
  if (status != MMAL_SUCCESS) {
    spdlog::critical("failed to create camera");
    return false;
  }

  MMAL_PARAMETER_INT32_T camera_num = {
      {MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, 0};
  status = mmal_port_parameter_set(camera_->control, &camera_num.hdr);
  if (status != MMAL_SUCCESS) {
    spdlog::critical("camera port param set");
    return false;
  }

  if (!camera_->output_num) {
    spdlog::critical("camera has no output port");
    return false;
  }
  // auto sesnor mode
  status = mmal_port_parameter_set_uint32(
      camera_->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, 0);
  if (status != MMAL_SUCCESS) {
    spdlog::critical("camera sensor config");
    return false;
  }

  MMAL_PORT_T *video_port = camera_->output[kVideoPort];
  status = mmal_port_enable(camera_->control, ControlCb);
  if (status != MMAL_SUCCESS) {
    spdlog::critical("camera port enable");
    return false;
  }

  MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
      {MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
      .max_stills_w = width_,
      .max_stills_h = height_,
      .stills_yuv422 = 0,
      .one_shot_stills = 0,
      .max_preview_video_w = width_,
      .max_preview_video_h = height_,
      .num_preview_video_frames = 3,
      .stills_capture_circular_buffer_height = 0,
      .fast_preview_resume = 0,
      .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC};
  mmal_port_parameter_set(camera_->control, &cam_config.hdr);

  MMAL_PARAMETER_EXPOSUREMETERINGMODE_T meter_mode = {
      {MMAL_PARAMETER_EXP_METERING_MODE, sizeof(meter_mode)},
      MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX};
  mmal_port_parameter_set(camera_->control, &meter_mode.hdr);

  auto format = video_port->format;
  format->encoding = MMAL_ENCODING_I420;
  format->encoding_variant = MMAL_ENCODING_I420;
  format->es->video.width = width_;
  format->es->video.height = height_;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = width_;
  format->es->video.crop.height = height_;
  format->es->video.frame_rate.num = fps_;
  format->es->video.frame_rate.den = 1;

  status = mmal_port_format_commit(video_port);
  if (status != MMAL_SUCCESS) {
    spdlog::critical("format commit");
    return false;
  }

  video_port->buffer_num = 1;  // prefer to drop frame when we're behind.
  video_port->userdata = (struct MMAL_PORT_USERDATA_T *)this;

  status = mmal_component_enable(camera_);
  if (status != MMAL_SUCCESS) {
    spdlog::critical("camera enable");
    return false;
  }

  spdlog::info("Camera enabled {} x {} @ {} fps", width_, height_, fps_);

  pool_ = mmal_port_pool_create(video_port, video_port->buffer_num,
                                video_port->buffer_size);
  if (!pool_) {
    spdlog::critical("camera failed to create buffer header pool");
    return false;
  }
  status = mmal_port_enable(video_port, BufCb);
  if (status != MMAL_SUCCESS) {
    spdlog::critical("camera failed to create buffer header pool");
    return false;
  }

  int qlen = mmal_queue_length(pool_->queue);
  for (int i = 0; i < qlen; ++i) {
    auto *buffer = mmal_queue_get(pool_->queue);
    if (!buffer) {
      spdlog::critical("unable to get buffer from pool");
    }
    if (mmal_port_send_buffer(video_port, buffer) != MMAL_SUCCESS) {
      spdlog::critical("unable to get send buffer to video port");
    }
  }

  if (mmal_port_parameter_set_boolean(video_port, MMAL_PARAMETER_CAPTURE, 1) !=
      MMAL_SUCCESS) {
    spdlog::critical("unable to start capture");
    return false;
  }

  return true;
}

void Camera::Stop() {
  if (mmal_port_parameter_set_boolean(camera_->output[kVideoPort],
                                      MMAL_PARAMETER_CAPTURE,
                                      0) != MMAL_SUCCESS) {
    spdlog::critical("unable to stop capture");
  }
  mmal_component_destroy(camera_);
  camera_ = nullptr;
}

void Camera::BufferCallback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
  if (buffer->length > 0) {
    mmal_buffer_header_mem_lock(buffer);
    cb_(buffer->data, buffer->length);
    mmal_buffer_header_mem_unlock(buffer);
  }

  mmal_buffer_header_release(buffer);

  if (port->is_enabled) {
    auto *new_buf = mmal_queue_get(pool_->queue);
    MMAL_STATUS_T s;
    if (new_buf != nullptr) {
      s = mmal_port_send_buffer(port, new_buf);
    }

    if (new_buf == nullptr || s != MMAL_SUCCESS) {
      spdlog::warn("Unable to return buffer to camera video port");
    }
  }
}