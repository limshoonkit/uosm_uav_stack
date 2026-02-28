/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include "node_params.hpp"
#include <memory>
#include <string>
#include <thread>
#include <atomic>

namespace uosm
{
    namespace perception
    {

        class CSICameraComponent : public rclcpp::Node
        {
        public:
            explicit CSICameraComponent(const rclcpp::NodeOptions &options)
                : Node("csi_camera_node", options)
            {
                sensor_id_ = util::getParam<int>(this, "sensor_id", 0, "  sensor_id: ");
                sensor_mode_ = util::getParam<int>(this, "sensor_mode", -1, "  sensor_mode: ");
                input_width_ = util::getParam<int>(this, "input_width", 1920, "  input_width: ");
                input_height_ = util::getParam<int>(this, "input_height", 1080, "  input_height: ");
                output_width_ = util::getParam<int>(this, "output_width", 640, "  output_width: ");
                output_height_ = util::getParam<int>(this, "output_height", 360, "  output_height: ");
                framerate_ = util::getParam<int>(this, "framerate", 30, "  framerate: ");
                output_framerate_ = util::getParam<int>(this, "output_framerate", 24, "  output_framerate: ");
                frame_id_ = util::getParam<std::string>(this, "frame_id", "camera_frame", "  frame_id: ");
                auto camera_name = util::getParam<std::string>(this, "camera_name", "csi_camera", "  camera_name: ");
                auto camera_info_url = util::getParam<std::string>(this, "camera_info_url", "", "  camera_info_url: ");
                enable_encoding_ = util::getParam<bool>(this, "enable_encoding", true, "  enable_encoding: ");
                encoding_format_ = util::getParam<std::string>(this, "encoding_format", "h264", "  encoding_format: ");
                encoding_bitrate_ = util::getParam<int>(this, "encoding_bitrate", 4000000, "  encoding_bitrate: ");
                publish_raw_ = util::getParam<bool>(this, "publish_raw", true, "  publish_raw: ");
                publish_compressed_ = util::getParam<bool>(this, "publish_compressed", true, "  publish_compressed: ");
                tnr_mode_ = util::getParam<int>(this, "tnr_mode", 1, "  tnr_mode: ");
                tnr_strength_ = util::getParam<double>(this, "tnr_strength", 0.5, "  tnr_strength: ");
                ee_mode_ = util::getParam<int>(this, "ee_mode", 1, "  ee_mode: ");
                ee_strength_ = util::getParam<double>(this, "ee_strength", 0.3, "  ee_strength: ");
                wbmode_ = util::getParam<int>(this, "wbmode", 1, "  wbmode: ");
                aeantibanding_ = util::getParam<int>(this, "aeantibanding", 1, "  aeantibanding: ");
                saturation_ = util::getParam<double>(this, "saturation", 1.1, "  saturation: ");
                exposurecompensation_ = util::getParam<double>(this, "exposurecompensation", 0.0, "  exposurecompensation: ");
                max_buffers_ = util::getParam<int>(this, "max_buffers", 2, "  max_buffers: ");
                drop_frames_ = util::getParam<bool>(this, "drop_frames", true, "  drop_frames: ");
                leaky_queue_ = util::getParam<bool>(this, "leaky_queue", true, "  leaky_queue: ");
                sync_appsink_ = util::getParam<bool>(this, "sync_appsink", false, "  sync_appsink: ");
                encoder_preset_ = util::getParam<int>(this, "encoder_preset", 1, "  encoder_preset: ");

                cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name);
                if (cinfo_manager_->loadCameraInfo(camera_info_url))
                {
                    RCLCPP_INFO(get_logger(), "Successfully loaded camera calibration from %s", camera_info_url.c_str());
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Could not load camera calibration from %s, using uncalibrated default", camera_info_url.c_str());
                }

                // Initialize GStreamer
                gst_init(nullptr, nullptr);

                // Initialize Node after one second
                init_timer_ = create_wall_timer(
                    std::chrono::seconds(1),
                    std::bind(&CSICameraComponent::initializeNode, this));
            }

            ~CSICameraComponent()
            {
                running_ = false;
                if (processing_thread_.joinable())
                {
                    processing_thread_.join();
                }

                if (pipeline_)
                {
                    gst_element_set_state(pipeline_, GST_STATE_NULL);
                    gst_object_unref(pipeline_);
                }
            }

        private:
            void initializeNode()
            {
                init_timer_->cancel();
                auto qos = rclcpp::QoS(1).best_effort();
                image_transport::ImageTransport it(shared_from_this());
                if (publish_raw_)
                {
                    image_pub_ = it.advertise("image_raw", 1);
                }
                if (publish_compressed_)
                {
                    compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", qos);
                    foxglove_video_pub_ = create_publisher<foxglove_msgs::msg::CompressedVideo>(
                        "image_raw/" + encoding_format_, qos);
                }
                camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);

                // Start GStreamer pipeline
                if (!initializeGStreamerPipeline())
                {
                    RCLCPP_ERROR(get_logger(), "Failed to initialize GStreamer pipeline");
                    return;
                }

                gst_element_set_state(pipeline_, GST_STATE_PLAYING);
                running_ = true;
                processing_thread_ = std::thread(&CSICameraComponent::processFrames, this);
                RCLCPP_INFO(get_logger(), "CSI Camera Node initialized successfully!");
            }

            bool initializeGStreamerPipeline()
            {
                // Build the source element with all parameters
                std::string source_element = "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id_) +
                                             " tnr-mode=" + std::to_string(tnr_mode_) +
                                             " tnr-strength=" + std::to_string(tnr_strength_) +
                                             " ee-mode=" + std::to_string(ee_mode_) +
                                             " ee-strength=" + std::to_string(ee_strength_) +
                                             " wbmode=" + std::to_string(wbmode_) +
                                             " aeantibanding=" + std::to_string(aeantibanding_) +
                                             " saturation=" + std::to_string(saturation_) +
                                             " exposurecompensation=" + std::to_string(exposurecompensation_) +
                                             " sensor-mode=" + std::to_string(sensor_mode_);

                // Define the source caps
                std::string source_caps = " ! video/x-raw(memory:NVMM),width=" + std::to_string(input_width_) +
                                          ",height=" + std::to_string(input_height_) +
                                          ",framerate=" + std::to_string(framerate_) + "/1,format=NV12";

                std::string source_base = source_element + source_caps;

                // Add videorate if the output framerate is different from the source
                if (output_framerate_ < framerate_)
                {
                    source_base += " ! videorate ! video/x-raw(memory:NVMM),framerate=" + std::to_string(output_framerate_) + "/1";
                }

                // Construct the main processing pipeline
                std::string processing_pipeline = source_base + " ! nvvidconv flip-method=0 ! " +
                                                  "video/x-raw(memory:NVMM),width=" + std::to_string(output_width_) +
                                                  ",height=" + std::to_string(output_height_) + ",format=NV12";

                // Queue settings for low latency
                std::string queue_settings = "queue max-size-buffers=" + std::to_string(max_buffers_);
                if (leaky_queue_)
                {
                    queue_settings += " leaky=downstream";
                }

                // AppSink settings for low latency
                std::string appsink_settings = " max-buffers=" + std::to_string(max_buffers_);
                if (drop_frames_)
                {
                    appsink_settings += " drop=true";
                }
                if (!sync_appsink_)
                {
                    appsink_settings += " sync=false";
                }

                std::string pipeline_str;
                if (publish_raw_ && enable_encoding_ && publish_compressed_)
                {
                    // both raw and compressed
                    std::string encoder = (encoding_format_ == "h265") ? "nvv4l2h265enc" : "nvv4l2h264enc";
                    std::string encoder_settings = encoder + " bitrate=" + std::to_string(encoding_bitrate_) +
                                                   " preset-level=" + std::to_string(encoder_preset_) +
                                                   " profile=0 insert-sps-pps=true";

                    pipeline_str = processing_pipeline + " ! tee name=t " +
                                   "t. ! " + queue_settings + " ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink name=raw_sink" + appsink_settings + " " +
                                   "t. ! " + queue_settings + " ! " + encoder_settings + " ! " +
                                   ((encoding_format_ == "h265") ? "video/x-h265" : "video/x-h264") + ",stream-format=byte-stream ! appsink name=encoded_sink" + appsink_settings;
                }
                else if (publish_raw_)
                {
                    // raw
                    pipeline_str = processing_pipeline + " ! nvvidconv ! video/x-raw,format=BGRx ! " +
                                   "videoconvert ! video/x-raw,format=BGR ! appsink name=raw_sink" + appsink_settings;
                }
                else if (enable_encoding_ && publish_compressed_)
                {
                    // compressed
                    std::string encoder = (encoding_format_ == "h265") ? "nvv4l2h265enc" : "nvv4l2h264enc";
                    std::string encoder_settings = encoder + " bitrate=" + std::to_string(encoding_bitrate_) +
                                                   " preset-level=" + std::to_string(encoder_preset_) +
                                                   " profile=0 insert-sps-pps=true";

                    pipeline_str = processing_pipeline + " ! " + encoder_settings + " ! " +
                                   ((encoding_format_ == "h265") ? "video/x-h265" : "video/x-h264") + ",stream-format=byte-stream ! appsink name=encoded_sink" + appsink_settings;
                }
                else
                {
                    RCLCPP_ERROR(get_logger(), "No publishers enabled! Set publish_raw and/or publish_compressed to true.");
                    return false;
                }

                RCLCPP_INFO(get_logger(), "GStreamer pipeline: %s", pipeline_str.c_str());

                // Create the GStreamer pipeline
                GError *error = nullptr;
                pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

                if (!pipeline_ || error)
                {
                    RCLCPP_ERROR(get_logger(), "Failed to create pipeline: %s",
                                 error ? error->message : "Unknown error");
                    if (error)
                        g_error_free(error);
                    return false;
                }

                if (publish_raw_)
                {
                    raw_sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "raw_sink");
                    if (!raw_sink_)
                    {
                        RCLCPP_ERROR(get_logger(), "Failed to get raw_sink");
                        return false;
                    }
                    g_object_set(raw_sink_, "emit-signals", TRUE, nullptr);
                    g_signal_connect(raw_sink_, "new-sample", G_CALLBACK(onNewRawSample), this);
                }

                if (enable_encoding_ && publish_compressed_)
                {
                    encoded_sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "encoded_sink");
                    if (!encoded_sink_)
                    {
                        RCLCPP_ERROR(get_logger(), "Failed to get encoded_sink");
                        return false;
                    }
                    g_object_set(encoded_sink_, "emit-signals", TRUE, nullptr);
                    g_signal_connect(encoded_sink_, "new-sample", G_CALLBACK(onNewEncodedSample), this);
                }

                return true;
            }

            static GstFlowReturn onNewRawSample(GstElement *sink, gpointer user_data)
            {
                auto *node = static_cast<CSICameraComponent *>(user_data);
                if (node->image_pub_.getNumSubscribers() > 0)
                {
                    return node->handleRawSample(sink);
                }
                return GST_FLOW_OK;
            }

            static GstFlowReturn onNewEncodedSample(GstElement *sink, gpointer user_data)
            {
                auto *node = static_cast<CSICameraComponent *>(user_data);
                if (node->compressed_pub_->get_subscription_count() > 0 ||
                    node->foxglove_video_pub_->get_subscription_count() > 0)
                {
                    return node->handleEncodedSample(sink);
                }
                return GST_FLOW_OK;
            }

            GstFlowReturn handleEncodedSample(GstElement *sink)
            {
                if (!publish_compressed_ && compressed_pub_->get_subscription_count() == 0)
                    return GST_FLOW_OK;

                GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
                if (!sample)
                    return GST_FLOW_ERROR;

                GstBuffer *buffer = gst_sample_get_buffer(sample);
                if (!buffer)
                {
                    gst_sample_unref(sample);
                    return GST_FLOW_ERROR;
                }

                // Map buffer
                GstMapInfo map_info;
                if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ))
                {
                    gst_sample_unref(sample);
                    return GST_FLOW_ERROR;
                }

                auto stamp = now();

                // sensor_msgs/CompressedImage (format field is informational only; not decoded by Foxglove)
                if (compressed_pub_->get_subscription_count() > 0)
                {
                    auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                    compressed_msg->header.stamp = stamp;
                    compressed_msg->header.frame_id = frame_id_;
                    compressed_msg->format = encoding_format_;
                    compressed_msg->data.assign(map_info.data, map_info.data + map_info.size);
                    compressed_pub_->publish(std::move(compressed_msg));
                }

                // foxglove_msgs/CompressedVideo — decoded natively by Foxglove
                if (foxglove_video_pub_->get_subscription_count() > 0)
                {
                    auto foxglove_msg = std::make_unique<foxglove_msgs::msg::CompressedVideo>();
                    foxglove_msg->timestamp = stamp;
                    foxglove_msg->frame_id = frame_id_;
                    foxglove_msg->format = encoding_format_;
                    foxglove_msg->data.assign(map_info.data, map_info.data + map_info.size);
                    foxglove_video_pub_->publish(std::move(foxglove_msg));
                }

                // Get camera info from the manager and publish
                auto camera_info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_manager_->getCameraInfo());
                camera_info_msg->header.stamp = stamp;
                camera_info_msg->header.frame_id = frame_id_;
                camera_info_msg->width = output_width_;
                camera_info_msg->height = output_height_;
                camera_info_pub_->publish(std::move(camera_info_msg));

                // Cleanup
                gst_buffer_unmap(buffer, &map_info);
                gst_sample_unref(sample);

                return GST_FLOW_OK;
            }

            GstFlowReturn handleRawSample(GstElement *sink)
            {
                if (!publish_raw_ && image_pub_.getNumSubscribers() == 0)
                    return GST_FLOW_OK;

                GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
                if (!sample)
                    return GST_FLOW_ERROR;

                GstBuffer *buffer = gst_sample_get_buffer(sample);
                GstCaps *caps = gst_sample_get_caps(sample);
                if (!buffer || !caps)
                {
                    gst_sample_unref(sample);
                    return GST_FLOW_ERROR;
                }

                GstVideoInfo video_info;
                if (!gst_video_info_from_caps(&video_info, caps))
                {
                    gst_sample_unref(sample);
                    return GST_FLOW_ERROR;
                }

                GstMapInfo map_info;
                if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ))
                {
                    gst_sample_unref(sample);
                    return GST_FLOW_ERROR;
                }

                cv::Mat frame(output_height_, output_width_, CV_8UC3, map_info.data);
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                auto stamp = now();
                msg->header.stamp = stamp;
                msg->header.frame_id = frame_id_;

                // Get camera info from the manager and publish
                auto camera_info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_manager_->getCameraInfo());
                camera_info_msg->header.stamp = stamp;
                camera_info_msg->header.frame_id = frame_id_;
                camera_info_msg->width = output_width_;
                camera_info_msg->height = output_height_;

                // Publish
                image_pub_.publish(std::move(msg));
                camera_info_pub_->publish(std::move(camera_info_msg));

                // Cleanup
                gst_buffer_unmap(buffer, &map_info);
                gst_sample_unref(sample);

                return GST_FLOW_OK;
            }

            void processFrames()
            {
                while (running_)
                {
                    // Process GStreamer bus messages
                    GstBus *bus = gst_element_get_bus(pipeline_);
                    GstMessage *msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND,
                                                                 static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

                    if (msg)
                    {
                        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR)
                        {
                            GError *error;
                            gchar *debug;
                            gst_message_parse_error(msg, &error, &debug);
                            RCLCPP_ERROR(get_logger(), "GStreamer error: %s", error->message);
                            g_error_free(error);
                            g_free(debug);
                            running_ = false;
                        }
                        else if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS)
                        {
                            RCLCPP_INFO(get_logger(), "End of stream");
                            running_ = false;
                        }
                        gst_message_unref(msg);
                    }
                    gst_object_unref(bus);
                }
            }

            // ROS parameters
            int sensor_id_;
            int sensor_mode_;
            int input_width_;
            int input_height_;
            int output_width_;
            int output_height_;
            int framerate_;
            int output_framerate_;
            std::string frame_id_;
            bool enable_encoding_;
            std::string encoding_format_;
            int encoding_bitrate_;
            bool publish_raw_;
            bool publish_compressed_;

            // Argus camera parameters
            int tnr_mode_;
            double tnr_strength_;
            int ee_mode_;
            double ee_strength_;
            int wbmode_;
            int aeantibanding_;
            double saturation_;
            double exposurecompensation_;

            // Pipeline parameters
            int max_buffers_;
            bool drop_frames_;
            bool leaky_queue_;
            bool sync_appsink_;
            int encoder_preset_;

            // GStreamer elements
            GstElement *pipeline_;
            GstElement *raw_sink_;
            GstElement *encoded_sink_;

            // publishers
            image_transport::Publisher image_pub_;
            rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
            rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr foxglove_video_pub_;
            rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
            std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;

            // timers
            rclcpp::TimerBase::SharedPtr init_timer_;

            // threadings
            std::thread processing_thread_;
            std::atomic<bool> running_;
        };
    } // namespace perception
} // namespace uosm

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uosm::perception::CSICameraComponent)