// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/camera/camera.proto)

#include "camera/camera.grpc.pb.h"
#include "plugins/camera/camera.h"

#include "log.h"
#include <atomic>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <vector>

namespace mavsdk {
namespace backend {

template<typename Camera = Camera>
class CameraServiceImpl final : public rpc::camera::CameraService::Service {
public:
    CameraServiceImpl(Camera& camera) : _camera(camera) {}

    template<typename ResponseType>
    void fillResponseWithResult(ResponseType* response, mavsdk::Camera::Result& result) const
    {
        auto rpc_result = translateToRpcResult(result);

        auto* rpc_camera_result = new rpc::camera::CameraResult();
        rpc_camera_result->set_result(rpc_result);
        rpc_camera_result->set_result_str(mavsdk::Camera::result_str(result));

        response->set_allocated_camera_result(rpc_camera_result);
    }

    static rpc::camera::Mode translateToRpcMode(const mavsdk::Camera::Mode& mode)
    {
        switch (mode) {
            default:
                LogErr() << "Unknown mode enum value: " << static_cast<int>(mode);
            // FALLTHROUGH
            case mavsdk::Camera::Mode::Unknown:
                return rpc::camera::MODE_UNKNOWN;
            case mavsdk::Camera::Mode::Photo:
                return rpc::camera::MODE_PHOTO;
            case mavsdk::Camera::Mode::Video:
                return rpc::camera::MODE_VIDEO;
        }
    }

    static mavsdk::Camera::Mode translateFromRpcMode(const rpc::camera::Mode mode)
    {
        switch (mode) {
            default:
                LogErr() << "Unknown mode enum value: " << static_cast<int>(mode);
            // FALLTHROUGH
            case rpc::camera::MODE_UNKNOWN:
                return mavsdk::Camera::Mode::Unknown;
            case rpc::camera::MODE_PHOTO:
                return mavsdk::Camera::Mode::Photo;
            case rpc::camera::MODE_VIDEO:
                return mavsdk::Camera::Mode::Video;
        }
    }

    static rpc::camera::CameraResult::Result
    translateToRpcResult(const mavsdk::Camera::Result& result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case mavsdk::Camera::Result::Unknown:
                return rpc::camera::CameraResult_Result_RESULT_UNKNOWN;
            case mavsdk::Camera::Result::Success:
                return rpc::camera::CameraResult_Result_RESULT_SUCCESS;
            case mavsdk::Camera::Result::InProgress:
                return rpc::camera::CameraResult_Result_RESULT_IN_PROGRESS;
            case mavsdk::Camera::Result::Busy:
                return rpc::camera::CameraResult_Result_RESULT_BUSY;
            case mavsdk::Camera::Result::Denied:
                return rpc::camera::CameraResult_Result_RESULT_DENIED;
            case mavsdk::Camera::Result::Error:
                return rpc::camera::CameraResult_Result_RESULT_ERROR;
            case mavsdk::Camera::Result::Timeout:
                return rpc::camera::CameraResult_Result_RESULT_TIMEOUT;
            case mavsdk::Camera::Result::WrongArgument:
                return rpc::camera::CameraResult_Result_RESULT_WRONG_ARGUMENT;
        }
    }

    static mavsdk::Camera::Result
    translateFromRpcResult(const rpc::camera::CameraResult::Result result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case rpc::camera::CameraResult_Result_RESULT_UNKNOWN:
                return mavsdk::Camera::Result::Unknown;
            case rpc::camera::CameraResult_Result_RESULT_SUCCESS:
                return mavsdk::Camera::Result::Success;
            case rpc::camera::CameraResult_Result_RESULT_IN_PROGRESS:
                return mavsdk::Camera::Result::InProgress;
            case rpc::camera::CameraResult_Result_RESULT_BUSY:
                return mavsdk::Camera::Result::Busy;
            case rpc::camera::CameraResult_Result_RESULT_DENIED:
                return mavsdk::Camera::Result::Denied;
            case rpc::camera::CameraResult_Result_RESULT_ERROR:
                return mavsdk::Camera::Result::Error;
            case rpc::camera::CameraResult_Result_RESULT_TIMEOUT:
                return mavsdk::Camera::Result::Timeout;
            case rpc::camera::CameraResult_Result_RESULT_WRONG_ARGUMENT:
                return mavsdk::Camera::Result::WrongArgument;
        }
    }

    static std::unique_ptr<rpc::camera::Position>
    translateToRpcPosition(const mavsdk::Camera::Position& position)
    {
        std::unique_ptr<rpc::camera::Position> rpc_obj(new rpc::camera::Position());

        rpc_obj->set_latitude_deg(position.latitude_deg);

        rpc_obj->set_longitude_deg(position.longitude_deg);

        rpc_obj->set_absolute_altitude_m(position.absolute_altitude_m);

        rpc_obj->set_relative_altitude_m(position.relative_altitude_m);

        return rpc_obj;
    }

    static mavsdk::Camera::Position translateFromRpcPosition(const rpc::camera::Position& position)
    {
        mavsdk::Camera::Position obj;

        obj.latitude_deg = position.latitude_deg();

        obj.longitude_deg = position.longitude_deg();

        obj.absolute_altitude_m = position.absolute_altitude_m();

        obj.relative_altitude_m = position.relative_altitude_m();

        return obj;
    }

    static std::unique_ptr<rpc::camera::Quaternion>
    translateToRpcQuaternion(const mavsdk::Camera::Quaternion& quaternion)
    {
        std::unique_ptr<rpc::camera::Quaternion> rpc_obj(new rpc::camera::Quaternion());

        rpc_obj->set_w(quaternion.w);

        rpc_obj->set_x(quaternion.x);

        rpc_obj->set_y(quaternion.y);

        rpc_obj->set_z(quaternion.z);

        return rpc_obj;
    }

    static mavsdk::Camera::Quaternion
    translateFromRpcQuaternion(const rpc::camera::Quaternion& quaternion)
    {
        mavsdk::Camera::Quaternion obj;

        obj.w = quaternion.w();

        obj.x = quaternion.x();

        obj.y = quaternion.y();

        obj.z = quaternion.z();

        return obj;
    }

    static std::unique_ptr<rpc::camera::EulerAngle>
    translateToRpcEulerAngle(const mavsdk::Camera::EulerAngle& euler_angle)
    {
        std::unique_ptr<rpc::camera::EulerAngle> rpc_obj(new rpc::camera::EulerAngle());

        rpc_obj->set_roll_deg(euler_angle.roll_deg);

        rpc_obj->set_pitch_deg(euler_angle.pitch_deg);

        rpc_obj->set_yaw_deg(euler_angle.yaw_deg);

        return rpc_obj;
    }

    static mavsdk::Camera::EulerAngle
    translateFromRpcEulerAngle(const rpc::camera::EulerAngle& euler_angle)
    {
        mavsdk::Camera::EulerAngle obj;

        obj.roll_deg = euler_angle.roll_deg();

        obj.pitch_deg = euler_angle.pitch_deg();

        obj.yaw_deg = euler_angle.yaw_deg();

        return obj;
    }

    static std::unique_ptr<rpc::camera::CaptureInfo>
    translateToRpcCaptureInfo(const mavsdk::Camera::CaptureInfo& capture_info)
    {
        std::unique_ptr<rpc::camera::CaptureInfo> rpc_obj(new rpc::camera::CaptureInfo());

        rpc_obj->set_allocated_position(translateToRpcPosition(capture_info.position).release());

        rpc_obj->set_allocated_attitude_quaternion(
            translateToRpcQuaternion(capture_info.attitude_quaternion).release());

        rpc_obj->set_allocated_attitude_euler_angle(
            translateToRpcEulerAngle(capture_info.attitude_euler_angle).release());

        rpc_obj->set_time_utc_us(capture_info.time_utc_us);

        rpc_obj->set_is_success(capture_info.is_success);

        rpc_obj->set_index(capture_info.index);

        rpc_obj->set_file_url(capture_info.file_url);

        return rpc_obj;
    }

    static mavsdk::Camera::CaptureInfo
    translateFromRpcCaptureInfo(const rpc::camera::CaptureInfo& capture_info)
    {
        mavsdk::Camera::CaptureInfo obj;

        obj.position = translateFromRpcPosition(capture_info.position());

        obj.attitude_quaternion = translateFromRpcQuaternion(capture_info.attitude_quaternion());

        obj.attitude_euler_angle = translateFromRpcEulerAngle(capture_info.attitude_euler_angle());

        obj.time_utc_us = capture_info.time_utc_us();

        obj.is_success = capture_info.is_success();

        obj.index = capture_info.index();

        obj.file_url = capture_info.file_url();

        return obj;
    }

    static std::unique_ptr<rpc::camera::VideoStreamSettings> translateToRpcVideoStreamSettings(
        const mavsdk::Camera::VideoStreamSettings& video_stream_settings)
    {
        std::unique_ptr<rpc::camera::VideoStreamSettings> rpc_obj(
            new rpc::camera::VideoStreamSettings());

        rpc_obj->set_frame_rate_hz(video_stream_settings.frame_rate_hz);

        rpc_obj->set_horizontal_resolution_pix(video_stream_settings.horizontal_resolution_pix);

        rpc_obj->set_vertical_resolution_pix(video_stream_settings.vertical_resolution_pix);

        rpc_obj->set_bit_rate_b_s(video_stream_settings.bit_rate_b_s);

        rpc_obj->set_rotation_deg(video_stream_settings.rotation_deg);

        rpc_obj->set_uri(video_stream_settings.uri);

        return rpc_obj;
    }

    static mavsdk::Camera::VideoStreamSettings translateFromRpcVideoStreamSettings(
        const rpc::camera::VideoStreamSettings& video_stream_settings)
    {
        mavsdk::Camera::VideoStreamSettings obj;

        obj.frame_rate_hz = video_stream_settings.frame_rate_hz();

        obj.horizontal_resolution_pix = video_stream_settings.horizontal_resolution_pix();

        obj.vertical_resolution_pix = video_stream_settings.vertical_resolution_pix();

        obj.bit_rate_b_s = video_stream_settings.bit_rate_b_s();

        obj.rotation_deg = video_stream_settings.rotation_deg();

        obj.uri = video_stream_settings.uri();

        return obj;
    }

    static rpc::camera::VideoStreamInfo::Status
    translateToRpcStatus(const mavsdk::Camera::VideoStreamInfo::Status& status)
    {
        switch (status) {
            default:
                LogErr() << "Unknown status enum value: " << static_cast<int>(status);
            // FALLTHROUGH
            case mavsdk::Camera::VideoStreamInfo::Status::NotRunning:
                return rpc::camera::VideoStreamInfo_Status_STATUS_NOT_RUNNING;
            case mavsdk::Camera::VideoStreamInfo::Status::InProgress:
                return rpc::camera::VideoStreamInfo_Status_STATUS_IN_PROGRESS;
        }
    }

    static mavsdk::Camera::VideoStreamInfo::Status
    translateFromRpcStatus(const rpc::camera::VideoStreamInfo::Status status)
    {
        switch (status) {
            default:
                LogErr() << "Unknown status enum value: " << static_cast<int>(status);
            // FALLTHROUGH
            case rpc::camera::VideoStreamInfo_Status_STATUS_NOT_RUNNING:
                return mavsdk::Camera::VideoStreamInfo::Status::NotRunning;
            case rpc::camera::VideoStreamInfo_Status_STATUS_IN_PROGRESS:
                return mavsdk::Camera::VideoStreamInfo::Status::InProgress;
        }
    }

    static std::unique_ptr<rpc::camera::VideoStreamInfo>
    translateToRpcVideoStreamInfo(const mavsdk::Camera::VideoStreamInfo& video_stream_info)
    {
        std::unique_ptr<rpc::camera::VideoStreamInfo> rpc_obj(new rpc::camera::VideoStreamInfo());

        rpc_obj->set_allocated_settings(
            translateToRpcVideoStreamSettings(video_stream_info.settings).release());

        rpc_obj->set_status(translateToRpcStatus(video_stream_info.status));

        return rpc_obj;
    }

    static mavsdk::Camera::VideoStreamInfo
    translateFromRpcVideoStreamInfo(const rpc::camera::VideoStreamInfo& video_stream_info)
    {
        mavsdk::Camera::VideoStreamInfo obj;

        obj.settings = translateFromRpcVideoStreamSettings(video_stream_info.settings());

        obj.status = translateFromRpcStatus(video_stream_info.status());

        return obj;
    }

    static rpc::camera::Status::StorageStatus
    translateToRpcStorageStatus(const mavsdk::Camera::Status::StorageStatus& storage_status)
    {
        switch (storage_status) {
            default:
                LogErr() << "Unknown storage_status enum value: "
                         << static_cast<int>(storage_status);
            // FALLTHROUGH
            case mavsdk::Camera::Status::StorageStatus::NotAvailable:
                return rpc::camera::Status_StorageStatus_STORAGE_STATUS_NOT_AVAILABLE;
            case mavsdk::Camera::Status::StorageStatus::Unformatted:
                return rpc::camera::Status_StorageStatus_STORAGE_STATUS_UNFORMATTED;
            case mavsdk::Camera::Status::StorageStatus::Formatted:
                return rpc::camera::Status_StorageStatus_STORAGE_STATUS_FORMATTED;
        }
    }

    static mavsdk::Camera::Status::StorageStatus
    translateFromRpcStorageStatus(const rpc::camera::Status::StorageStatus storage_status)
    {
        switch (storage_status) {
            default:
                LogErr() << "Unknown storage_status enum value: "
                         << static_cast<int>(storage_status);
            // FALLTHROUGH
            case rpc::camera::Status_StorageStatus_STORAGE_STATUS_NOT_AVAILABLE:
                return mavsdk::Camera::Status::StorageStatus::NotAvailable;
            case rpc::camera::Status_StorageStatus_STORAGE_STATUS_UNFORMATTED:
                return mavsdk::Camera::Status::StorageStatus::Unformatted;
            case rpc::camera::Status_StorageStatus_STORAGE_STATUS_FORMATTED:
                return mavsdk::Camera::Status::StorageStatus::Formatted;
        }
    }

    static std::unique_ptr<rpc::camera::Status>
    translateToRpcStatus(const mavsdk::Camera::Status& status)
    {
        std::unique_ptr<rpc::camera::Status> rpc_obj(new rpc::camera::Status());

        rpc_obj->set_video_on(status.video_on);

        rpc_obj->set_photo_interval_on(status.photo_interval_on);

        rpc_obj->set_used_storage_mib(status.used_storage_mib);

        rpc_obj->set_available_storage_mib(status.available_storage_mib);

        rpc_obj->set_total_storage_mib(status.total_storage_mib);

        rpc_obj->set_recording_time_s(status.recording_time_s);

        rpc_obj->set_media_folder_name(status.media_folder_name);

        rpc_obj->set_storage_status(translateToRpcStorageStatus(status.storage_status));

        return rpc_obj;
    }

    static mavsdk::Camera::Status translateFromRpcStatus(const rpc::camera::Status& status)
    {
        mavsdk::Camera::Status obj;

        obj.video_on = status.video_on();

        obj.photo_interval_on = status.photo_interval_on();

        obj.used_storage_mib = status.used_storage_mib();

        obj.available_storage_mib = status.available_storage_mib();

        obj.total_storage_mib = status.total_storage_mib();

        obj.recording_time_s = status.recording_time_s();

        obj.media_folder_name = status.media_folder_name();

        obj.storage_status = translateFromRpcStorageStatus(status.storage_status());

        return obj;
    }

    static std::unique_ptr<rpc::camera::Option>
    translateToRpcOption(const mavsdk::Camera::Option& option)
    {
        std::unique_ptr<rpc::camera::Option> rpc_obj(new rpc::camera::Option());

        rpc_obj->set_option_id(option.option_id);

        rpc_obj->set_option_description(option.option_description);

        return rpc_obj;
    }

    static mavsdk::Camera::Option translateFromRpcOption(const rpc::camera::Option& option)
    {
        mavsdk::Camera::Option obj;

        obj.option_id = option.option_id();

        obj.option_description = option.option_description();

        return obj;
    }

    static std::unique_ptr<rpc::camera::Setting>
    translateToRpcSetting(const mavsdk::Camera::Setting& setting)
    {
        std::unique_ptr<rpc::camera::Setting> rpc_obj(new rpc::camera::Setting());

        rpc_obj->set_setting_id(setting.setting_id);

        rpc_obj->set_setting_description(setting.setting_description);

        rpc_obj->set_allocated_option(translateToRpcOption(setting.option).release());

        rpc_obj->set_is_range(setting.is_range);

        return rpc_obj;
    }

    static mavsdk::Camera::Setting translateFromRpcSetting(const rpc::camera::Setting& setting)
    {
        mavsdk::Camera::Setting obj;

        obj.setting_id = setting.setting_id();

        obj.setting_description = setting.setting_description();

        obj.option = translateFromRpcOption(setting.option());

        obj.is_range = setting.is_range();

        return obj;
    }

    static std::unique_ptr<rpc::camera::SettingOptions>
    translateToRpcSettingOptions(const mavsdk::Camera::SettingOptions& setting_options)
    {
        std::unique_ptr<rpc::camera::SettingOptions> rpc_obj(new rpc::camera::SettingOptions());

        rpc_obj->set_setting_id(setting_options.setting_id);

        rpc_obj->set_setting_description(setting_options.setting_description);

        for (const auto& elem : setting_options.options) {
            auto* ptr = rpc_obj->add_options();
            ptr->CopyFrom(*translateToRpcOption(elem).release());
        }

        rpc_obj->set_is_range(setting_options.is_range);

        return rpc_obj;
    }

    static mavsdk::Camera::SettingOptions
    translateFromRpcSettingOptions(const rpc::camera::SettingOptions& setting_options)
    {
        mavsdk::Camera::SettingOptions obj;

        obj.setting_id = setting_options.setting_id();

        obj.setting_description = setting_options.setting_description();

        for (const auto& elem : setting_options.options()) {
            obj.options.push_back(translateFromRpcOption(elem));
        }

        obj.is_range = setting_options.is_range();

        return obj;
    }

    static std::unique_ptr<rpc::camera::Information>
    translateToRpcInformation(const mavsdk::Camera::Information& information)
    {
        std::unique_ptr<rpc::camera::Information> rpc_obj(new rpc::camera::Information());

        rpc_obj->set_vendor_name(information.vendor_name);

        rpc_obj->set_model_name(information.model_name);

        return rpc_obj;
    }

    static mavsdk::Camera::Information
    translateFromRpcInformation(const rpc::camera::Information& information)
    {
        mavsdk::Camera::Information obj;

        obj.vendor_name = information.vendor_name();

        obj.model_name = information.model_name();

        return obj;
    }

    grpc::Status TakePhoto(
        grpc::ServerContext* /* context */,
        const rpc::camera::TakePhotoRequest* /* request */,
        rpc::camera::TakePhotoResponse* response) override
    {
        auto result = _camera.take_photo();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status StartPhotoInterval(
        grpc::ServerContext* /* context */,
        const rpc::camera::StartPhotoIntervalRequest* request,
        rpc::camera::StartPhotoIntervalResponse* response) override
    {
        if (request == nullptr) {
            LogWarn() << "StartPhotoInterval sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result = _camera.start_photo_interval(request->interval_s());

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status StopPhotoInterval(
        grpc::ServerContext* /* context */,
        const rpc::camera::StopPhotoIntervalRequest* /* request */,
        rpc::camera::StopPhotoIntervalResponse* response) override
    {
        auto result = _camera.stop_photo_interval();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status StartVideo(
        grpc::ServerContext* /* context */,
        const rpc::camera::StartVideoRequest* /* request */,
        rpc::camera::StartVideoResponse* response) override
    {
        auto result = _camera.start_video();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status StopVideo(
        grpc::ServerContext* /* context */,
        const rpc::camera::StopVideoRequest* /* request */,
        rpc::camera::StopVideoResponse* response) override
    {
        auto result = _camera.stop_video();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status StartVideoStreaming(
        grpc::ServerContext* /* context */,
        const rpc::camera::StartVideoStreamingRequest* /* request */,
        rpc::camera::StartVideoStreamingResponse* response) override
    {
        auto result = _camera.start_video_streaming();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status StopVideoStreaming(
        grpc::ServerContext* /* context */,
        const rpc::camera::StopVideoStreamingRequest* /* request */,
        rpc::camera::StopVideoStreamingResponse* response) override
    {
        auto result = _camera.stop_video_streaming();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status SetMode(
        grpc::ServerContext* /* context */,
        const rpc::camera::SetModeRequest* request,
        rpc::camera::SetModeResponse* response) override
    {
        if (request == nullptr) {
            LogWarn() << "SetMode sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result = _camera.set_mode(translateFromRpcMode(request->mode()));

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status SubscribeMode(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::camera::SubscribeModeRequest* /* request */,
        grpc::ServerWriter<rpc::camera::ModeResponse>* writer) override
    {
        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);

        std::mutex subscribe_mutex{};

        _camera.mode_async([this, &writer, &stream_closed_promise, is_finished, &subscribe_mutex](
                               const mavsdk::Camera::Mode mode) {
            rpc::camera::ModeResponse rpc_response;

            rpc_response.set_mode(translateToRpcMode(mode));

            std::lock_guard<std::mutex> lock(subscribe_mutex);
            if (!*is_finished && !writer->Write(rpc_response)) {
                _camera.mode_async(nullptr);
                *is_finished = true;
                unregister_stream_stop_promise(stream_closed_promise);
                stream_closed_promise->set_value();
            }
        });

        stream_closed_future.wait();
        return grpc::Status::OK;
    }

    grpc::Status SubscribeInformation(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::camera::SubscribeInformationRequest* /* request */,
        grpc::ServerWriter<rpc::camera::InformationResponse>* writer) override
    {
        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);

        std::mutex subscribe_mutex{};

        _camera.information_async(
            [this, &writer, &stream_closed_promise, is_finished, &subscribe_mutex](
                const mavsdk::Camera::Information information) {
                rpc::camera::InformationResponse rpc_response;

                rpc_response.set_allocated_information(
                    translateToRpcInformation(information).release());

                std::lock_guard<std::mutex> lock(subscribe_mutex);
                if (!*is_finished && !writer->Write(rpc_response)) {
                    _camera.information_async(nullptr);
                    *is_finished = true;
                    unregister_stream_stop_promise(stream_closed_promise);
                    stream_closed_promise->set_value();
                }
            });

        stream_closed_future.wait();
        return grpc::Status::OK;
    }

    grpc::Status SubscribeVideoStreamInfo(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::camera::SubscribeVideoStreamInfoRequest* /* request */,
        grpc::ServerWriter<rpc::camera::VideoStreamInfoResponse>* writer) override
    {
        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);

        std::mutex subscribe_mutex{};

        _camera.video_stream_info_async(
            [this, &writer, &stream_closed_promise, is_finished, &subscribe_mutex](
                const mavsdk::Camera::VideoStreamInfo video_stream_info) {
                rpc::camera::VideoStreamInfoResponse rpc_response;

                rpc_response.set_allocated_video_stream_info(
                    translateToRpcVideoStreamInfo(video_stream_info).release());

                std::lock_guard<std::mutex> lock(subscribe_mutex);
                if (!*is_finished && !writer->Write(rpc_response)) {
                    _camera.video_stream_info_async(nullptr);
                    *is_finished = true;
                    unregister_stream_stop_promise(stream_closed_promise);
                    stream_closed_promise->set_value();
                }
            });

        stream_closed_future.wait();
        return grpc::Status::OK;
    }

    grpc::Status SubscribeCaptureInfo(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::camera::SubscribeCaptureInfoRequest* /* request */,
        grpc::ServerWriter<rpc::camera::CaptureInfoResponse>* writer) override
    {
        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);

        std::mutex subscribe_mutex{};

        _camera.capture_info_async(
            [this, &writer, &stream_closed_promise, is_finished, &subscribe_mutex](
                const mavsdk::Camera::CaptureInfo capture_info) {
                rpc::camera::CaptureInfoResponse rpc_response;

                rpc_response.set_allocated_capture_info(
                    translateToRpcCaptureInfo(capture_info).release());

                std::lock_guard<std::mutex> lock(subscribe_mutex);
                if (!*is_finished && !writer->Write(rpc_response)) {
                    _camera.capture_info_async(nullptr);
                    *is_finished = true;
                    unregister_stream_stop_promise(stream_closed_promise);
                    stream_closed_promise->set_value();
                }
            });

        stream_closed_future.wait();
        return grpc::Status::OK;
    }

    grpc::Status SubscribeStatus(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::camera::SubscribeStatusRequest* /* request */,
        grpc::ServerWriter<rpc::camera::StatusResponse>* writer) override
    {
        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);

        std::mutex subscribe_mutex{};

        _camera.status_async([this, &writer, &stream_closed_promise, is_finished, &subscribe_mutex](
                                 const mavsdk::Camera::Status status) {
            rpc::camera::StatusResponse rpc_response;

            rpc_response.set_allocated_camera_status(translateToRpcStatus(status).release());

            std::lock_guard<std::mutex> lock(subscribe_mutex);
            if (!*is_finished && !writer->Write(rpc_response)) {
                _camera.status_async(nullptr);
                *is_finished = true;
                unregister_stream_stop_promise(stream_closed_promise);
                stream_closed_promise->set_value();
            }
        });

        stream_closed_future.wait();
        return grpc::Status::OK;
    }

    grpc::Status SubscribeCurrentSettings(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::camera::SubscribeCurrentSettingsRequest* /* request */,
        grpc::ServerWriter<rpc::camera::CurrentSettingsResponse>* writer) override
    {
        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);

        std::mutex subscribe_mutex{};

        _camera.current_settings_async(
            [this, &writer, &stream_closed_promise, is_finished, &subscribe_mutex](
                const std::vector<mavsdk::Camera::Setting> current_settings) {
                rpc::camera::CurrentSettingsResponse rpc_response;

                for (const auto& elem : current_settings) {
                    auto* ptr = rpc_response.add_current_settings();
                    ptr->CopyFrom(*translateToRpcSetting(elem).release());
                }

                std::lock_guard<std::mutex> lock(subscribe_mutex);
                if (!*is_finished && !writer->Write(rpc_response)) {
                    _camera.current_settings_async(nullptr);
                    *is_finished = true;
                    unregister_stream_stop_promise(stream_closed_promise);
                    stream_closed_promise->set_value();
                }
            });

        stream_closed_future.wait();
        return grpc::Status::OK;
    }

    grpc::Status SubscribePossibleSettingOptions(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::camera::SubscribePossibleSettingOptionsRequest* /* request */,
        grpc::ServerWriter<rpc::camera::PossibleSettingOptionsResponse>* writer) override
    {
        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);

        std::mutex subscribe_mutex{};

        _camera.possible_setting_options_async(
            [this, &writer, &stream_closed_promise, is_finished, &subscribe_mutex](
                const std::vector<mavsdk::Camera::SettingOptions> possible_setting_options) {
                rpc::camera::PossibleSettingOptionsResponse rpc_response;

                for (const auto& elem : possible_setting_options) {
                    auto* ptr = rpc_response.add_setting_options();
                    ptr->CopyFrom(*translateToRpcSettingOptions(elem).release());
                }

                std::lock_guard<std::mutex> lock(subscribe_mutex);
                if (!*is_finished && !writer->Write(rpc_response)) {
                    _camera.possible_setting_options_async(nullptr);
                    *is_finished = true;
                    unregister_stream_stop_promise(stream_closed_promise);
                    stream_closed_promise->set_value();
                }
            });

        stream_closed_future.wait();
        return grpc::Status::OK;
    }

    grpc::Status SetSetting(
        grpc::ServerContext* /* context */,
        const rpc::camera::SetSettingRequest* request,
        rpc::camera::SetSettingResponse* response) override
    {
        if (request == nullptr) {
            LogWarn() << "SetSetting sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result = _camera.set_setting(translateFromRpcSetting(request->setting()));

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status GetSetting(
        grpc::ServerContext* /* context */,
        const rpc::camera::GetSettingRequest* request,
        rpc::camera::GetSettingResponse* response) override
    {
        if (request == nullptr) {
            LogWarn() << "GetSetting sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result_pair = _camera.get_setting(translateFromRpcSetting(request->setting()));

        if (response != nullptr) {
            fillResponseWithResult(response, result_pair.first);
            response->set_allocated_setting(translateToRpcSetting(result_pair.second).release());
        }

        return grpc::Status::OK;
    }

    grpc::Status FormatStorage(
        grpc::ServerContext* /* context */,
        const rpc::camera::FormatStorageRequest* /* request */,
        rpc::camera::FormatStorageResponse* response) override
    {
        auto result = _camera.format_storage();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    void stop()
    {
        _stopped.store(true);
        for (auto& prom : _stream_stop_promises) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        }
    }

private:
    void register_stream_stop_promise(std::weak_ptr<std::promise<void>> prom)
    {
        // If we have already stopped, set promise immediately and don't add it to list.
        if (_stopped.load()) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        } else {
            _stream_stop_promises.push_back(prom);
        }
    }

    void unregister_stream_stop_promise(std::shared_ptr<std::promise<void>> prom)
    {
        for (auto it = _stream_stop_promises.begin(); it != _stream_stop_promises.end();
             /* ++it */) {
            if (it->lock() == prom) {
                it = _stream_stop_promises.erase(it);
            } else {
                ++it;
            }
        }
    }

    Camera& _camera;
    std::atomic<bool> _stopped{false};
    std::vector<std::weak_ptr<std::promise<void>>> _stream_stop_promises{};
};

} // namespace backend
} // namespace mavsdk