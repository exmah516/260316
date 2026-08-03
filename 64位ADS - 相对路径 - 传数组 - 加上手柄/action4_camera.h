#pragma once

#include <cstdint>
#include <memory>
#include <string>

enum class Action4CameraState : int
{
	Closed = 0,
	Opening = 1,
	Previewing = 2,
	Recording = 3,
	Missing = 4,
	Disconnected = 5,
	Error = 6
};

enum class Action4InputFormat : int
{
	Unknown = 0,
	H264 = 1,
	Mjpeg = 2,
	Rgb32 = 3
};

struct Action4CameraSnapshot
{
	Action4CameraState state = Action4CameraState::Closed;
	Action4InputFormat input_format = Action4InputFormat::Unknown;
	int width = 0;
	int height = 0;
	int fps_numerator = 0;
	int fps_denominator = 1;
	bool preview_enabled = false;
	bool recording = false;
	std::uint64_t recording_elapsed_us = 0;
	std::uint64_t frame_count = 0;
	std::uint64_t dropped_frames = 0;
	std::uint64_t timing_rows_dropped = 0;
	bool timing_writer_error = false;
	int error_code = 0;
};

// Action 4 的 UVC 采集、MP4 封装和共享内存预览均由该后台对象独占。
class Action4CameraRecorder
{
public:
	Action4CameraRecorder();
	~Action4CameraRecorder();

	Action4CameraRecorder(const Action4CameraRecorder&) = delete;
	Action4CameraRecorder& operator=(const Action4CameraRecorder&) = delete;

	void set_preview_enabled(bool enabled);
	bool start_recording(
		const std::wstring& video_path,
		const std::wstring& frame_timing_path,
		std::int64_t session_anchor_qpc,
		std::int64_t qpc_frequency);
	void stop_recording_and_wait();
	bool timing_writer_failed() const;
	Action4CameraSnapshot snapshot() const;
	void shutdown();

private:
	class Impl;
	std::unique_ptr<Impl> impl_;
};
