#include "action4_camera.h"

#include "async_csv_writer.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cwctype>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <mfapi.h>
#include <mferror.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <wrl/client.h>

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")

using Microsoft::WRL::ComPtr;

namespace
{
	constexpr wchar_t kPreviewMappingName[] = L"Local\\ADS_Action4_Preview_v1";
	constexpr wchar_t kPreviewEventName[] = L"Local\\ADS_Action4_PreviewEvent_v1";
	constexpr std::uint32_t kPreviewMagic = 0x56503441; // "A4PV"
	constexpr std::uint32_t kPreviewVersion = 1;
	constexpr int kPreviewWidth = 1280;
	constexpr int kPreviewHeight = 720;
	constexpr int kPreviewStride = kPreviewWidth * 4;
	constexpr std::size_t kPreviewBytes = static_cast<std::size_t>(kPreviewStride) * kPreviewHeight;
	constexpr LONGLONG kFrameDuration100ns = 10000000LL / 30LL;
	constexpr DWORD kVideoBitrate = 20000000;

#pragma pack(push, 1)
	struct PreviewSharedHeader
	{
		std::uint32_t magic;
		std::uint32_t version;
		std::uint32_t header_size;
		std::uint32_t width;
		std::uint32_t height;
		std::uint32_t stride;
		std::uint32_t pixel_format; // 1 = BGRA32
		volatile LONG active_index;
		volatile LONG64 frame_sequence;
		std::int64_t elapsed_us;
		std::uint8_t reserved[16];
	};
#pragma pack(pop)
	static_assert(sizeof(PreviewSharedHeader) == 64, "Action 4 预览共享内存头布局发生变化。");

	struct VideoFrameTimingRow
	{
		std::uint64_t frame_index = 0;
		std::int64_t source_timestamp_100ns = 0;
		std::uint64_t callback_elapsed_us = 0;
		bool keyframe = false;
		bool discontinuity = false;
	};

	bool write_frame_timing(std::FILE* fp, const VideoFrameTimingRow& row)
	{
		return std::fprintf(
			fp,
			"%llu,%lld,%llu,%d,%d\n",
			static_cast<unsigned long long>(row.frame_index),
			static_cast<long long>(row.source_timestamp_100ns),
			static_cast<unsigned long long>(row.callback_elapsed_us),
			row.keyframe ? 1 : 0,
			row.discontinuity ? 1 : 0) >= 0;
	}

	std::wstring to_lower(std::wstring value)
	{
		std::transform(value.begin(), value.end(), value.begin(), [](wchar_t ch) {
			return static_cast<wchar_t>(std::towlower(ch));
		});
		return value;
	}

	std::uint64_t qpc_elapsed_us(std::int64_t now, std::int64_t anchor, std::int64_t frequency)
	{
		if (frequency <= 0 || anchor <= 0 || now <= anchor)
		{
			return 0;
		}
		return static_cast<std::uint64_t>(((now - anchor) * 1000000LL) / frequency);
	}

	std::int64_t current_qpc()
	{
		LARGE_INTEGER now{};
		QueryPerformanceCounter(&now);
		return now.QuadPart;
	}

	int hresult_code(HRESULT hr)
	{
		return static_cast<int>(hr);
	}

	struct NativeCandidate
	{
		ComPtr<IMFMediaType> type;
		GUID subtype = GUID_NULL;
		UINT32 width = 0;
		UINT32 height = 0;
		UINT32 fps_numerator = 0;
		UINT32 fps_denominator = 1;
		int rank = 100;
	};
}

class Action4CameraRecorder::Impl
{
public:
	Impl()
	{
		initialize_preview_mapping();
		worker_ = std::thread(&Impl::worker_loop, this);
	}

	~Impl()
	{
		shutdown();
	}

	void set_preview_enabled(bool enabled)
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (preview_requested_ == enabled)
			{
				return;
			}
			preview_requested_ = enabled;
			++request_revision_;
			snapshot_.preview_enabled = enabled;
		}
		condition_.notify_all();
	}

	bool start_recording(
		const std::wstring& video_path,
		const std::wstring& frame_timing_path,
		std::int64_t session_anchor_qpc,
		std::int64_t qpc_frequency)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (shutdown_requested_ || recording_requested_ || recording_active_ || finalizing_)
		{
			return false;
		}
		video_path_ = video_path;
		frame_timing_path_ = frame_timing_path;
		frame_writer_.reset_status();
		session_anchor_qpc_ = session_anchor_qpc;
		qpc_frequency_ = qpc_frequency > 0 ? qpc_frequency : 1;
		recording_requested_ = true;
		++recording_generation_;
		++request_revision_;
		snapshot_.frame_count = 0;
		snapshot_.dropped_frames = 0;
		snapshot_.recording_elapsed_us = 0;
		snapshot_.error_code = 0;
		condition_.notify_all();
		return true;
	}

	void stop_recording_and_wait()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if (recording_requested_)
		{
			recording_requested_ = false;
			++request_revision_;
			condition_.notify_all();
		}
		condition_.wait(lock, [&]() {
			return !recording_starting_ && !recording_active_ && !finalizing_;
		});
	}

	bool timing_writer_failed() const
	{
		return frame_writer_.has_error();
	}

	Action4CameraSnapshot snapshot() const
	{
		std::lock_guard<std::mutex> lock(mutex_);
		Action4CameraSnapshot copy = snapshot_;
		copy.timing_rows_dropped = frame_writer_.dropped_count();
		copy.timing_writer_error = frame_writer_.has_error();
		if (recording_active_)
		{
			copy.recording_elapsed_us = qpc_elapsed_us(current_qpc(), recording_start_qpc_, qpc_frequency_);
		}
		return copy;
	}

	void shutdown()
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (shutdown_requested_)
			{
				return;
			}
			shutdown_requested_ = true;
			recording_requested_ = false;
			preview_requested_ = false;
			++request_revision_;
		}
		condition_.notify_all();
		if (worker_.joinable())
		{
			worker_.join();
		}
		close_preview_mapping();
	}

private:
	void initialize_preview_mapping()
	{
		const std::size_t total_bytes = sizeof(PreviewSharedHeader) + kPreviewBytes * 2;
		preview_mapping_ = CreateFileMappingW(
			INVALID_HANDLE_VALUE,
			nullptr,
			PAGE_READWRITE,
			0,
			static_cast<DWORD>(total_bytes),
			kPreviewMappingName);
		if (preview_mapping_ == nullptr)
		{
			return;
		}
		preview_view_ = static_cast<std::uint8_t*>(MapViewOfFile(preview_mapping_, FILE_MAP_ALL_ACCESS, 0, 0, total_bytes));
		if (preview_view_ == nullptr)
		{
			CloseHandle(preview_mapping_);
			preview_mapping_ = nullptr;
			return;
		}
		preview_event_ = CreateEventW(nullptr, FALSE, FALSE, kPreviewEventName);
		preview_header_ = reinterpret_cast<PreviewSharedHeader*>(preview_view_);
		std::memset(preview_view_, 0, total_bytes);
		preview_header_->magic = kPreviewMagic;
		preview_header_->version = kPreviewVersion;
		preview_header_->header_size = sizeof(PreviewSharedHeader);
		preview_header_->width = kPreviewWidth;
		preview_header_->height = kPreviewHeight;
		preview_header_->stride = kPreviewStride;
		preview_header_->pixel_format = 1;
	}

	void close_preview_mapping()
	{
		if (preview_view_ != nullptr)
		{
			UnmapViewOfFile(preview_view_);
			preview_view_ = nullptr;
			preview_header_ = nullptr;
		}
		if (preview_mapping_ != nullptr)
		{
			CloseHandle(preview_mapping_);
			preview_mapping_ = nullptr;
		}
		if (preview_event_ != nullptr)
		{
			CloseHandle(preview_event_);
			preview_event_ = nullptr;
		}
	}

	HRESULT activate_action4(ComPtr<IMFMediaSource>& source, bool& missing)
	{
		missing = false;
		ComPtr<IMFAttributes> attributes;
		HRESULT hr = MFCreateAttributes(&attributes, 1);
		if (FAILED(hr)) return hr;
		hr = attributes->SetGUID(
			MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
			MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
		if (FAILED(hr)) return hr;

		IMFActivate** devices = nullptr;
		UINT32 device_count = 0;
		hr = MFEnumDeviceSources(attributes.Get(), &devices, &device_count);
		if (FAILED(hr)) return hr;

		IMFActivate* name_match = nullptr;
		IMFActivate* id_match = nullptr;
		for (UINT32 i = 0; i < device_count; ++i)
		{
			wchar_t* friendly_raw = nullptr;
			UINT32 friendly_length = 0;
			wchar_t* link_raw = nullptr;
			UINT32 link_length = 0;
			devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &friendly_raw, &friendly_length);
			devices[i]->GetAllocatedString(
				MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,
				&link_raw,
				&link_length);
			const std::wstring friendly = friendly_raw != nullptr ? friendly_raw : L"";
			const std::wstring link_lower = to_lower(link_raw != nullptr ? link_raw : L"");
			if (link_lower.find(L"vid_2ca3&pid_0023") != std::wstring::npos)
			{
				id_match = devices[i];
			}
			if (friendly == L"OsmoAction4" || to_lower(friendly).find(L"osmoaction4") != std::wstring::npos)
			{
				name_match = devices[i];
			}
			CoTaskMemFree(friendly_raw);
			CoTaskMemFree(link_raw);
		}

		IMFActivate* selected = id_match != nullptr ? id_match : name_match;
		if (selected != nullptr)
		{
			hr = selected->ActivateObject(IID_PPV_ARGS(&source));
		}
		else
		{
			missing = true;
			hr = MF_E_NOT_FOUND;
		}

		for (UINT32 i = 0; i < device_count; ++i)
		{
			devices[i]->Release();
		}
		CoTaskMemFree(devices);
		return hr;
	}

	HRESULT collect_candidates(IMFSourceReader* reader, std::vector<NativeCandidate>& candidates)
	{
		for (DWORD type_index = 0;; ++type_index)
		{
			ComPtr<IMFMediaType> type;
			const HRESULT hr = reader->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, type_index, &type);
			if (hr == MF_E_NO_MORE_TYPES)
			{
				break;
			}
			if (FAILED(hr))
			{
				return hr;
			}

			GUID subtype = GUID_NULL;
			UINT32 width = 0;
			UINT32 height = 0;
			UINT32 fps_numerator = 0;
			UINT32 fps_denominator = 1;
			if (FAILED(type->GetGUID(MF_MT_SUBTYPE, &subtype)) ||
				FAILED(MFGetAttributeSize(type.Get(), MF_MT_FRAME_SIZE, &width, &height)) ||
				FAILED(MFGetAttributeRatio(type.Get(), MF_MT_FRAME_RATE, &fps_numerator, &fps_denominator)) ||
				fps_denominator == 0)
			{
				continue;
			}
			const double fps = static_cast<double>(fps_numerator) / fps_denominator;
			if (std::abs(fps - 30.0) > 0.6)
			{
				continue;
			}

			int resolution_rank = 100;
			int format_rank = 100;
			if (width == 1920 && height == 1080)
			{
				resolution_rank = 0;
				if (subtype == MFVideoFormat_H264) format_rank = 0;
				else if (subtype == MFVideoFormat_MJPG) format_rank = 1;
				else continue;
			}
			else if (width == 1280 && height == 720)
			{
				resolution_rank = 10;
				if (subtype == MFVideoFormat_H264) format_rank = 0;
				else if (subtype == MFVideoFormat_MJPG) format_rank = 1;
				else format_rank = 2;
			}
			else continue;

			if (subtype == MFVideoFormat_H264)
			{
				UINT32 sequence_header_size = 0;
				if (FAILED(type->GetBlobSize(MF_MT_MPEG_SEQUENCE_HEADER, &sequence_header_size)) ||
					sequence_header_size == 0)
				{
					continue;
				}
			}

			NativeCandidate candidate;
			candidate.type = type;
			candidate.subtype = subtype;
			candidate.width = width;
			candidate.height = height;
			candidate.fps_numerator = fps_numerator;
			candidate.fps_denominator = fps_denominator;
			candidate.rank = resolution_rank + format_rank;
			candidates.push_back(candidate);
		}
		std::sort(candidates.begin(), candidates.end(), [](const NativeCandidate& left, const NativeCandidate& right) {
			return left.rank < right.rank;
		});
		return candidates.empty() ? MF_E_INVALIDMEDIATYPE : S_OK;
	}

	HRESULT configure_reader(const NativeCandidate& candidate)
	{
		reader_->Flush(MF_SOURCE_READER_FIRST_VIDEO_STREAM);
		HRESULT hr = reader_->SetCurrentMediaType(
			MF_SOURCE_READER_FIRST_VIDEO_STREAM,
			nullptr,
			candidate.type.Get());
		if (FAILED(hr)) return hr;

		ComPtr<IMFMediaType> decoded_type;
		hr = MFCreateMediaType(&decoded_type);
		if (FAILED(hr)) return hr;
		decoded_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
		decoded_type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);
		decoded_type->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
		MFSetAttributeSize(decoded_type.Get(), MF_MT_FRAME_SIZE, candidate.width, candidate.height);
		MFSetAttributeRatio(
			decoded_type.Get(),
			MF_MT_FRAME_RATE,
			candidate.fps_numerator,
			candidate.fps_denominator);
		MFSetAttributeRatio(decoded_type.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
		hr = reader_->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, decoded_type.Get());
		if (FAILED(hr)) return hr;

		ComPtr<IMFMediaType> actual_type;
		hr = reader_->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, &actual_type);
		if (FAILED(hr)) return hr;
		UINT32 actual_width = 0;
		UINT32 actual_height = 0;
		UINT32 actual_fps_numerator = 0;
		UINT32 actual_fps_denominator = 1;
		MFGetAttributeSize(actual_type.Get(), MF_MT_FRAME_SIZE, &actual_width, &actual_height);
		MFGetAttributeRatio(actual_type.Get(), MF_MT_FRAME_RATE, &actual_fps_numerator, &actual_fps_denominator);
		UINT32 stride_value = 0;
		if (FAILED(actual_type->GetUINT32(MF_MT_DEFAULT_STRIDE, &stride_value)))
		{
			LONG calculated_stride = 0;
			if (SUCCEEDED(MFGetStrideForBitmapInfoHeader(MFVideoFormat_RGB32.Data1, actual_width, &calculated_stride)))
			{
				stride_value = static_cast<UINT32>(calculated_stride);
			}
			else
			{
				stride_value = actual_width * 4;
			}
		}

		capture_width_ = static_cast<int>(actual_width);
		capture_height_ = static_cast<int>(actual_height);
		capture_stride_ = static_cast<LONG>(stride_value);
		capture_fps_numerator_ = static_cast<int>(actual_fps_numerator);
		capture_fps_denominator_ = actual_fps_denominator > 0 ? static_cast<int>(actual_fps_denominator) : 1;
		selected_input_format_ = candidate.subtype == MFVideoFormat_H264
			? Action4InputFormat::H264
			: (candidate.subtype == MFVideoFormat_MJPG ? Action4InputFormat::Mjpeg : Action4InputFormat::Unknown);
		return S_OK;
	}

	HRESULT prime_reader(const NativeCandidate& candidate)
	{
		const int maximum_attempts = candidate.subtype == MFVideoFormat_H264 ? 60 : 10;
		for (int attempt = 0; attempt < maximum_attempts; ++attempt)
		{
			DWORD actual_stream = 0;
			DWORD stream_flags = 0;
			LONGLONG timestamp = 0;
			ComPtr<IMFSample> sample;
			const HRESULT hr = reader_->ReadSample(
				MF_SOURCE_READER_FIRST_VIDEO_STREAM,
				0,
				&actual_stream,
				&stream_flags,
				&timestamp,
				&sample);
			if (FAILED(hr) || (stream_flags & MF_SOURCE_READERF_ERROR) != 0 ||
				(stream_flags & MF_SOURCE_READERF_ENDOFSTREAM) != 0)
			{
				return FAILED(hr) ? hr : MF_E_END_OF_STREAM;
			}
			if (!sample)
			{
				continue;
			}
			if (candidate.subtype == MFVideoFormat_H264)
			{
				UINT32 clean_point = FALSE;
				sample->GetUINT32(MFSampleExtension_CleanPoint, &clean_point);
				if (clean_point == FALSE)
				{
					continue;
				}
			}
			primed_sample_ = sample;
			primed_timestamp_ = timestamp;
			primed_stream_flags_ = stream_flags;
			return S_OK;
		}
		return MF_E_INVALID_STREAM_DATA;
	}

	HRESULT open_capture(bool& missing)
	{
		missing = false;
		close_capture();
		HRESULT hr = activate_action4(source_, missing);
		if (FAILED(hr)) return hr;

		ComPtr<IMFAttributes> reader_attributes;
		hr = MFCreateAttributes(&reader_attributes, 4);
		if (FAILED(hr)) return hr;
		reader_attributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, TRUE);
		reader_attributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE);
		reader_attributes->SetUINT32(MF_LOW_LATENCY, TRUE);
		hr = MFCreateSourceReaderFromMediaSource(source_.Get(), reader_attributes.Get(), &reader_);
		if (FAILED(hr)) return hr;

		std::vector<NativeCandidate> candidates;
		hr = collect_candidates(reader_.Get(), candidates);
		if (FAILED(hr)) return hr;
		HRESULT last_error = MF_E_INVALIDMEDIATYPE;
		for (const NativeCandidate& candidate : candidates)
		{
			last_error = configure_reader(candidate);
			if (SUCCEEDED(last_error))
			{
				last_error = prime_reader(candidate);
			}
			if (SUCCEEDED(last_error))
			{
				std::lock_guard<std::mutex> lock(mutex_);
				snapshot_.input_format = selected_input_format_;
				snapshot_.width = capture_width_;
				snapshot_.height = capture_height_;
				snapshot_.fps_numerator = capture_fps_numerator_;
				snapshot_.fps_denominator = capture_fps_denominator_;
				return S_OK;
			}
		}
		return last_error;
	}

	void close_capture()
	{
		reader_.Reset();
		if (source_)
		{
			source_->Shutdown();
			source_.Reset();
		}
		capture_width_ = 0;
		capture_height_ = 0;
		capture_stride_ = 0;
		primed_sample_.Reset();
		primed_timestamp_ = 0;
		primed_stream_flags_ = 0;
	}

	HRESULT start_sink_writer()
	{
		if (capture_width_ <= 0 || capture_height_ <= 0)
		{
			return MF_E_INVALIDMEDIATYPE;
		}
		const char* frame_header =
			"frame_index,source_timestamp_100ns,callback_elapsed_us,keyframe,discontinuity\n";
		if (!frame_writer_.start(frame_timing_path_, frame_header, &write_frame_timing))
		{
			return HRESULT_FROM_WIN32(ERROR_OPEN_FAILED);
		}

		ComPtr<IMFAttributes> sink_attributes;
		HRESULT hr = MFCreateAttributes(&sink_attributes, 3);
		if (FAILED(hr)) return hr;
		sink_attributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE);
		sink_attributes->SetUINT32(MF_SINK_WRITER_DISABLE_THROTTLING, TRUE);
		hr = MFCreateSinkWriterFromURL(video_path_.c_str(), nullptr, sink_attributes.Get(), &sink_writer_);
		if (FAILED(hr))
		{
			frame_writer_.stop();
			return hr;
		}

		ComPtr<IMFMediaType> output_type;
		hr = MFCreateMediaType(&output_type);
		if (FAILED(hr)) return hr;
		output_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
		output_type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264);
		output_type->SetUINT32(MF_MT_AVG_BITRATE, kVideoBitrate);
		output_type->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
		MFSetAttributeSize(output_type.Get(), MF_MT_FRAME_SIZE, capture_width_, capture_height_);
		MFSetAttributeRatio(
			output_type.Get(),
			MF_MT_FRAME_RATE,
			capture_fps_numerator_ > 0 ? capture_fps_numerator_ : 30,
			capture_fps_denominator_ > 0 ? capture_fps_denominator_ : 1);
		MFSetAttributeRatio(output_type.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
		hr = sink_writer_->AddStream(output_type.Get(), &sink_stream_index_);
		if (FAILED(hr)) return hr;

		ComPtr<IMFMediaType> input_type;
		hr = MFCreateMediaType(&input_type);
		if (FAILED(hr)) return hr;
		input_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
		input_type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB32);
		input_type->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
		MFSetAttributeSize(input_type.Get(), MF_MT_FRAME_SIZE, capture_width_, capture_height_);
		MFSetAttributeRatio(
			input_type.Get(),
			MF_MT_FRAME_RATE,
			capture_fps_numerator_ > 0 ? capture_fps_numerator_ : 30,
			capture_fps_denominator_ > 0 ? capture_fps_denominator_ : 1);
		MFSetAttributeRatio(input_type.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
		hr = sink_writer_->SetInputMediaType(sink_stream_index_, input_type.Get(), nullptr);
		if (FAILED(hr)) return hr;
		hr = sink_writer_->BeginWriting();
		if (FAILED(hr)) return hr;

		first_source_timestamp_ = std::numeric_limits<LONGLONG>::min();
		last_source_timestamp_ = std::numeric_limits<LONGLONG>::min();
		frame_index_ = 0;
		recording_start_qpc_ = current_qpc();
		{
			std::lock_guard<std::mutex> lock(mutex_);
			recording_active_ = true;
			finalizing_ = false;
			snapshot_.recording = true;
			snapshot_.state = Action4CameraState::Recording;
			snapshot_.frame_count = 0;
			snapshot_.dropped_frames = 0;
		}
		condition_.notify_all();
		return S_OK;
	}

	void finish_sink_writer(bool preserve_disconnect_state)
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (!recording_active_ && !sink_writer_)
			{
				return;
			}
			finalizing_ = true;
			recording_active_ = false;
			snapshot_.recording = false;
		}
		HRESULT finalize_hr = S_OK;
		if (sink_writer_)
		{
			finalize_hr = sink_writer_->Finalize();
			sink_writer_.Reset();
		}
		frame_writer_.stop();

		if (frame_index_ == 0)
		{
			DeleteFileW(video_path_.c_str());
			DeleteFileW(frame_timing_path_.c_str());
		}
		{
			std::lock_guard<std::mutex> lock(mutex_);
			finalizing_ = false;
			snapshot_.recording_elapsed_us = qpc_elapsed_us(current_qpc(), recording_start_qpc_, qpc_frequency_);
			if (FAILED(finalize_hr) && !preserve_disconnect_state)
			{
				snapshot_.state = Action4CameraState::Error;
				snapshot_.error_code = hresult_code(finalize_hr);
			}
			else if (!preserve_disconnect_state)
			{
				snapshot_.state = preview_requested_ ? Action4CameraState::Previewing : Action4CameraState::Closed;
			}
		}
		condition_.notify_all();
	}

	void fail_recording_start(HRESULT hr)
	{
		sink_writer_.Reset();
		frame_writer_.stop();
		DeleteFileW(video_path_.c_str());
		DeleteFileW(frame_timing_path_.c_str());
		std::lock_guard<std::mutex> lock(mutex_);
		failed_recording_generation_ = recording_generation_;
		recording_active_ = false;
		finalizing_ = false;
		snapshot_.recording = false;
		snapshot_.state = Action4CameraState::Error;
		snapshot_.error_code = hresult_code(hr);
		condition_.notify_all();
	}

	void publish_preview(IMFSample* sample, std::uint64_t elapsed_us)
	{
		if (preview_header_ == nullptr || sample == nullptr || capture_width_ <= 0 || capture_height_ <= 0)
		{
			return;
		}
		ComPtr<IMFMediaBuffer> buffer;
		if (FAILED(sample->ConvertToContiguousBuffer(&buffer)))
		{
			return;
		}
		BYTE* source_data = nullptr;
		DWORD max_length = 0;
		DWORD current_length = 0;
		if (FAILED(buffer->Lock(&source_data, &max_length, &current_length)) || source_data == nullptr)
		{
			return;
		}

		LONG source_stride = capture_stride_ != 0 ? capture_stride_ : capture_width_ * 4;
		const std::size_t absolute_stride = static_cast<std::size_t>(source_stride < 0 ? -source_stride : source_stride);
		if (absolute_stride * static_cast<std::size_t>(capture_height_) > current_length)
		{
			buffer->Unlock();
			return;
		}
		const BYTE* first_row = source_stride < 0
			? source_data + absolute_stride * static_cast<std::size_t>(capture_height_ - 1)
			: source_data;

		const LONG active = preview_header_->active_index;
		const LONG next_index = active == 0 ? 1 : 0;
		BYTE* destination = preview_view_ + sizeof(PreviewSharedHeader) + kPreviewBytes * next_index;
		for (int y = 0; y < kPreviewHeight; ++y)
		{
			const int source_y = (y * capture_height_) / kPreviewHeight;
			const BYTE* source_row = source_stride < 0
				? first_row - absolute_stride * static_cast<std::size_t>(source_y)
				: first_row + absolute_stride * static_cast<std::size_t>(source_y);
			BYTE* destination_row = destination + static_cast<std::size_t>(y) * kPreviewStride;
			for (int x = 0; x < kPreviewWidth; ++x)
			{
				const int source_x = (x * capture_width_) / kPreviewWidth;
				const BYTE* pixel = source_row + source_x * 4;
				BYTE* output = destination_row + x * 4;
				output[0] = pixel[0];
				output[1] = pixel[1];
				output[2] = pixel[2];
				output[3] = 0xFF;
			}
		}
		buffer->Unlock();

		preview_header_->elapsed_us = static_cast<std::int64_t>(elapsed_us);
		MemoryBarrier();
		InterlockedExchange(&preview_header_->active_index, next_index);
		InterlockedIncrement64(&preview_header_->frame_sequence);
		if (preview_event_ != nullptr)
		{
			SetEvent(preview_event_);
		}
	}

	HRESULT process_sample(IMFSample* sample, LONGLONG source_timestamp, DWORD stream_flags)
	{
		const std::int64_t callback_qpc = current_qpc();
		const std::uint64_t callback_elapsed_us = qpc_elapsed_us(callback_qpc, session_anchor_qpc_, qpc_frequency_);
		bool preview_enabled = false;
		bool recording_active = false;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			preview_enabled = preview_requested_;
			recording_active = recording_active_;
		}
		if (preview_enabled)
		{
			publish_preview(sample, callback_elapsed_us);
		}

		if (!recording_active || !sink_writer_)
		{
			return S_OK;
		}
		if (first_source_timestamp_ == std::numeric_limits<LONGLONG>::min())
		{
			first_source_timestamp_ = source_timestamp;
		}
		const LONGLONG normalized_timestamp = std::max<LONGLONG>(0, source_timestamp - first_source_timestamp_);
		sample->SetSampleTime(normalized_timestamp);
		sample->SetSampleDuration(kFrameDuration100ns);
		HRESULT hr = sink_writer_->WriteSample(sink_stream_index_, sample);
		if (FAILED(hr))
		{
			return hr;
		}

		UINT32 clean_point = FALSE;
		UINT32 sample_discontinuity = FALSE;
		sample->GetUINT32(MFSampleExtension_CleanPoint, &clean_point);
		sample->GetUINT32(MFSampleExtension_Discontinuity, &sample_discontinuity);
		const bool discontinuity = sample_discontinuity != FALSE ||
			(stream_flags & MF_SOURCE_READERF_STREAMTICK) != 0;
		if (last_source_timestamp_ != std::numeric_limits<LONGLONG>::min())
		{
			const LONGLONG delta = source_timestamp - last_source_timestamp_;
			if (delta > (kFrameDuration100ns * 3) / 2)
			{
				const std::uint64_t missed = static_cast<std::uint64_t>(delta / kFrameDuration100ns) - 1;
				std::lock_guard<std::mutex> lock(mutex_);
				snapshot_.dropped_frames += missed;
			}
		}
		last_source_timestamp_ = source_timestamp;

		VideoFrameTimingRow timing;
		timing.frame_index = frame_index_;
		timing.source_timestamp_100ns = source_timestamp;
		timing.callback_elapsed_us = callback_elapsed_us;
		timing.keyframe = clean_point != FALSE;
		timing.discontinuity = discontinuity;
		if (!frame_writer_.try_enqueue(timing) && frame_writer_.has_error())
		{
			return HRESULT_FROM_WIN32(ERROR_WRITE_FAULT);
		}
		++frame_index_;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			snapshot_.frame_count = frame_index_;
			snapshot_.recording_elapsed_us = qpc_elapsed_us(callback_qpc, recording_start_qpc_, qpc_frequency_);
		}
		return S_OK;
	}

	void handle_disconnect(HRESULT hr)
	{
		bool had_recording = false;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			had_recording = recording_active_;
			if (recording_requested_)
			{
				failed_recording_generation_ = recording_generation_;
			}
			snapshot_.state = Action4CameraState::Disconnected;
			snapshot_.error_code = hresult_code(hr);
		}
		if (had_recording)
		{
			finish_sink_writer(true);
		}
		close_capture();
		condition_.notify_all();
	}

	void handle_recording_error(HRESULT hr)
	{
		bool had_recording = false;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			had_recording = recording_active_;
			if (recording_requested_)
			{
				failed_recording_generation_ = recording_generation_;
			}
			snapshot_.state = Action4CameraState::Error;
			snapshot_.error_code = hresult_code(hr);
		}
		if (had_recording)
		{
			// 保留上面设置的 Error 状态，同时尽力封装已经写入的 MP4。
			finish_sink_writer(true);
		}
		close_capture();
		condition_.notify_all();
	}

	void worker_loop()
	{
		const HRESULT com_hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
		const HRESULT mf_hr = MFStartup(MF_VERSION, MFSTARTUP_FULL);
		if (FAILED(mf_hr))
		{
			std::lock_guard<std::mutex> lock(mutex_);
			snapshot_.state = Action4CameraState::Error;
			snapshot_.error_code = hresult_code(mf_hr);
			condition_.notify_all();
			if (SUCCEEDED(com_hr)) CoUninitialize();
			return;
		}

		std::uint64_t attempted_revision = std::numeric_limits<std::uint64_t>::max();
		while (true)
		{
			bool shutdown = false;
			bool preview = false;
			bool record = false;
			std::uint64_t revision = 0;
			std::uint64_t generation = 0;
			std::uint64_t failed_generation = 0;
			{
				std::lock_guard<std::mutex> lock(mutex_);
				shutdown = shutdown_requested_;
				preview = preview_requested_;
				record = recording_requested_;
				revision = request_revision_;
				generation = recording_generation_;
				failed_generation = failed_recording_generation_;
			}
			if (shutdown)
			{
				break;
			}

			const bool desired = preview || record;
			if (!reader_)
			{
				const bool same_session_failed = record && failed_generation == generation;
				if (!desired || same_session_failed || attempted_revision == revision)
				{
					std::unique_lock<std::mutex> lock(mutex_);
					if (!desired && !recording_active_ && !finalizing_)
					{
						const bool terminal_state =
							snapshot_.state == Action4CameraState::Missing ||
							snapshot_.state == Action4CameraState::Disconnected ||
							snapshot_.state == Action4CameraState::Error;
						if (!terminal_state)
						{
							snapshot_.state = Action4CameraState::Closed;
						}
						snapshot_.preview_enabled = preview_requested_;
					}
					condition_.wait_for(lock, std::chrono::milliseconds(100), [&]() {
						return shutdown_requested_ || request_revision_ != revision;
					});
					continue;
				}

				attempted_revision = revision;
				{
					std::lock_guard<std::mutex> lock(mutex_);
					snapshot_.state = Action4CameraState::Opening;
					snapshot_.error_code = 0;
					snapshot_.input_format = Action4InputFormat::Unknown;
					snapshot_.width = 0;
					snapshot_.height = 0;
					snapshot_.fps_numerator = 0;
					snapshot_.fps_denominator = 1;
				}
				bool missing = false;
				const HRESULT open_hr = open_capture(missing);
				if (FAILED(open_hr))
				{
					close_capture();
					std::lock_guard<std::mutex> lock(mutex_);
					if (recording_requested_)
					{
						failed_recording_generation_ = recording_generation_;
					}
					snapshot_.state = missing ? Action4CameraState::Missing : Action4CameraState::Error;
					snapshot_.error_code = hresult_code(open_hr);
					snapshot_.recording = false;
					condition_.notify_all();
					continue;
				}
				{
					std::lock_guard<std::mutex> lock(mutex_);
					snapshot_.state = recording_requested_
						? Action4CameraState::Opening
						: Action4CameraState::Previewing;
				}
				continue;
			}

			if (record && !recording_active_ && failed_generation != generation)
			{
				{
					std::lock_guard<std::mutex> lock(mutex_);
					// stop 请求可能在本轮状态快照之后到达，启动编码器前必须再次确认。
					if (!recording_requested_ || generation != recording_generation_)
					{
						continue;
					}
					recording_starting_ = true;
				}
				const HRESULT sink_hr = start_sink_writer();
				if (FAILED(sink_hr))
				{
					fail_recording_start(sink_hr);
				}
				{
					std::lock_guard<std::mutex> lock(mutex_);
					recording_starting_ = false;
				}
				condition_.notify_all();
				continue;
			}
			if (!record && recording_active_)
			{
				finish_sink_writer(false);
				continue;
			}
			if (!desired && !recording_active_)
			{
				close_capture();
				std::lock_guard<std::mutex> lock(mutex_);
				snapshot_.state = Action4CameraState::Closed;
				continue;
			}

			DWORD actual_stream = 0;
			DWORD stream_flags = 0;
			LONGLONG timestamp = 0;
			ComPtr<IMFSample> sample;
			HRESULT read_hr = S_OK;
			if (primed_sample_)
			{
				sample = primed_sample_;
				timestamp = primed_timestamp_;
				stream_flags = primed_stream_flags_;
				primed_sample_.Reset();
			}
			else
			{
				read_hr = reader_->ReadSample(
					MF_SOURCE_READER_FIRST_VIDEO_STREAM,
					0,
					&actual_stream,
					&stream_flags,
					&timestamp,
					&sample);
			}
			if (FAILED(read_hr) || (stream_flags & MF_SOURCE_READERF_ERROR) != 0 ||
				(stream_flags & MF_SOURCE_READERF_ENDOFSTREAM) != 0)
			{
				handle_disconnect(FAILED(read_hr) ? read_hr : MF_E_END_OF_STREAM);
				continue;
			}
			if (sample)
			{
				const HRESULT process_hr = process_sample(sample.Get(), timestamp, stream_flags);
				if (FAILED(process_hr))
				{
					handle_recording_error(process_hr);
				}
			}
		}

		if (recording_active_ || sink_writer_)
		{
			finish_sink_writer(false);
		}
		close_capture();
		MFShutdown();
		if (SUCCEEDED(com_hr))
		{
			CoUninitialize();
		}
		{
			std::lock_guard<std::mutex> lock(mutex_);
			snapshot_.state = Action4CameraState::Closed;
			snapshot_.recording = false;
			recording_active_ = false;
			finalizing_ = false;
		}
		condition_.notify_all();
	}

	mutable std::mutex mutex_;
	std::condition_variable condition_;
	std::thread worker_;
	bool shutdown_requested_ = false;
	bool preview_requested_ = false;
	bool recording_requested_ = false;
	bool recording_starting_ = false;
	bool recording_active_ = false;
	bool finalizing_ = false;
	std::uint64_t request_revision_ = 0;
	std::uint64_t recording_generation_ = 0;
	std::uint64_t failed_recording_generation_ = std::numeric_limits<std::uint64_t>::max();
	Action4CameraSnapshot snapshot_;

	std::wstring video_path_;
	std::wstring frame_timing_path_;
	std::int64_t session_anchor_qpc_ = 0;
	std::int64_t qpc_frequency_ = 1;
	std::int64_t recording_start_qpc_ = 0;
	LONGLONG first_source_timestamp_ = std::numeric_limits<LONGLONG>::min();
	LONGLONG last_source_timestamp_ = std::numeric_limits<LONGLONG>::min();
	std::uint64_t frame_index_ = 0;

	ComPtr<IMFMediaSource> source_;
	ComPtr<IMFSourceReader> reader_;
	ComPtr<IMFSample> primed_sample_;
	LONGLONG primed_timestamp_ = 0;
	DWORD primed_stream_flags_ = 0;
	ComPtr<IMFSinkWriter> sink_writer_;
	DWORD sink_stream_index_ = 0;
	int capture_width_ = 0;
	int capture_height_ = 0;
	LONG capture_stride_ = 0;
	int capture_fps_numerator_ = 0;
	int capture_fps_denominator_ = 1;
	Action4InputFormat selected_input_format_ = Action4InputFormat::Unknown;
	AsyncCsvWriter<VideoFrameTimingRow, 4096> frame_writer_;

	HANDLE preview_mapping_ = nullptr;
	HANDLE preview_event_ = nullptr;
	std::uint8_t* preview_view_ = nullptr;
	PreviewSharedHeader* preview_header_ = nullptr;
};

Action4CameraRecorder::Action4CameraRecorder()
	: impl_(new Impl())
{
}

Action4CameraRecorder::~Action4CameraRecorder() = default;

void Action4CameraRecorder::set_preview_enabled(bool enabled)
{
	impl_->set_preview_enabled(enabled);
}

bool Action4CameraRecorder::start_recording(
	const std::wstring& video_path,
	const std::wstring& frame_timing_path,
	std::int64_t session_anchor_qpc,
	std::int64_t qpc_frequency)
{
	return impl_->start_recording(video_path, frame_timing_path, session_anchor_qpc, qpc_frequency);
}

void Action4CameraRecorder::stop_recording_and_wait()
{
	impl_->stop_recording_and_wait();
}

bool Action4CameraRecorder::timing_writer_failed() const
{
	return impl_->timing_writer_failed();
}

Action4CameraSnapshot Action4CameraRecorder::snapshot() const
{
	return impl_->snapshot();
}

void Action4CameraRecorder::shutdown()
{
	if (impl_)
	{
		impl_->shutdown();
	}
}
