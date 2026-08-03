#pragma once

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

// 单生产者、单消费者异步 CSV 写入器。
// 生产者只复制一行到环形队列；文件写入、定期 flush 和错误检测全部在后台线程完成。
template <typename Row, std::size_t Capacity = 8192>
class AsyncCsvWriter
{
public:
	using WriteRowFunction = bool (*)(std::FILE*, const Row&);

	AsyncCsvWriter()
		: ring_(Capacity)
	{
		static_assert(Capacity >= 2, "CSV 环形队列容量至少为 2。");
	}

	~AsyncCsvWriter()
	{
		stop();
	}

	AsyncCsvWriter(const AsyncCsvWriter&) = delete;
	AsyncCsvWriter& operator=(const AsyncCsvWriter&) = delete;

	bool start(const std::wstring& path, const char* utf8_header, WriteRowFunction write_row)
	{
		if (accepting_.load(std::memory_order_acquire) || writer_thread_.joinable() || write_row == nullptr)
		{
			return false;
		}

		std::FILE* opened = nullptr;
		if (_wfopen_s(&opened, path.c_str(), L"wb") != 0 || opened == nullptr)
		{
			last_error_.store(errno != 0 ? errno : EIO, std::memory_order_release);
			failed_.store(true, std::memory_order_release);
			return false;
		}

		if (utf8_header != nullptr)
		{
			const std::size_t header_length = std::char_traits<char>::length(utf8_header);
			if (std::fwrite(utf8_header, 1, header_length, opened) != header_length ||
				std::fflush(opened) != 0)
			{
				last_error_.store(errno != 0 ? errno : EIO, std::memory_order_release);
				failed_.store(true, std::memory_order_release);
				std::fclose(opened);
				return false;
			}
		}

		fp_ = opened;
		write_row_ = write_row;
		head_.store(0, std::memory_order_relaxed);
		tail_.store(0, std::memory_order_relaxed);
		dropped_.store(0, std::memory_order_relaxed);
		failed_.store(false, std::memory_order_relaxed);
		last_error_.store(0, std::memory_order_relaxed);
		stop_requested_.store(false, std::memory_order_relaxed);
		worker_exited_.store(false, std::memory_order_relaxed);
		accepting_.store(true, std::memory_order_release);

		try
		{
			writer_thread_ = std::thread(&AsyncCsvWriter::writer_loop, this);
		}
		catch (...)
		{
			accepting_.store(false, std::memory_order_release);
			std::fclose(fp_);
			fp_ = nullptr;
			last_error_.store(EAGAIN, std::memory_order_release);
			failed_.store(true, std::memory_order_release);
			worker_exited_.store(true, std::memory_order_release);
			return false;
		}
		return true;
	}

	void stop()
	{
		request_stop();
		if (writer_thread_.joinable())
		{
			writer_thread_.join();
		}
		if (fp_ != nullptr)
		{
			if (std::fflush(fp_) != 0 && !failed_.load(std::memory_order_acquire))
			{
				mark_failed();
			}
			std::fclose(fp_);
			fp_ = nullptr;
		}
	}

	// 控制线程只关闭生产者入口；消费者会排空当前队列并自行退出。
	void request_stop()
	{
		accepting_.store(false, std::memory_order_release);
		stop_requested_.store(true, std::memory_order_release);
	}

	bool try_enqueue(const Row& row)
	{
		if (!accepting_.load(std::memory_order_acquire))
		{
			return false;
		}

		const std::size_t head = head_.load(std::memory_order_relaxed);
		const std::size_t next = (head + 1) % Capacity;
		if (next == tail_.load(std::memory_order_acquire))
		{
			dropped_.fetch_add(1, std::memory_order_relaxed);
			return false;
		}
		ring_[head] = row;
		head_.store(next, std::memory_order_release);
		return true;
	}

	bool is_running() const { return accepting_.load(std::memory_order_acquire); }
	bool stop_completed() const { return worker_exited_.load(std::memory_order_acquire); }
	bool has_error() const { return failed_.load(std::memory_order_acquire); }
	int last_error() const { return last_error_.load(std::memory_order_acquire); }
	std::uint64_t dropped_count() const { return dropped_.load(std::memory_order_acquire); }

	bool reset_status()
	{
		if (accepting_.load(std::memory_order_acquire) || writer_thread_.joinable() || fp_ != nullptr)
		{
			return false;
		}
		dropped_.store(0, std::memory_order_relaxed);
		failed_.store(false, std::memory_order_relaxed);
		last_error_.store(0, std::memory_order_relaxed);
		stop_requested_.store(false, std::memory_order_relaxed);
		worker_exited_.store(true, std::memory_order_release);
		return true;
	}

private:
	void mark_failed()
	{
		last_error_.store(errno != 0 ? errno : EIO, std::memory_order_release);
		failed_.store(true, std::memory_order_release);
		accepting_.store(false, std::memory_order_release);
	}

	void writer_loop()
	{
		std::size_t rows_since_flush = 0;
		while (true)
		{
			const std::size_t tail = tail_.load(std::memory_order_relaxed);
			const std::size_t head = head_.load(std::memory_order_acquire);
			if (tail == head)
			{
				if (stop_requested_.load(std::memory_order_acquire))
				{
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(2));
				continue;
			}

			if (fp_ == nullptr || !write_row_(fp_, ring_[tail]) || std::ferror(fp_) != 0)
			{
				mark_failed();
				tail_.store((tail + 1) % Capacity, std::memory_order_release);
				break;
			}
			tail_.store((tail + 1) % Capacity, std::memory_order_release);

			if (++rows_since_flush >= 64)
			{
				if (std::fflush(fp_) != 0)
				{
					mark_failed();
					break;
				}
				rows_since_flush = 0;
			}
		}

		if (fp_ != nullptr && std::fflush(fp_) != 0 && !failed_.load(std::memory_order_acquire))
		{
			mark_failed();
		}
		accepting_.store(false, std::memory_order_release);
		worker_exited_.store(true, std::memory_order_release);
	}

	std::vector<Row> ring_;
	std::atomic<std::size_t> head_{ 0 };
	std::atomic<std::size_t> tail_{ 0 };
	std::atomic<bool> accepting_{ false };
	std::atomic<bool> stop_requested_{ false };
	std::atomic<bool> failed_{ false };
	std::atomic<bool> worker_exited_{ true };
	std::atomic<int> last_error_{ 0 };
	std::atomic<std::uint64_t> dropped_{ 0 };
	std::thread writer_thread_;
	std::FILE* fp_ = nullptr;
	WriteRowFunction write_row_ = nullptr;
};
