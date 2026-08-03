#include "vis_server.h"
#include <cstring>
#include <vector>

static const char* kPipeName = "\\\\.\\pipe\\ADS_Control_Vis";
static constexpr std::uint32_t kCommandMagic = 0x31434D56;
static constexpr std::uint16_t kCommandVersion = 1;
static constexpr DWORD kMaxCommandPayloadBytes = 1024;

VisServer::~VisServer()
{
	stop();
}

bool VisServer::start()
{
	if (running_.load()) return true;
	InitializeCriticalSection(&state_cs_);
	InitializeCriticalSection(&cmd_cs_);
	stop_requested_.store(false);
	try
	{
		server_thread_ = std::thread(&VisServer::server_loop, this);
	}
	catch (...)
	{
		DeleteCriticalSection(&state_cs_);
		DeleteCriticalSection(&cmd_cs_);
		return false;
	}
	running_.store(true);
	return true;
}

void VisServer::stop()
{
	if (!running_.load() && !server_thread_.joinable()) return;
	stop_requested_.store(true);
	// 同步 ConnectNamedPipe 可能正在等待客户端，用一次本地连接将其唤醒。
	HANDLE wake_pipe = CreateFileA(
		kPipeName,
		GENERIC_READ | GENERIC_WRITE,
		0,
		nullptr,
		OPEN_EXISTING,
		0,
		nullptr);
	if (wake_pipe != INVALID_HANDLE_VALUE)
	{
		CloseHandle(wake_pipe);
	}
	if (server_thread_.joinable())
	{
		server_thread_.join();
	}
	DeleteCriticalSection(&state_cs_);
	DeleteCriticalSection(&cmd_cs_);
	running_.store(false);
}

void VisServer::push_state(const VisState& state)
{
	EnterCriticalSection(&state_cs_);
	latest_state_ = state;
	state_dirty_ = true;
	LeaveCriticalSection(&state_cs_);
}

bool VisServer::poll_command(VisCommand& cmd)
{
	EnterCriticalSection(&cmd_cs_);
	if (cmd_head_ == cmd_tail_)
	{
		LeaveCriticalSection(&cmd_cs_);
		return false;
	}
	cmd = cmd_queue_[cmd_tail_];
	cmd_tail_ = (cmd_tail_ + 1) % kCmdQueueSize;
	LeaveCriticalSection(&cmd_cs_);
	return true;
}

void VisServer::server_loop()
{
	while (!stop_requested_.load())
	{
		HANDLE pipe = CreateNamedPipeA(
			kPipeName,
			PIPE_ACCESS_DUPLEX,
			PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
			1,
			sizeof(VisState) + 64,
			sizeof(VisWireCommandHeader) + kMaxCommandPayloadBytes + 64,
			100,
			nullptr);

		if (pipe == INVALID_HANDLE_VALUE)
		{
			Sleep(500);
			continue;
		}

		const BOOL connect_result = ConnectNamedPipe(pipe, nullptr);
		const DWORD connect_error = connect_result ? ERROR_SUCCESS : GetLastError();
		const bool connected = connect_result != FALSE || connect_error == ERROR_PIPE_CONNECTED;

		if (stop_requested_.load() || !connected)
		{
			DisconnectNamedPipe(pipe);
			CloseHandle(pipe);
			break;
		}

		while (!stop_requested_.load())
		{
			EnterCriticalSection(&state_cs_);
			bool dirty = state_dirty_;
			VisState snap = latest_state_;
			state_dirty_ = false;
			LeaveCriticalSection(&state_cs_);

			if (dirty)
			{
				DWORD written = 0;
				if (!WriteFile(pipe, &snap, sizeof(snap), &written, nullptr))
				{
					break;
				}
			}

			DWORD available = 0;
			DWORD message_bytes = 0;
			if (PeekNamedPipe(pipe, nullptr, 0, nullptr, &available, &message_bytes) && message_bytes > 0)
			{
				if (message_bytes > sizeof(VisWireCommandHeader) + kMaxCommandPayloadBytes)
				{
					break;
				}
				std::vector<std::uint8_t> message(message_bytes);
				DWORD read_bytes = 0;
				if (!ReadFile(pipe, message.data(), static_cast<DWORD>(message.size()), &read_bytes, nullptr))
				{
					break;
				}
				if (read_bytes >= sizeof(VisWireCommandHeader))
				{
					VisWireCommandHeader header{};
					std::memcpy(&header, message.data(), sizeof(header));
					const bool valid_header =
						header.magic == kCommandMagic &&
						header.version == kCommandVersion &&
						header.header_size == sizeof(VisWireCommandHeader) &&
						header.payload_size <= kMaxCommandPayloadBytes &&
						read_bytes == sizeof(VisWireCommandHeader) + header.payload_size;
					if (valid_header)
					{
						VisCommand cmd;
						cmd.type = header.type;
						cmd.param1 = header.param1;
						cmd.param2 = header.param2;
						if (header.payload_size > 0)
						{
							cmd.payload_utf8.assign(
								reinterpret_cast<const char*>(message.data() + sizeof(VisWireCommandHeader)),
								header.payload_size);
						}
						EnterCriticalSection(&cmd_cs_);
						const int next_head = (cmd_head_ + 1) % kCmdQueueSize;
						if (next_head != cmd_tail_)
						{
							cmd_queue_[cmd_head_] = std::move(cmd);
							cmd_head_ = next_head;
						}
						LeaveCriticalSection(&cmd_cs_);
					}
				}
			}

			Sleep(5);
		}

		DisconnectNamedPipe(pipe);
		CloseHandle(pipe);
	}
}
