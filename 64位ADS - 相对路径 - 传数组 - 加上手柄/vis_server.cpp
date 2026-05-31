#include "vis_server.h"
#include <cstring>

static const char* kPipeName = "\\\\.\\pipe\\ADS_Control_Vis";

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
			sizeof(VisCommand) + 64,
			100,
			nullptr);

		if (pipe == INVALID_HANDLE_VALUE)
		{
			Sleep(500);
			continue;
		}

		OVERLAPPED ov{};
		ov.hEvent = CreateEvent(nullptr, TRUE, FALSE, nullptr);
		ConnectNamedPipe(pipe, &ov);

		while (!stop_requested_.load())
		{
			DWORD wait_result = WaitForSingleObject(ov.hEvent, 100);
			if (wait_result == WAIT_OBJECT_0) break;
		}
		CloseHandle(ov.hEvent);

		if (stop_requested_.load())
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
			if (PeekNamedPipe(pipe, nullptr, 0, nullptr, &available, nullptr) && available > 0)
			{
				VisCommand cmd{};
				DWORD read_bytes = 0;
				if (ReadFile(pipe, &cmd, sizeof(cmd), &read_bytes, nullptr) &&
					read_bytes == sizeof(cmd))
				{
					EnterCriticalSection(&cmd_cs_);
					int next_head = (cmd_head_ + 1) % kCmdQueueSize;
					if (next_head != cmd_tail_)
					{
						cmd_queue_[cmd_head_] = cmd;
						cmd_head_ = next_head;
					}
					LeaveCriticalSection(&cmd_cs_);
				}
			}

			Sleep(5);
		}

		DisconnectNamedPipe(pipe);
		CloseHandle(pipe);
	}
}
