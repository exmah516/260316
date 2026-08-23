#pragma once

#include "DualClampController.h"

#include <string>

class DualClampPipeServer
{
public:
	DualClampPipeServer();
	int run(DualClampController& controller);

private:
	std::string handle_command(DualClampController& controller, const std::string& command);
};
