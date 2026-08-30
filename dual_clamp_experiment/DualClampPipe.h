#pragma once

#include "DualClampController.h"
#include "ProgrammedDeliveryController.h"
#include "StandaloneRecordController.h"

#include <string>

class DualClampPipeServer
{
public:
	DualClampPipeServer();
	int run(DualClampController& controller);
	int run(DualClampController& controller, ProgrammedDeliveryController& program_controller);
	int run(DualClampController& controller, ProgrammedDeliveryController& program_controller,
		StandaloneRecordController& standalone_controller);

private:
	std::string handle_command(DualClampController& controller, const std::string& command);
	std::string handle_program_command(ProgrammedDeliveryController& controller, const std::string& command);
	std::string handle_standalone_command(StandaloneRecordController& controller, const std::string& command);
};
