#include "ProgrammedDeliveryTypes.h"

const char* programmed_delivery_mode_name(ProgrammedDeliveryMode mode)
{
	switch (mode)
	{
	case ProgrammedDeliveryMode::Legacy: return "legacy";
	case ProgrammedDeliveryMode::Catheter: return "catheter";
	case ProgrammedDeliveryMode::Guidewire: return "guidewire";
	default: return "unknown";
	}
}

const char* programmed_delivery_phase_name(ProgrammedDeliveryPhase phase)
{
	switch (phase)
	{
	case ProgrammedDeliveryPhase::Idle: return "Idle";
	case ProgrammedDeliveryPhase::SetupMove: return "SetupMove";
	case ProgrammedDeliveryPhase::Ready: return "Ready";
	case ProgrammedDeliveryPhase::Baseline: return "Baseline";
	case ProgrammedDeliveryPhase::ForwardToTrigger: return "ForwardToTrigger";
	case ProgrammedDeliveryPhase::ReleaseMovingClamp: return "ReleaseMovingClamp";
	case ProgrammedDeliveryPhase::ReturnMoving: return "ReturnMoving";
	case ProgrammedDeliveryPhase::ReclampMovingClamp: return "ReclampMovingClamp";
	case ProgrammedDeliveryPhase::CycleDecision: return "CycleDecision";
	case ProgrammedDeliveryPhase::FinalForward: return "FinalForward";
	case ProgrammedDeliveryPhase::Completed: return "Completed";
	case ProgrammedDeliveryPhase::Aborted: return "Aborted";
	case ProgrammedDeliveryPhase::Error: return "Error";
	default: return "Unknown";
	}
}
