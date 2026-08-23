#include "DualClampTypes.h"

const char* dual_clamp_phase_name(DualClampPhase phase)
{
	switch (phase)
	{
	case DualClampPhase::Idle: return "Idle";
	case DualClampPhase::SelfCheck: return "SelfCheck";
	case DualClampPhase::Baseline: return "Baseline";
	case DualClampPhase::FixedHold: return "FixedHold";
	case DualClampPhase::ReleaseMoving: return "ReleaseMoving";
	case DualClampPhase::ReturnMoving: return "ReturnMoving";
	case DualClampPhase::ReclampMoving: return "ReclampMoving";
	case DualClampPhase::RecoverHold: return "RecoverHold";
	case DualClampPhase::RecoverMove: return "RecoverMove";
	case DualClampPhase::Completed: return "Completed";
	case DualClampPhase::Aborted: return "Aborted";
	case DualClampPhase::Error: return "Error";
	case DualClampPhase::SelfCheckDone: return "SelfCheckDone";
	case DualClampPhase::SetupMove: return "SetupMove";
	case DualClampPhase::ReadyForClamp: return "ReadyForClamp";
	default: return "Unknown";
	}
}
