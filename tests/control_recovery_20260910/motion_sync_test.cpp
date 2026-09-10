#include "motion_sync.h"
#include "plc_io.h"
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

// 只替换外设接口，直接测试工程中的同步实现；本程序不加载 ADS 或手柄 DLL。
Handle::Handle(DWORD serial) : serial_number_(serial) {}
namespace AdsSymbol
{
	const AxisReturnAdsSymbols axis6_return{};
}
static bool read_ok = true;
static std::vector<std::array<double, 7>> writes;
namespace plc_io
{
	bool clear_axis1_group_return_requests(AppContext&) { return true; }
	bool clear_axis_return_request(AppContext&, const AxisReturnAdsSymbols&) { return true; }
	bool write_axis4_manual_requests(AppContext&, bool, bool) { return true; }
	bool read_plc_state(AppContext&) { return read_ok; }
	void load_pos_from_actual(AppContext& ctx)
	{
		for (int i = 0; i < 7; ++i) ctx.pos[i] = ctx.plc_act_pos[i];
	}
	bool write_refer(AppContext& ctx)
	{
		std::array<double, 7> absolute{};
		for (int i = 0; i < 7; ++i) absolute[i] = ctx.pos[i] + ctx.plc_init_pos[i];
		writes.push_back(absolute);
		return true;
	}
}
bool is_within_range(double v, double lo, double hi, double tol)
{
	return v >= lo - tol && v <= hi + tol;
}
bool get_average_handle_pose(Handle&, int, double& linear, double& rotation)
{
	linear = 1.5;
	rotation = 0.8;
	return true;
}
bool get_average_dual_pos(Handle& a, Handle& b, int n, double& a0, double& a1, double& b0, double& b1)
{
	return get_average_handle_pose(a, n, a0, a1) && get_average_handle_pose(b, n, b0, b1);
}

struct Fixture
{
	AppContext ctx{};
	ControlConfig cfg{};
	Handle handle{};
	HandleFilterState filter1{}, filter6{};
	CrawlState crawl1{}, crawl6{};
	double pos[7]{}, actual[7]{}, init[7]{}, left[7]{};
#define SCALAR_FIELDS(X) \
	X(axis3_base_rel) X(axis5_base_rel) X(axis6_mirror_base_rel) \
	X(axis2_hold_rel) X(axis7_hold_rel) \
	X(axis1_prev_linear_filtered) X(axis6_prev_linear_filtered) \
	X(axis1_prev_rot_filtered) X(axis6_prev_rot_filtered) \
	X(axis1_follow_cmd_abs) X(axis6_follow_cmd_abs) \
	X(axis1_prev_abs_for_trigger) X(axis6_prev_abs_for_trigger) \
	X(independent_axis1_hold_rel) X(independent_axis2_hold_rel) \
	X(independent_axis3_hold_rel) X(independent_axis5_hold_rel) \
	X(axis6_locked_window_start_abs) X(axis6_locked_window_end_abs) \
	X(axis6_coop_prev_axis1_cmd_abs)
#define BOOL_FIELDS(X) \
	X(axis1_reverse_switch_guard_active) X(axis6_reverse_switch_guard_active) \
	X(axis1_prev_abs_valid) X(axis6_prev_abs_valid) X(axis6_window_locked) X(axis6_coop_ff_inited)
#define DECL_DOUBLE(name) double name = 0;
	SCALAR_FIELDS(DECL_DOUBLE)
#define DECL_BOOL(name) bool name = false;
	BOOL_FIELDS(DECL_BOOL)
	Fixture()
	{
		writes.clear();
		read_ok = true;
		ctx.cfg = &cfg;
		ctx.axis1_input_handle = ctx.axis6_input_handle = &handle;
		ctx.axis1_handle_filter = &filter1;
		ctx.axis6_handle_filter = &filter6;
		ctx.axis1_crawl = &crawl1;
		ctx.axis6_crawl = &crawl6;
		ctx.pos = pos;
		ctx.plc_act_pos = actual;
		ctx.plc_init_pos = init;
		ctx.plc_leftlimit = left;
#define BIND(name) ctx.name = &name;
		SCALAR_FIELDS(BIND)
		BOOL_FIELDS(BIND)
	}
	void expect_rotation(double a2, double a7)
	{
		assert(!writes.empty());
		for (const auto& target : writes)
		{
			assert(std::abs(target[1] - a2) < 1e-9);
			assert(std::abs(target[6] - a7) < 1e-9);
		}
	}
};

int main()
{
	{
		Fixture f;
		// 复现旧 PLC：绝对位置已为零，但偏置仍为自检前角度。
		f.init[1] = 37;
		f.init[6] = -62;
		f.actual[1] = -37;
		f.actual[6] = 62;
		assert(motion_sync::sync_axis1(f.ctx, 1, false));
		f.expect_rotation(0, 0);
		assert(f.crawl1.rot_base_rel == -37);
	}
	{
		Fixture f;
		f.init[1] = 37;
		f.init[6] = -62;
		f.actual[1] = -37;
		f.actual[6] = 62;
		assert(motion_sync::sync_all(f.ctx, 1));
		f.expect_rotation(0, 0);
	}
	{
		Fixture f;
		// 修正后的 PLC：角度与偏置均为零，即使内存保持值陈旧也不得恢复。
		f.axis2_hold_rel = 37;
		f.axis7_hold_rel = -62;
		assert(motion_sync::sync_axis6(f.ctx, 1, true, false, false));
		f.expect_rotation(0, 0);
		assert(f.crawl6.rot_base_rel == 0);
	}
	{
		Fixture f;
		// 非零位置重连必须原地保持，不能一律强制回零。
		f.actual[1] = 18;
		f.actual[6] = -23;
		f.axis2_hold_rel = 37;
		f.axis7_hold_rel = -62;
		assert(motion_sync::sync_cooperative_guidewire(f.ctx, 1, false, false));
		f.expect_rotation(18, -23);
		assert(f.crawl1.rot_base_rel == 18);
		assert(f.crawl6.rot_base_rel == -23);
	}
	{
		Fixture f;
		// 正常业务同步保留既有目标语义，避免影响自动换手。
		f.axis2_hold_rel = 37;
		f.axis7_hold_rel = -62;
		assert(motion_sync::sync_axis1(f.ctx, 1));
		f.expect_rotation(37, -62);
	}
	{
		Fixture f;
		read_ok = false;
		assert(!motion_sync::sync_axis1(f.ctx, 1, false));
		assert(!motion_sync::sync_axis6(f.ctx, 1, true, false, false));
		assert(!motion_sync::sync_all(f.ctx, 1));
		assert(writes.empty());
	}
	std::cout << "PASS: 6 motion recovery regression cases (no hardware)." << std::endl;
}
