    1: host_session_changed := G.host_session_id <> host_last_session_id;
    2: host_heartbeat_changed := host_session_changed OR
    3: 	(G.host_heartbeat_sequence <> host_last_heartbeat_sequence);
    4: 
    5: IF host_session_changed THEN
    6: 	// 非零会话发生变化表示新的上位机接管，必须重新完成恢复握手。
    7: 	IF host_last_session_id <> 0 THEN
    8: 		host_timeout_latched := TRUE;
    9: 	END_IF
   10: 	host_last_session_id := G.host_session_id;
   11: END_IF
   12: 
   13: IF host_heartbeat_changed THEN
   14: 	host_last_heartbeat_sequence := G.host_heartbeat_sequence;
   15: END_IF
   16: 
   17: host_heartbeat_seen := G.host_session_id <> 0;
   18: // 心跳变化的扫描周期将 TON 复位；序号连续 100 ms 不变化才判定通信超时。
   19: host_watchdog_timer(
   20: 	IN:=host_heartbeat_seen AND NOT host_heartbeat_changed,
   21: 	PT:=T#100MS
   22: );
   23: IF (NOT host_heartbeat_seen) OR host_watchdog_timer.Q THEN
   24: 	host_timeout_latched := TRUE;
   25: END_IF
   26: G.host_comm_timeout := host_timeout_latched;
   27: 
   28: IF G.handle_reinit_req THEN
   29: 	init_done := FALSE;
   30: 	G.handle_reinit_done := FALSE;
   31: 	G.handle_reinit_req := FALSE;
   32: 	G.estop_hold_req := TRUE;
   33: 
   34: 	FOR i:=1 TO 7 DO
   35: 		ext_setpoint_enabled[i] := FALSE;
   36: 		spg_enable_exec[i] := FALSE;
   37: 		spg_disable_exec[i] := FALSE;
   38: 		return_state[i] := 0;
   39: 		return_exec[i] := FALSE;
   40: 		return_finish_error[i] := FALSE;
   41: 		return_error_id_pending[i] := 0;
   42: 		traj_pos[i] := 0.0;
   43: 		traj_vel[i] := 0.0;
   44: 		traj_acc[i] := 0.0;
   45: 		host_hold_abs[i] := 0.0;
   46: 		G.return_cmd[i].Req := FALSE;
   47: 		G.return_cmd[i].Busy := FALSE;
   48: 		G.return_cmd[i].Done := FALSE;
   49: 		G.return_cmd[i].Error := FALSE;
   50: 		G.return_cmd[i].ErrorId := 0;
   51: 	END_FOR
   52: 	host_hold_captured := FALSE;
   53: 	host_timeout_prev := FALSE;
   54: 
   55: 	axis4_manual_state := 0;
   56: 	axis4_request_dir_now := 0;
   57: 	axis4_requested_dir := 0;
   58: 	axis4_pending_dir := 0;
   59: 	axis4_move_exec := FALSE;
   60: 	axis4_stop_exec := FALSE;
   61: 	axis4_finish_error := FALSE;
   62: 	axis4_error_id_pending := 0;
   63: 	G.axis4_fwd_req := FALSE;
   64: 	G.axis4_rev_req := FALSE;
   65: 	G.axis4_manual_busy := FALSE;
   66: 	G.axis4_manual_done := FALSE;
   67: 	G.axis4_manual_error := FALSE;
   68: 	G.axis4_manual_error_id := 0;
   69: END_IF
   70: 
   71: IF NOT init_done THEN
   72: 	G.estop_hold_req := TRUE;
   73: 	all_enabled := TRUE;
   74: 	FOR i:=1 TO 7 DO
   75: 		G.SetPointGenEnable[i](Axis:=G.axis[i], Execute:=TRUE);
   76: 		IF NOT G.SetPointGenEnable[i].Done THEN
   77: 			all_enabled := FALSE;
   78: 		END_IF
   79: 	END_FOR
   80: 	
   81: 	IF all_enabled THEN
   82: 		FOR j:=1 TO 7 DO
   83: 			G.Act_pos[j] := G.axis[j].NcToPlc.ActPos - G.init_pos[j];
   84: 			G.act_pos_from_left[j] := G.axis[j].NcToPlc.ActPos - G.leftlimit[j];
   85: 			G.ref_slow[j] := G.Act_pos[j];
   86: 			G.act_h[j] := G.Act_pos[j];
   87: 			G.act_hf[j] := G.Act_pos[j];
   88: 			G.refer[j] := G.Act_pos[j];
   89: 			G.refer_from_left[j] := G.act_pos_from_left[j];
   90: 			ext_setpoint_enabled[j] := TRUE;
   91: 
   92: 			FOR k:=1 TO 30 DO
   93: 				act_hh[j][k] := G.Act_pos[j];
   94: 			END_FOR
   95: 			act_h_prev[j] := G.Act_pos[j];
   96: 			traj_pos[j] := G.Act_pos[j];
   97: 			traj_vel[j] := 0.0;
   98: 			traj_acc[j] := 0.0;
   99: 			IF slimit_enable[j] THEN
  100: 				right_slimit[j] := G.leftlimit[j] + rslimit_distance[j];
  101: 			END_IF
  102: 			G.return_cmd[j].Busy := FALSE;
  103: 			G.return_cmd[j].Done := FALSE;
  104: 			G.return_cmd[j].Error := FALSE;
  105: 			G.return_cmd[j].ErrorId := 0;
  106: 		END_FOR
  107: 
  108: 		G.axis4_manual_busy := FALSE;
  109: 		G.axis4_manual_done := FALSE;
  110: 		G.axis4_manual_error := FALSE;
  111: 		G.axis4_manual_error_id := 0;
  112: 
  113: 		init_done := TRUE;
  114: 		// 通信超时未解除时不提前确认恢复；正式握手后会再次进入本初始化流程。
  115: 		G.handle_reinit_done := NOT G.host_comm_timeout;
  116: 	END_IF
  117: 	RETURN;
  118: END_IF
  119: 
  120: all_powered := TRUE;
  121: FOR i:=1 TO 7 DO
  122: 	IF NOT G.power_output[i].Done THEN
  123: 		all_powered := FALSE;
  124: 		EXIT;
  125: 	END_IF
  126: END_FOR
  127: 
  128: has_axis_error := FALSE;
  129: FOR i:=1 TO 7 DO
  130: 	IF G.axis[i].Status.Error OR G.power_output[i].Error OR G.reset_output[i].Error THEN
  131: 		has_axis_error := TRUE;
  132: 		EXIT;
  133: 	END_IF
  134: END_FOR
  135: 
  136: IF NOT all_powered THEN
  137: 	G.estop_hold_req := TRUE;
  138: 	hold_active := TRUE;
  139: 	FOR j:=1 TO 7 DO
  140: 		G.Act_pos[j] := G.axis[j].NcToPlc.ActPos - G.init_pos[j];
  141: 		G.act_pos_from_left[j] := G.axis[j].NcToPlc.ActPos - G.leftlimit[j];
  142: 		G.refer[j] := G.Act_pos[j];
  143: 		G.ref_slow[j] := G.Act_pos[j];
  144: 		G.act_h[j] := G.Act_pos[j];
  145: 		G.act_hf[j] := G.Act_pos[j];
  146: 		G.refer_from_left[j] := G.act_pos_from_left[j];
  147: 		FOR k:=1 TO 30 DO
  148: 			act_hh[j][k] := G.Act_pos[j];
  149: 		END_FOR
  150: 		act_h_prev[j] := G.Act_pos[j];
  151: 		traj_pos[j] := G.Act_pos[j];
  152: 		traj_vel[j] := 0.0;
  153: 		traj_acc[j] := 0.0;
  154: 	END_FOR
  155: 	IF has_axis_error THEN
  156: 		G.gen_state := _err;
  157: 	END_IF
  158: 	RETURN;
  159: END_IF
  160: 
  161: IF hold_active THEN
  162: 	hold_active := FALSE;
  163: 	G.handle_reinit_req := TRUE;
  164: 	G.estop_hold_req := TRUE;
  165: 	IF has_axis_error THEN
  166: 		G.gen_state := _err;
  167: 	END_IF
  168: 	RETURN;
  169: END_IF
  170: 
  171: IF has_axis_error THEN
  172: 	G.estop_hold_req := TRUE;
  173: 	G.gen_state := _err;
  174: 	RETURN;
  175: END_IF
  176: 
  177: IF G.host_comm_timeout THEN
  178: 	G.estop_hold_req := TRUE;
  179: 	G.handle_reinit_done := FALSE;
  180: 
  181: 	IF (NOT host_timeout_prev) OR (NOT host_hold_captured) THEN
  182: 		// 仅在进入超时保持时采样一次。之后始终维持同一绝对位置，
  183: 		// 避免 Actual -> Set Position 的逐周期回灌导致低频来回摆动。
  184: 		FOR i:=1 TO 7 DO
  185: 			host_hold_abs[i] := G.axis[i].NcToPlc.ActPos;
  186: 		END_FOR
  187: 		host_hold_captured := TRUE;
  188: 	END_IF
  189: 
  190: 	IF NOT host_timeout_prev THEN
  191: 		// 仅捕获超时发生时已经在运行的独立运动，禁止恢复后自动续跑。
  192: 		FOR i:=1 TO 7 DO
  193: 			host_return_stop_active[i] := return_enable[i] AND
  194: 				((return_state[i] <> 0) OR G.return_cmd[i].Busy OR fb_return_move_abs[i].Busy);
  195: 			host_return_stop_exec[i] := FALSE;
  196: 		END_FOR
  197: 		host_axis4_stop_active := (axis4_manual_state <> 0) OR fb_axis4_move_velocity.Busy;
  198: 	END_IF
  199: 
  200: 	// 超时期间持续清除运动请求，避免上位机重连时把旧请求重新送入状态机。
  201: 	G.axis1_fast_return := FALSE;
  202: 	G.axis6_fast_retract := FALSE;
  203: 	G.axis4_fwd_req := FALSE;
  204: 	G.axis4_rev_req := FALSE;
  205: 	G.startup_smoothing_bypass := FALSE;
  206: 	axis4_move_exec := FALSE;
  207: 	host_all_stopped := TRUE;
  208: 
  209: 	FOR i:=1 TO 7 DO
  210: 		spg_enable_exec[i] := FALSE;
  211: 		spg_disable_exec[i] := FALSE;
  212: 		return_exec[i] := FALSE;
  213: 		G.return_cmd[i].Req := FALSE;
  214: 		G.return_cmd[i].Done := FALSE;
  215: 
  216: 		// 实际位置仍实时刷新用于诊断；外部参考始终保持超时边沿的锁存位置。
  217: 		G.Act_pos[i] := G.axis[i].NcToPlc.ActPos - G.init_pos[i];
  218: 		G.act_pos_from_left[i] := G.axis[i].NcToPlc.ActPos - G.leftlimit[i];
  219: 		G.refer[i] := host_hold_abs[i] - G.init_pos[i];
  220: 		G.ref_slow[i] := G.refer[i];
  221: 		G.act_h[i] := G.refer[i];
  222: 		G.act_hf[i] := G.refer[i];
  223: 		G.refer_from_left[i] := host_hold_abs[i] - G.leftlimit[i];
  224: 		FOR k:=1 TO 30 DO
  225: 			act_hh[i][k] := G.refer[i];
  226: 		END_FOR
  227: 		act_h_prev[i] := G.refer[i];
  228: 		traj_pos[i] := G.refer[i];
  229: 		traj_vel[i] := 0.0;
  230: 		traj_acc[i] := 0.0;
  231: 
  232: 		IF host_return_stop_active[i] THEN
  233: 			IF fb_host_return_stop[i].Done THEN
  234: 				host_return_stop_exec[i] := FALSE;
  235: 				host_return_stop_active[i] := FALSE;
  236: 			ELSIF fb_host_return_stop[i].Error THEN
  237: 				// 拉低一个扫描周期清除错误，下一周期继续尝试受控停止。
  238: 				host_return_stop_exec[i] := FALSE;
  239: 				host_all_stopped := FALSE;
  240: 			ELSE
  241: 				host_return_stop_exec[i] := TRUE;
  242: 				host_all_stopped := FALSE;
  243: 			END_IF
  244: 		ELSE
  245: 			host_return_stop_exec[i] := FALSE;
  246: 		END_IF
  247: 		G.return_cmd[i].Busy := host_return_stop_active[i];
  248: 
  249: 		IF ext_setpoint_enabled[i] THEN
  250: 			MC_ExtSetPointGenFeed(
  251: 				Position:=host_hold_abs[i],
  252: 				Velocity:=0.0,
  253: 				Acceleration:=0.0,
  254: 				Direction:=1,
  255: 				Axis:=G.axis[i]);
  256: 		END_IF
  257: 
  258: 		// 所有运动功能块在看门狗分支中仍保持每扫描调用。
  259: 		G.SetPointGenEnable[i](Axis:=G.axis[i], Execute:=FALSE);
  260: 		G.SetPointGenDisable[i](Axis:=G.axis[i], Execute:=FALSE);
  261: 		fb_return_move_abs[i](
  262: 			Axis:=G.axis[i],
  263: 			Position:=return_target_abs[i],
  264: 			Velocity:=return_velocity[i],
  265: 			Acceleration:=return_acc[i],
  266: 			Deceleration:=return_dec[i],
  267: 			Jerk:=return_jerk[i],
  268: 			Execute:=FALSE
  269: 		);
  270: 		fb_host_return_stop[i](
  271: 			Axis:=G.axis[i],
  272: 			Execute:=host_return_stop_exec[i],
  273: 			Deceleration:=100.0,
  274: 			Jerk:=500.0
  275: 		);
  276: 	END_FOR
  277: 
  278: 	IF host_axis4_stop_active THEN
  279: 		IF fb_axis4_stop.Done THEN
  280: 			axis4_stop_exec := FALSE;
  281: 			host_axis4_stop_active := FALSE;
  282: 		ELSIF fb_axis4_stop.Error THEN
  283: 			// 拉低一个扫描周期清除错误，下一周期继续尝试受控停止。
  284: 			axis4_stop_exec := FALSE;
  285: 			host_all_stopped := FALSE;
  286: 		ELSE
  287: 			axis4_stop_exec := TRUE;
  288: 			host_all_stopped := FALSE;
  289: 		END_IF
  290: 	ELSE
  291: 		axis4_stop_exec := FALSE;
  292: 	END_IF
  293: 	G.axis4_manual_busy := host_axis4_stop_active;
  294: 	G.axis4_manual_done := FALSE;
  295: 
  296: 	fb_axis4_move_velocity(
  297: 		Axis:=G.axis[4],
  298: 		Execute:=FALSE,
  299: 		Velocity:=axis4_manual_velocity,
  300: 		Acceleration:=axis4_manual_acc,
  301: 		Deceleration:=axis4_manual_dec,
  302: 		Jerk:=axis4_manual_jerk,
  303: 		Direction:=axis4_manual_direction
  304: 	);
  305: 	fb_axis4_stop(
  306: 		Axis:=G.axis[4],
  307: 		Execute:=axis4_stop_exec,
  308: 		Deceleration:=axis4_manual_dec,
  309: 		Jerk:=axis4_manual_jerk
  310: 	);
  311: 
  312: 	// 清请求并解除超时即为 PLC 对恢复请求的确认；随后复用既有重初始化流程。
  313: 	IF G.host_recover_req AND host_heartbeat_seen AND
  314: 		(NOT host_watchdog_timer.Q) AND host_all_stopped THEN
  315: 		host_timeout_latched := FALSE;
  316: 		G.host_comm_timeout := FALSE;
  317: 		G.host_recover_req := FALSE;
  318: 		G.handle_reinit_req := TRUE;
  319: 	END_IF
  320: 
  321: 	host_timeout_prev := G.host_comm_timeout;
  322: 	RETURN;
  323: END_IF
  324: 
  325: host_timeout_prev := FALSE;
  326: host_hold_captured := FALSE;
  327: // 非超时状态收到的恢复请求没有业务含义，清除以免成为下一次超时的陈旧握手。
  328: G.host_recover_req := FALSE;
  329: 
  330: G.estop_hold_req := FALSE;
  331: 
  332: IF G.cylinder5_press_req THEN
  333: 	G.cylinder5_value := 0;
  334: ELSE
  335: 	G.cylinder5_value := 2000;
  336: END_IF
  337: 
  338: FOR i:=1 TO 7 DO
  339: 	spg_enable_exec[i] := FALSE;
  340: 	spg_disable_exec[i] := FALSE;
  341: 	return_exec[i] := FALSE;
  342: END_FOR
  343: axis4_move_exec := FALSE;
  344: axis4_stop_exec := FALSE;
  345: 
  346: FOR j:=1 TO 7 DO
  347: 	G.Act_pos[j] := G.axis[j].NcToPlc.ActPos - G.init_pos[j];
  348: 	G.act_pos_from_left[j] := G.axis[j].NcToPlc.ActPos - G.leftlimit[j];
  349: 	G.refer_from_left[j] := (G.refer[j] + G.init_pos[j]) - G.leftlimit[j];
  350: 
  351: 	v_step := G.v_limit[j];
  352: 	// 回退接管旁路：回退 Busy 期间跳过斜坡/均值，避免回退末端重新进入慢链路
  353: 	return_handoff_bypass := ((j = 1) AND (G.axis1_fast_return OR G.return_cmd[1].Busy))
  354: 		OR ((j = 6) AND (G.axis6_fast_retract OR G.return_cmd[6].Busy));
  355: 	IF return_handoff_bypass THEN
  356: 		avg_window := 1;
  357: 		ramp_counter[j] := 0;
  358: 	ELSE
  359: 		ramp_counter[j] := MIN(ramp_counter[j] + 1, avg_window_size);
  360: 		avg_window := MAX(1, ramp_counter[j]);
  361: 	END_IF
  362: 
  363: 	FOR k:=2 TO 20 DO
  364: 		act_hh[j][22-k] := act_hh[j][21-k];
  365: 	END_FOR
  366: 
  367: 	IF slimit_enable[j] THEN
  368: 		IF (G.axis[j].NcToPlc.ActPos >= right_slimit[j]) AND (G.refer[j] > G.ref_slow[j]) THEN
  369: 			G.refer[j] := G.ref_slow[j];
  370: 		END_IF
  371: 	END_IF
  372: 
  373: 	IF return_handoff_bypass THEN
  374: 		G.ref_slow[j] := G.refer[j];
  375: 	ELSIF (G.refer[j] - G.ref_slow[j]) > v_step THEN
  376: 		G.ref_slow[j] := G.ref_slow[j] + v_step;
  377: 	ELSIF (G.refer[j] - G.ref_slow[j]) < -v_step THEN
  378: 		G.ref_slow[j] := G.ref_slow[j] - v_step;
  379: 	ELSE
  380: 		G.ref_slow[j] := G.refer[j];
  381: 	END_IF
  382: 
  383: 	IF return_handoff_bypass THEN
  384: 		FOR i:=1 TO 20 DO
  385: 			act_hh[j][i] := G.ref_slow[j];
  386: 		END_FOR
  387: 		G.act_h[j] := G.ref_slow[j];
  388: 	ELSE
  389: 		act_hh[j][1] := G.ref_slow[j];
  390: 		G.act_h[j] := 0;
  391: 		FOR i:=1 TO avg_window DO
  392: 			G.act_h[j] := G.act_h[j] + act_hh[j][i] / TO_LREAL(avg_window);
  393: 		END_FOR
  394: 	END_IF
  395: 
  396: 	IF traj_enable[j] AND NOT return_handoff_bypass AND NOT G.startup_smoothing_bypass THEN
  397: 		acc_raw := traj_wn * traj_wn * (G.refer[j] - traj_pos[j]) - 2.0 * traj_wn * traj_vel[j];
  398: 		IF acc_raw > traj_amax[j] THEN
  399: 			acc_raw := traj_amax[j];
  400: 		ELSIF acc_raw < -traj_amax[j] THEN
  401: 			acc_raw := -traj_amax[j];
  402: 		END_IF
  403: 		traj_acc[j] := acc_raw;
  404: 		traj_vel[j] := traj_vel[j] + 0.001 * traj_acc[j];
  405: 		IF traj_vel[j] > traj_vmax[j] THEN
  406: 			traj_vel[j] := traj_vmax[j];
  407: 		ELSIF traj_vel[j] < -traj_vmax[j] THEN
  408: 			traj_vel[j] := -traj_vmax[j];
  409: 		END_IF
  410: 		traj_pos[j] := traj_pos[j] + 0.001 * traj_vel[j];
  411: 		IF ABS(G.refer[j] - traj_pos[j]) < 0.001 AND ABS(traj_vel[j]) < 0.01 THEN
  412: 			traj_pos[j] := G.refer[j];
  413: 			traj_vel[j] := 0.0;
  414: 			traj_acc[j] := 0.0;
  415: 		END_IF
  416: 	ELSE
  417: 		traj_pos[j] := G.act_h[j];
  418: 		traj_vel[j] := 0.0;
  419: 		traj_acc[j] := 0.0;
  420: 	END_IF
  421: END_FOR
  422: 
  423: FOR i:=1 TO 7 DO
  424: 	IF NOT return_enable[i] THEN
  425: 		CONTINUE;
  426: 	END_IF
  427: 
  428: 	IF return_state[i] = 0 THEN
  429: 		G.return_cmd[i].Busy := FALSE;
  430: 		IF NOT G.return_cmd[i].Req THEN
  431: 			G.return_cmd[i].Done := FALSE;
  432: 			G.return_cmd[i].Error := FALSE;
  433: 			G.return_cmd[i].ErrorId := 0;
  434: 			return_finish_error[i] := FALSE;
  435: 			return_error_id_pending[i] := 0;
  436: 		ELSIF (NOT G.return_cmd[i].Done) AND (NOT G.return_cmd[i].Error) THEN
  437: 			// Req 由上位机在完整写入参数后最后置位；此处只锁存一次，
  438: 			// 本次 MC_MoveAbsolute 后续不再直接读取可被 ADS 改写的命令参数。
  439: 			return_target_abs[i] := G.return_cmd[i].TargetAbs;
  440: 			return_velocity[i] := G.return_cmd[i].Velocity;
  441: 			return_acc[i] := G.return_cmd[i].Acc;
  442: 			return_dec[i] := G.return_cmd[i].Dec;
  443: 			return_jerk[i] := G.return_cmd[i].Jerk;
  444: 			return_state[i] := 10;
  445: 		END_IF
  446: 	END_IF
  447: 
  448: 	CASE return_state[i] OF
  449: 	10:
  450: 		G.return_cmd[i].Busy := TRUE;
  451: 		spg_disable_exec[i] := TRUE;
  452: 		IF G.SetPointGenDisable[i].Done OR NOT G.SetPointGenDisable[i].Enabled THEN
  453: 			ext_setpoint_enabled[i] := FALSE;
  454: 			return_state[i] := 20;
  455: 		END_IF
  456: 
  457: 	20:
  458: 		G.return_cmd[i].Busy := TRUE;
  459: 		return_exec[i] := TRUE;
  460: 		G.refer_from_left[i] := return_target_abs[i] - G.leftlimit[i];
  461: 
  462: 		IF fb_return_move_abs[i].Done OR
  463: 		   ((NOT fb_return_move_abs[i].Busy) AND (ABS(G.axis[i].NcToPlc.ActPos - return_target_abs[i]) <= 0.05)) THEN
  464: 			return_finish_error[i] := FALSE;
  465: 			return_error_id_pending[i] := 0;
  466: 			return_state[i] := 30;
  467: 		ELSIF fb_return_move_abs[i].Error OR fb_return_move_abs[i].CommandAborted THEN
  468: 			return_finish_error[i] := TRUE;
  469: 			return_error_id_pending[i] := fb_return_move_abs[i].ErrorID;
  470: 			return_state[i] := 30;
  471: 		END_IF
  472: 
  473: 	30:
  474: 		G.return_cmd[i].Busy := TRUE;
  475: 		IF ABS(G.axis[i].NcToPlc.ActVelo) <= 0.5 THEN
  476: 			G.Act_pos[i] := G.axis[i].NcToPlc.ActPos - G.init_pos[i];
  477: 			G.act_pos_from_left[i] := G.axis[i].NcToPlc.ActPos - G.leftlimit[i];
  478: 			G.refer[i] := G.Act_pos[i];
  479: 			G.ref_slow[i] := G.Act_pos[i];
  480: 			G.act_h[i] := G.Act_pos[i];
  481: 			G.act_hf[i] := G.Act_pos[i];
  482: 			G.refer_from_left[i] := G.act_pos_from_left[i];
  483: 			FOR k:=1 TO 30 DO
  484: 				act_hh[i][k] := G.Act_pos[i];
  485: 			END_FOR
  486: 			act_h_prev[i] := G.Act_pos[i];
  487: 			traj_pos[i] := G.Act_pos[i];
  488: 			traj_vel[i] := 0.0;
  489: 			traj_acc[i] := 0.0;
  490: 			// 先留出一个完整 PLC 扫描周期预送当前设定值，再重新使能外部设定值发生器。
  491: 			return_state[i] := 31;
  492: 		END_IF
  493: 
  494: 	31:
  495: 		G.return_cmd[i].Busy := TRUE;
  496: 		spg_enable_exec[i] := TRUE;
  497: 		IF G.SetPointGenEnable[i].Done THEN
  498: 			ext_setpoint_enabled[i] := TRUE;
  499: 			G.return_cmd[i].Busy := FALSE;
  500: 			IF return_finish_error[i] THEN
  501: 				G.return_cmd[i].Done := FALSE;
  502: 				G.return_cmd[i].Error := TRUE;
  503: 				G.return_cmd[i].ErrorId := return_error_id_pending[i];
  504: 			ELSE
  505: 				G.return_cmd[i].Done := TRUE;
  506: 				G.return_cmd[i].Error := FALSE;
  507: 				G.return_cmd[i].ErrorId := 0;
  508: 			END_IF
  509: 			return_finish_error[i] := FALSE;
  510: 			return_error_id_pending[i] := 0;
  511: 			return_state[i] := 0;
  512: 		END_IF
  513: 	END_CASE
  514: END_FOR
  515: 
  516: axis4_request_dir_now := 0;
  517: IF G.axis4_fwd_req AND NOT G.axis4_rev_req THEN
  518: 	axis4_request_dir_now := 1;
  519: ELSIF G.axis4_rev_req AND NOT G.axis4_fwd_req THEN
  520: 	axis4_request_dir_now := -1;
  521: END_IF
  522: 
  523: IF axis4_manual_state = 0 THEN
  524: 	G.axis4_manual_busy := FALSE;
  525: 	IF axis4_request_dir_now = 0 THEN
  526: 		G.axis4_manual_done := FALSE;
  527: 		G.axis4_manual_error := FALSE;
  528: 		G.axis4_manual_error_id := 0;
  529: 	ELSIF NOT G.axis4_manual_error THEN
  530: 		G.axis4_manual_done := FALSE;
  531: 		G.axis4_manual_error := FALSE;
  532: 		G.axis4_manual_error_id := 0;
  533: 		axis4_requested_dir := axis4_request_dir_now;
  534: 		axis4_pending_dir := 0;
  535: 		axis4_finish_error := FALSE;
  536: 		axis4_error_id_pending := 0;
  537: 		axis4_manual_state := 10;
  538: 	END_IF
  539: END_IF
  540: 
  541: CASE axis4_manual_state OF
  542: 10:
  543: 	G.axis4_manual_busy := TRUE;
  544: 	spg_disable_exec[4] := TRUE;
  545: 	IF G.SetPointGenDisable[4].Done OR NOT G.SetPointGenDisable[4].Enabled THEN
  546: 		ext_setpoint_enabled[4] := FALSE;
  547: 		axis4_manual_state := 20;
  548: 	END_IF
  549: 
  550: 20:
  551: 	G.axis4_manual_busy := TRUE;
  552: 	axis4_pending_dir := axis4_request_dir_now;
  553: 	IF axis4_requested_dir >= 0 THEN
  554: 		axis4_manual_direction := MC_Direction.MC_Positive_Direction;
  555: 	ELSE
  556: 		axis4_manual_direction := MC_Direction.MC_Negative_Direction;
  557: 	END_IF
  558: 
  559: 	IF axis4_pending_dir = 0 THEN
  560: 		axis4_manual_state := 30;
  561: 	ELSIF axis4_pending_dir <> axis4_requested_dir THEN
  562: 		axis4_manual_state := 30;
  563: 	ELSE
  564: 		axis4_move_exec := TRUE;
  565: 	END_IF
  566: 
  567: 	IF fb_axis4_move_velocity.Error OR fb_axis4_move_velocity.CommandAborted THEN
  568: 		axis4_finish_error := TRUE;
  569: 		axis4_error_id_pending := fb_axis4_move_velocity.ErrorID;
  570: 		G.axis4_manual_error_id := fb_axis4_move_velocity.ErrorID;
  571: 		axis4_pending_dir := 0;
  572: 		axis4_manual_state := 40;
  573: 	END_IF
  574: 
  575: 30:
  576: 	G.axis4_manual_busy := TRUE;
  577: 	axis4_pending_dir := axis4_request_dir_now;
  578: 	axis4_stop_exec := TRUE;
  579: 
  580: 	IF fb_axis4_stop.Error THEN
  581: 		axis4_finish_error := TRUE;
  582: 		axis4_error_id_pending := fb_axis4_stop.ErrorID;
  583: 		axis4_pending_dir := 0;
  584: 		axis4_manual_state := 39;
  585: 	ELSIF fb_axis4_stop.Done OR (ABS(G.axis[4].NcToPlc.ActVelo) <= 0.05) THEN
  586: 		IF axis4_pending_dir <> 0 THEN
  587: 			axis4_requested_dir := axis4_pending_dir;
  588: 			axis4_manual_state := 20;
  589: 		ELSE
  590: 			axis4_requested_dir := 0;
  591: 			axis4_manual_state := 39;
  592: 		END_IF
  593: 	END_IF
  594: 
  595: 39:
  596: G.axis4_manual_busy := TRUE;
  597: // MC_Stop needs one cycle with Execute=FALSE before SetPointGenEnable can get a clean rising edge
  598: axis4_manual_state := 40;
  599: 	
  600: 40:
  601: 	G.axis4_manual_busy := TRUE;
  602: 	G.Act_pos[4] := G.axis[4].NcToPlc.ActPos - G.init_pos[4];
  603: 	G.act_pos_from_left[4] := G.axis[4].NcToPlc.ActPos - G.leftlimit[4];
  604: 	G.refer[4] := G.Act_pos[4];
  605: 	G.ref_slow[4] := G.Act_pos[4];
  606: 	G.act_h[4] := G.Act_pos[4];
  607: 	G.act_hf[4] := G.Act_pos[4];
  608: 	G.refer_from_left[4] := G.act_pos_from_left[4];
  609: 	FOR k:=1 TO 30 DO
  610: 		act_hh[4][k] := G.Act_pos[4];
  611: 	END_FOR
  612: 	act_h_prev[4] := G.Act_pos[4];
  613: 	traj_pos[4] := G.Act_pos[4];
  614: 	traj_vel[4] := 0.0;
  615: 	traj_acc[4] := 0.0;
  616: 	spg_enable_exec[4] := TRUE;
  617: 	IF G.SetPointGenEnable[4].Done THEN
  618: 		ext_setpoint_enabled[4] := TRUE;
  619: 		G.axis4_manual_busy := FALSE;
  620: 		IF axis4_finish_error THEN
  621: 			G.axis4_manual_done := FALSE;
  622: 			G.axis4_manual_error := TRUE;
  623: 			G.axis4_manual_error_id := axis4_error_id_pending;
  624: 		ELSE
  625: 			G.axis4_manual_done := TRUE;
  626: 			G.axis4_manual_error := FALSE;
  627: 			G.axis4_manual_error_id := 0;
  628: 		END_IF
  629: 		axis4_finish_error := FALSE;
  630: 		axis4_error_id_pending := 0;
  631: 		axis4_manual_state := 0;
  632: 	END_IF
  633: END_CASE
  634: 
  635: FOR i:=1 TO 7 DO
  636: 	IF G.power_output[i].Error THEN
  637: 		G.axis_Reset[i].Execute := TRUE;
  638: 	END_IF
  639: END_FOR
  640: 
  641: FOR i:=1 TO 7 DO
  642: 	// 计划回退结束后，状态 31 的预送周期和 SetPointGenEnable 过渡周期都必须喂入当前设定值。
  643: 	// 否则 NC 重新接管时会短暂使用禁用前遗留的左窗口设定值，造成约一个窗口宽度的反向尖峰。
  644: 	IF ext_setpoint_enabled[i] OR (return_state[i] = 31) OR spg_enable_exec[i] THEN
  645: 		IF traj_enable[i] AND NOT G.startup_smoothing_bypass THEN
  646: 			pos_abs := traj_pos[i] + G.init_pos[i];
  647: 			MC_ExtSetPointGenFeed(
  648: 				Position     := pos_abs,
  649: 				Velocity     := traj_vel[i],
  650: 				Acceleration := traj_acc[i],
  651: 				Direction    := 1,
  652: 				Axis         := G.axis[i]);
  653: 		ELSE
  654: 			pos_abs := G.act_h[i] + G.init_pos[i];
  655: 			legacy_vel := (G.act_h[i] - act_h_prev[i]) / 0.001;
  656: 			MC_ExtSetPointGenFeed(
  657: 				Position     := pos_abs,
  658: 				Velocity     := legacy_vel,
  659: 				Acceleration := 0,
  660: 				Direction    := 1,
  661: 				Axis         := G.axis[i]);
  662: 		END_IF
  663: 		G.act_hf[i] := G.act_h[i];
  664: 	ELSE
  665: 		G.act_hf[i] := G.axis[i].NcToPlc.ActPos - G.init_pos[i];
  666: 	END_IF
  667: 	act_h_prev[i] := G.act_h[i];
  668: END_FOR
  669: 
  670: FOR i:=1 TO 7 DO
  671: 	G.SetPointGenEnable[i](Axis:=G.axis[i], Execute:=spg_enable_exec[i]);
  672: 	G.SetPointGenDisable[i](Axis:=G.axis[i], Execute:=spg_disable_exec[i]);
  673: 	fb_return_move_abs[i](
  674: 		Axis:=G.axis[i],
  675: 		Position:=return_target_abs[i],
  676: 		Velocity:=return_velocity[i],
  677: 		Acceleration:=return_acc[i],
  678: 		Deceleration:=return_dec[i],
  679: 		Jerk:=return_jerk[i],
  680: 		Execute:=return_exec[i]
  681: 	);
  682: END_FOR
  683: 
  684: fb_axis4_move_velocity(
  685: 	Axis:=G.axis[4],
  686: 	Execute:=axis4_move_exec,
  687: 	Velocity:=axis4_manual_velocity,
  688: 	Acceleration:=axis4_manual_acc,
  689: 	Deceleration:=axis4_manual_dec,
  690: 	Jerk:=axis4_manual_jerk,
  691: 	Direction:=axis4_manual_direction
  692: );
  693: fb_axis4_stop(
  694: 	Axis:=G.axis[4],
  695: 	Execute:=axis4_stop_exec,
  696: 	Deceleration:=axis4_manual_dec,
  697: 	Jerk:=axis4_manual_jerk
  698: );
  699: 