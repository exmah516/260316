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
   41: 		return_cancelled[i] := FALSE;
   42: 		return_error_id_pending[i] := 0;
   43: 		host_return_stop_active[i] := FALSE;
   44: 		host_return_stop_exec[i] := FALSE;
   45: 		traj_pos[i] := 0.0;
   46: 		traj_vel[i] := 0.0;
   47: 		traj_acc[i] := 0.0;
   48: 		host_hold_abs[i] := 0.0;
   49: 		G.return_cmd[i].Req := FALSE;
   50: 		G.return_cmd[i].Busy := FALSE;
   51: 		G.return_cmd[i].Done := FALSE;
   52: 		G.return_cmd[i].Error := FALSE;
   53: 		G.return_cmd[i].ErrorId := 0;
   54: 	END_FOR
   55: 	host_hold_captured := FALSE;
   56: 	host_timeout_prev := FALSE;
   57: 
   58: 	axis4_manual_state := 0;
   59: 	axis4_request_dir_now := 0;
   60: 	axis4_requested_dir := 0;
   61: 	axis4_pending_dir := 0;
   62: 	axis4_move_exec := FALSE;
   63: 	axis4_stop_exec := FALSE;
   64: 	axis4_finish_error := FALSE;
   65: 	axis4_error_id_pending := 0;
   66: 	G.axis4_fwd_req := FALSE;
   67: 	G.axis4_rev_req := FALSE;
   68: 	G.axis4_manual_busy := FALSE;
   69: 	G.axis4_manual_done := FALSE;
   70: 	G.axis4_manual_error := FALSE;
   71: 	G.axis4_manual_error_id := 0;
   72: END_IF
   73: 
   74: IF NOT init_done THEN
   75: 	G.estop_hold_req := TRUE;
   76: 	G.power_[3].Enable := FALSE;
   77: 	all_enabled := TRUE;
   78: 	FOR i:=1 TO 7 DO
   79: 		IF i = 3 THEN
   80: 			G.SetPointGenEnable[i](Axis:=G.axis[i], Execute:=FALSE);
   81: 		ELSE
   82: 			G.SetPointGenEnable[i](Axis:=G.axis[i], Execute:=TRUE);
   83: 			IF NOT G.SetPointGenEnable[i].Done THEN
   84: 				all_enabled := FALSE;
   85: 			END_IF
   86: 		END_IF
   87: 	END_FOR
   88: 	
   89: 	IF all_enabled THEN
   90: 		FOR j:=1 TO 7 DO
   91: 			G.Act_pos[j] := G.axis[j].NcToPlc.ActPos - G.init_pos[j];
   92: 			G.act_pos_from_left[j] := G.axis[j].NcToPlc.ActPos - G.leftlimit[j];
   93: 			G.ref_slow[j] := G.Act_pos[j];
   94: 			G.act_h[j] := G.Act_pos[j];
   95: 			G.act_hf[j] := G.Act_pos[j];
   96: 			G.refer[j] := G.Act_pos[j];
   97: 			G.refer_from_left[j] := G.act_pos_from_left[j];
   98: 			ext_setpoint_enabled[j] := j <> 3;
   99: 
  100: 			FOR k:=1 TO 30 DO
  101: 				act_hh[j][k] := G.Act_pos[j];
  102: 			END_FOR
  103: 			act_h_prev[j] := G.Act_pos[j];
  104: 			traj_pos[j] := G.Act_pos[j];
  105: 			traj_vel[j] := 0.0;
  106: 			traj_acc[j] := 0.0;
  107: 			IF slimit_enable[j] THEN
  108: 				right_slimit[j] := G.leftlimit[j] + rslimit_distance[j];
  109: 			END_IF
  110: 			G.return_cmd[j].Busy := FALSE;
  111: 			G.return_cmd[j].Done := FALSE;
  112: 			G.return_cmd[j].Error := FALSE;
  113: 			G.return_cmd[j].ErrorId := 0;
  114: 		END_FOR
  115: 
  116: 		G.axis4_manual_busy := FALSE;
  117: 		G.axis4_manual_done := FALSE;
  118: 		G.axis4_manual_error := FALSE;
  119: 		G.axis4_manual_error_id := 0;
  120: 
  121: 		init_done := TRUE;
  122: 		// 通信超时未解除时不提前确认恢复；正式握手后会再次进入本初始化流程。
  123: 		G.handle_reinit_done := NOT G.host_comm_timeout;
  124: 	END_IF
  125: 	RETURN;
  126: END_IF
  127: 
  128: G.estop_hold_req := FALSE;
  129: G.power_[3].Enable := FALSE;
  130: G.power_[5].Enable := TRUE;
  131: 
  132: all_powered := TRUE;
  133: FOR i:=1 TO 7 DO
  134: 	IF i <> 3 AND NOT (G.program_test_mode = 1 AND i = 5) AND NOT G.power_output[i].Done THEN
  135: 		all_powered := FALSE;
  136: 		EXIT;
  137: 	END_IF
  138: END_FOR
  139: 
  140: has_axis_error := FALSE;
  141: FOR i:=1 TO 7 DO
  142: 	IF i = 3 OR (G.program_test_mode = 1 AND i = 5) THEN
  143: 		CONTINUE;
  144: 	END_IF
  145: 	IF G.axis[i].Status.Error OR G.power_output[i].Error OR G.reset_output[i].Error THEN
  146: 		has_axis_error := TRUE;
  147: 		EXIT;
  148: 	END_IF
  149: END_FOR
  150: 
  151: 
  152: IF (NOT all_powered) OR has_axis_error OR G.host_comm_timeout THEN
  153: 	pd_fault := TRUE;
  154: 	ProgramDelivery();
  155: 	ProgramRecord();
  156: END_IF
  157: 
  158: IF NOT all_powered THEN
  159: 	G.estop_hold_req := TRUE;
  160: 	hold_active := TRUE;
  161: 	FOR j:=1 TO 7 DO
  162: 		G.Act_pos[j] := G.axis[j].NcToPlc.ActPos - G.init_pos[j];
  163: 		G.act_pos_from_left[j] := G.axis[j].NcToPlc.ActPos - G.leftlimit[j];
  164: 		G.refer[j] := G.Act_pos[j];
  165: 		G.ref_slow[j] := G.Act_pos[j];
  166: 		G.act_h[j] := G.Act_pos[j];
  167: 		G.act_hf[j] := G.Act_pos[j];
  168: 		G.refer_from_left[j] := G.act_pos_from_left[j];
  169: 		FOR k:=1 TO 30 DO
  170: 			act_hh[j][k] := G.Act_pos[j];
  171: 		END_FOR
  172: 		act_h_prev[j] := G.Act_pos[j];
  173: 		traj_pos[j] := G.Act_pos[j];
  174: 		traj_vel[j] := 0.0;
  175: 		traj_acc[j] := 0.0;
  176: 	END_FOR
  177: 	IF has_axis_error THEN
  178: 		G.gen_state := _err;
  179: 	END_IF
  180: 	RETURN;
  181: END_IF
  182: 
  183: IF hold_active THEN
  184: 	hold_active := FALSE;
  185: 	G.handle_reinit_req := TRUE;
  186: 	G.estop_hold_req := TRUE;
  187: 	IF has_axis_error THEN
  188: 		G.gen_state := _err;
  189: 	END_IF
  190: 	RETURN;
  191: END_IF
  192: 
  193: IF has_axis_error THEN
  194: 	G.estop_hold_req := TRUE;
  195: 	G.gen_state := _err;
  196: 	RETURN;
  197: END_IF
  198: 
  199: IF G.host_comm_timeout THEN
  200: 	G.estop_hold_req := TRUE;
  201: 	G.handle_reinit_done := FALSE;
  202: 
  203: 	IF (NOT host_timeout_prev) OR (NOT host_hold_captured) THEN
  204: 		// 仅在进入超时保持时采样一次。之后始终维持同一绝对位置，
  205: 		// 避免 Actual -> Set Position 的逐周期回灌导致低频来回摆动。
  206: 		FOR i:=1 TO 7 DO
  207: 			host_hold_abs[i] := G.axis[i].NcToPlc.ActPos;
  208: 		END_FOR
  209: 		host_hold_captured := TRUE;
  210: 	END_IF
  211: 
  212: 	IF NOT host_timeout_prev THEN
  213: 		// 仅捕获超时发生时已经在运行的独立运动，禁止恢复后自动续跑。
  214: 		FOR i:=1 TO 7 DO
  215: 			host_return_stop_active[i] :=
  216: 				(return_state[i] <> 0) OR G.return_cmd[i].Busy OR fb_return_move_abs[i].Busy;
  217: 			host_return_stop_exec[i] := FALSE;
  218: 		END_FOR
  219: 		host_axis4_stop_active := (axis4_manual_state <> 0) OR fb_axis4_move_velocity.Busy;
  220: 	END_IF
  221: 
  222: 	// 超时期间持续清除运动请求，避免上位机重连时把旧请求重新送入状态机。
  223: 	G.axis1_fast_return := FALSE;
  224: 	G.axis6_fast_retract := FALSE;
  225: 	G.axis4_fwd_req := FALSE;
  226: 	G.axis4_rev_req := FALSE;
  227: 	G.startup_smoothing_bypass := FALSE;
  228: 	axis4_move_exec := FALSE;
  229: 	host_all_stopped := TRUE;
  230: 
  231: 	FOR i:=1 TO 7 DO
  232: 		spg_enable_exec[i] := FALSE;
  233: 		spg_disable_exec[i] := FALSE;
  234: 		return_exec[i] := FALSE;
  235: 		G.return_cmd[i].Req := FALSE;
  236: 		G.return_cmd[i].Done := FALSE;
  237: 
  238: 		// 实际位置仍实时刷新用于诊断；外部参考始终保持超时边沿的锁存位置。
  239: 		G.Act_pos[i] := G.axis[i].NcToPlc.ActPos - G.init_pos[i];
  240: 		G.act_pos_from_left[i] := G.axis[i].NcToPlc.ActPos - G.leftlimit[i];
  241: 		G.refer[i] := host_hold_abs[i] - G.init_pos[i];
  242: 		G.ref_slow[i] := G.refer[i];
  243: 		G.act_h[i] := G.refer[i];
  244: 		G.act_hf[i] := G.refer[i];
  245: 		G.refer_from_left[i] := host_hold_abs[i] - G.leftlimit[i];
  246: 		FOR k:=1 TO 30 DO
  247: 			act_hh[i][k] := G.refer[i];
  248: 		END_FOR
  249: 		act_h_prev[i] := G.refer[i];
  250: 		traj_pos[i] := G.refer[i];
  251: 		traj_vel[i] := 0.0;
  252: 		traj_acc[i] := 0.0;
  253: 
  254: 		IF host_return_stop_active[i] THEN
  255: 			IF fb_host_return_stop[i].Done THEN
  256: 				host_return_stop_exec[i] := FALSE;
  257: 				host_return_stop_active[i] := FALSE;
  258: 			ELSIF fb_host_return_stop[i].Error THEN
  259: 				// 拉低一个扫描周期清除错误，下一周期继续尝试受控停止。
  260: 				host_return_stop_exec[i] := FALSE;
  261: 				host_all_stopped := FALSE;
  262: 			ELSE
  263: 				host_return_stop_exec[i] := TRUE;
  264: 				host_all_stopped := FALSE;
  265: 			END_IF
  266: 		ELSE
  267: 			host_return_stop_exec[i] := FALSE;
  268: 		END_IF
  269: 		G.return_cmd[i].Busy := host_return_stop_active[i];
  270: 
  271: 		IF ext_setpoint_enabled[i] THEN
  272: 			MC_ExtSetPointGenFeed(
  273: 				Position:=host_hold_abs[i],
  274: 				Velocity:=0.0,
  275: 				Acceleration:=0.0,
  276: 				Direction:=1,
  277: 				Axis:=G.axis[i]);
  278: 		END_IF
  279: 
  280: 		// 所有运动功能块在看门狗分支中仍保持每扫描调用。
  281: 		G.SetPointGenEnable[i](Axis:=G.axis[i], Execute:=FALSE);
  282: 		G.SetPointGenDisable[i](Axis:=G.axis[i], Execute:=FALSE);
  283: 		fb_return_move_abs[i](
  284: 			Axis:=G.axis[i],
  285: 			Position:=return_target_abs[i],
  286: 			Velocity:=return_velocity[i],
  287: 			Acceleration:=return_acc[i],
  288: 			Deceleration:=return_dec[i],
  289: 			Jerk:=return_jerk[i],
  290: 			Execute:=FALSE
  291: 		);
  292: 		fb_host_return_stop[i](
  293: 			Axis:=G.axis[i],
  294: 			Execute:=host_return_stop_exec[i],
  295: 			Deceleration:=100.0,
  296: 			Jerk:=500.0
  297: 		);
  298: 	END_FOR
  299: 
  300: 	IF host_axis4_stop_active THEN
  301: 		IF fb_axis4_stop.Done THEN
  302: 			axis4_stop_exec := FALSE;
  303: 			host_axis4_stop_active := FALSE;
  304: 		ELSIF fb_axis4_stop.Error THEN
  305: 			// 拉低一个扫描周期清除错误，下一周期继续尝试受控停止。
  306: 			axis4_stop_exec := FALSE;
  307: 			host_all_stopped := FALSE;
  308: 		ELSE
  309: 			axis4_stop_exec := TRUE;
  310: 			host_all_stopped := FALSE;
  311: 		END_IF
  312: 	ELSE
  313: 		axis4_stop_exec := FALSE;
  314: 	END_IF
  315: 	G.axis4_manual_busy := host_axis4_stop_active;
  316: 	G.axis4_manual_done := FALSE;
  317: 
  318: 	fb_axis4_move_velocity(
  319: 		Axis:=G.axis[4],
  320: 		Execute:=FALSE,
  321: 		Velocity:=axis4_manual_velocity,
  322: 		Acceleration:=axis4_manual_acc,
  323: 		Deceleration:=axis4_manual_dec,
  324: 		Jerk:=axis4_manual_jerk,
  325: 		Direction:=axis4_manual_direction
  326: 	);
  327: 	fb_axis4_stop(
  328: 		Axis:=G.axis[4],
  329: 		Execute:=axis4_stop_exec,
  330: 		Deceleration:=axis4_manual_dec,
  331: 		Jerk:=axis4_manual_jerk
  332: 	);
  333: 
  334: 	// 清请求并解除超时即为 PLC 对恢复请求的确认；随后复用既有重初始化流程。
  335: 	IF G.host_recover_req AND host_heartbeat_seen AND
  336: 		(NOT host_watchdog_timer.Q) AND host_all_stopped THEN
  337: 		host_timeout_latched := FALSE;
  338: 		G.host_comm_timeout := FALSE;
  339: 		G.host_recover_req := FALSE;
  340: 		G.handle_reinit_req := TRUE;
  341: 	END_IF
  342: 
  343: 	host_timeout_prev := G.host_comm_timeout;
  344: 	RETURN;
  345: END_IF
  346: 
  347: host_timeout_prev := FALSE;
  348: host_hold_captured := FALSE;
  349: // 非超时状态收到的恢复请求没有业务含义，清除以免成为下一次超时的陈旧握手。
  350: G.host_recover_req := FALSE;
  351: 
  352: pd_fault := FALSE;
  353: ProgramDelivery();
  354: ProgramRecord();
  355: 
  356: IF G.cylinder5_press_req THEN
  357: 	G.cylinder5_value := 0;
  358: ELSE
  359: 	G.cylinder5_value := 2000;
  360: END_IF
  361: 
  362: FOR i:=1 TO 7 DO
  363: 	spg_enable_exec[i] := FALSE;
  364: 	spg_disable_exec[i] := FALSE;
  365: 	return_exec[i] := FALSE;
  366: END_FOR
  367: axis4_move_exec := FALSE;
  368: axis4_stop_exec := FALSE;
  369: 
  370: FOR j:=1 TO 7 DO
  371: 	G.Act_pos[j] := G.axis[j].NcToPlc.ActPos - G.init_pos[j];
  372: 	G.act_pos_from_left[j] := G.axis[j].NcToPlc.ActPos - G.leftlimit[j];
  373: 	G.refer_from_left[j] := (G.refer[j] + G.init_pos[j]) - G.leftlimit[j];
  374: 
  375: 	v_step := G.v_limit[j];
  376: 	// 回退接管旁路：回退 Busy 期间跳过斜坡/均值，避免回退末端重新进入慢链路
  377: 	return_handoff_bypass := ((j = 1) AND (G.axis1_fast_return OR G.return_cmd[1].Busy))
  378: 		OR ((j = 6) AND (G.axis6_fast_retract OR G.return_cmd[6].Busy));
  379: 	IF return_handoff_bypass THEN
  380: 		avg_window := 1;
  381: 		ramp_counter[j] := 0;
  382: 	ELSE
  383: 		ramp_counter[j] := MIN(ramp_counter[j] + 1, avg_window_size);
  384: 		avg_window := MAX(1, ramp_counter[j]);
  385: 	END_IF
  386: 
  387: 	FOR k:=2 TO 20 DO
  388: 		act_hh[j][22-k] := act_hh[j][21-k];
  389: 	END_FOR
  390: 
  391: 	IF slimit_enable[j] THEN
  392: 		IF (G.axis[j].NcToPlc.ActPos >= right_slimit[j]) AND (G.refer[j] > G.ref_slow[j]) THEN
  393: 			G.refer[j] := G.ref_slow[j];
  394: 		END_IF
  395: 	END_IF
  396: 
  397: 	IF return_handoff_bypass THEN
  398: 		G.ref_slow[j] := G.refer[j];
  399: 	ELSIF (G.refer[j] - G.ref_slow[j]) > v_step THEN
  400: 		G.ref_slow[j] := G.ref_slow[j] + v_step;
  401: 	ELSIF (G.refer[j] - G.ref_slow[j]) < -v_step THEN
  402: 		G.ref_slow[j] := G.ref_slow[j] - v_step;
  403: 	ELSE
  404: 		G.ref_slow[j] := G.refer[j];
  405: 	END_IF
  406: 
  407: 	IF return_handoff_bypass THEN
  408: 		FOR i:=1 TO 20 DO
  409: 			act_hh[j][i] := G.ref_slow[j];
  410: 		END_FOR
  411: 		G.act_h[j] := G.ref_slow[j];
  412: 	ELSE
  413: 		act_hh[j][1] := G.ref_slow[j];
  414: 		G.act_h[j] := 0;
  415: 		FOR i:=1 TO avg_window DO
  416: 			G.act_h[j] := G.act_h[j] + act_hh[j][i] / TO_LREAL(avg_window);
  417: 		END_FOR
  418: 	END_IF
  419: 
  420: 	IF traj_enable[j] AND NOT return_handoff_bypass AND NOT G.startup_smoothing_bypass THEN
  421: 		acc_raw := traj_wn * traj_wn * (G.refer[j] - traj_pos[j]) - 2.0 * traj_wn * traj_vel[j];
  422: 		IF acc_raw > traj_amax[j] THEN
  423: 			acc_raw := traj_amax[j];
  424: 		ELSIF acc_raw < -traj_amax[j] THEN
  425: 			acc_raw := -traj_amax[j];
  426: 		END_IF
  427: 		traj_acc[j] := acc_raw;
  428: 		traj_vel[j] := traj_vel[j] + 0.001 * traj_acc[j];
  429: 		IF traj_vel[j] > traj_vmax[j] THEN
  430: 			traj_vel[j] := traj_vmax[j];
  431: 		ELSIF traj_vel[j] < -traj_vmax[j] THEN
  432: 			traj_vel[j] := -traj_vmax[j];
  433: 		END_IF
  434: 		traj_pos[j] := traj_pos[j] + 0.001 * traj_vel[j];
  435: 		IF ABS(G.refer[j] - traj_pos[j]) < 0.001 AND ABS(traj_vel[j]) < 0.01 THEN
  436: 			traj_pos[j] := G.refer[j];
  437: 			traj_vel[j] := 0.0;
  438: 			traj_acc[j] := 0.0;
  439: 		END_IF
  440: 	ELSE
  441: 		traj_pos[j] := G.act_h[j];
  442: 		traj_vel[j] := 0.0;
  443: 		traj_acc[j] := 0.0;
  444: 	END_IF
  445: END_FOR
  446: 
  447: FOR i:=1 TO 7 DO
  448: 	IF NOT return_enable[i] AND NOT ((pd_mode <> 0) AND ((i = 2) OR (i = 7))) THEN
  449: 		CONTINUE;
  450: 	END_IF
  451: 
  452: 	IF return_state[i] = 0 THEN
  453: 		G.return_cmd[i].Busy := FALSE;
  454: 		host_return_stop_active[i] := FALSE;
  455: 		host_return_stop_exec[i] := FALSE;
  456: 		IF NOT G.return_cmd[i].Req THEN
  457: 			G.return_cmd[i].Done := FALSE;
  458: 			G.return_cmd[i].Error := FALSE;
  459: 			G.return_cmd[i].ErrorId := 0;
  460: 			return_finish_error[i] := FALSE;
  461: 			return_cancelled[i] := FALSE;
  462: 			return_error_id_pending[i] := 0;
  463: 		ELSIF (NOT G.return_cmd[i].Done) AND (NOT G.return_cmd[i].Error) THEN
  464: 			// Req 由上位机在完整写入参数后最后置位；此处只锁存一次，
  465: 			// 本次 MC_MoveAbsolute 后续不再直接读取可被 ADS 改写的命令参数。
  466: 			return_target_abs[i] := G.return_cmd[i].TargetAbs;
  467: 			return_velocity[i] := G.return_cmd[i].Velocity;
  468: 			return_acc[i] := G.return_cmd[i].Acc;
  469: 			return_dec[i] := G.return_cmd[i].Dec;
  470: 			return_jerk[i] := G.return_cmd[i].Jerk;
  471: 			return_cancelled[i] := FALSE;
  472: 			return_state[i] := 10;
  473: 		END_IF
  474: 	END_IF
  475: 
  476: 	// 上位机在任何尚未完成的阶段撤销请求时，都先受控停止轴。
  477: 	// return_cancelled防止状态40复用30/31收尾后因Req仍为FALSE再次进入停止环。
  478: 	IF ((return_state[i] = 10) OR (return_state[i] = 20) OR
  479: 		(return_state[i] = 30) OR (return_state[i] = 31)) AND
  480: 		(NOT G.return_cmd[i].Req) AND (NOT return_cancelled[i]) THEN
  481: 		return_exec[i] := FALSE;
  482: 		spg_enable_exec[i] := FALSE;
  483: 		spg_disable_exec[i] := FALSE;
  484: 		return_cancelled[i] := TRUE;
  485: 		host_return_stop_active[i] := TRUE;
  486: 		// 清除上一次执行沿；状态 40 首次处理时不会采信旧的 Done/Error。
  487: 		host_return_stop_exec[i] := FALSE;
  488: 		G.return_cmd[i].Done := FALSE;
  489: 		G.return_cmd[i].Error := FALSE;
  490: 		G.return_cmd[i].ErrorId := 0;
  491: 		return_state[i] := 40;
  492: 	END_IF
  493: 
  494: 	CASE return_state[i] OF
  495: 	10:
  496: 		G.return_cmd[i].Busy := TRUE;
  497: 		spg_disable_exec[i] := TRUE;
  498: 		IF G.SetPointGenDisable[i].Done OR NOT G.SetPointGenDisable[i].Enabled THEN
  499: 			ext_setpoint_enabled[i] := FALSE;
  500: 			return_state[i] := 20;
  501: 		END_IF
  502: 
  503: 	20:
  504: 		G.return_cmd[i].Busy := TRUE;
  505: 		return_exec[i] := TRUE;
  506: 		G.refer_from_left[i] := return_target_abs[i] - G.leftlimit[i];
  507: 
  508: 		IF fb_return_move_abs[i].Done OR
  509: 		   ((NOT fb_return_move_abs[i].Busy) AND (ABS(G.axis[i].NcToPlc.ActPos - return_target_abs[i]) <= 0.05)) THEN
  510: 			return_finish_error[i] := FALSE;
  511: 			return_error_id_pending[i] := 0;
  512: 			return_state[i] := 30;
  513: 		ELSIF fb_return_move_abs[i].Error OR fb_return_move_abs[i].CommandAborted THEN
  514: 			return_finish_error[i] := TRUE;
  515: 			return_error_id_pending[i] := fb_return_move_abs[i].ErrorID;
  516: 			return_state[i] := 30;
  517: 		END_IF
  518: 
  519: 	30:
  520: 		G.return_cmd[i].Busy := TRUE;
  521: 		IF ABS(G.axis[i].NcToPlc.ActVelo) <= 0.5 THEN
  522: 			G.Act_pos[i] := G.axis[i].NcToPlc.ActPos - G.init_pos[i];
  523: 			G.act_pos_from_left[i] := G.axis[i].NcToPlc.ActPos - G.leftlimit[i];
  524: 			G.refer[i] := G.Act_pos[i];
  525: 			G.ref_slow[i] := G.Act_pos[i];
  526: 			G.act_h[i] := G.Act_pos[i];
  527: 			G.act_hf[i] := G.Act_pos[i];
  528: 			G.refer_from_left[i] := G.act_pos_from_left[i];
  529: 			FOR k:=1 TO 30 DO
  530: 				act_hh[i][k] := G.Act_pos[i];
  531: 			END_FOR
  532: 			act_h_prev[i] := G.Act_pos[i];
  533: 			traj_pos[i] := G.Act_pos[i];
  534: 			traj_vel[i] := 0.0;
  535: 			traj_acc[i] := 0.0;
  536: 			// 先留出一个完整 PLC 扫描周期预送当前设定值，再重新使能外部设定值发生器。
  537: 			return_state[i] := 31;
  538: 		END_IF
  539: 
  540: 	31:
  541: 		G.return_cmd[i].Busy := TRUE;
  542: 		spg_enable_exec[i] := TRUE;
  543: 		IF G.SetPointGenEnable[i].Done THEN
  544: 			ext_setpoint_enabled[i] := TRUE;
  545: 			G.return_cmd[i].Busy := FALSE;
  546: 			IF return_finish_error[i] THEN
  547: 				G.return_cmd[i].Done := FALSE;
  548: 				G.return_cmd[i].Error := TRUE;
  549: 				G.return_cmd[i].ErrorId := return_error_id_pending[i];
  550: 			ELSIF return_cancelled[i] THEN
  551: 				// 主动取消只表示已经安全停稳，不把取消伪装成原计划动作成功完成。
  552: 				G.return_cmd[i].Done := FALSE;
  553: 				G.return_cmd[i].Error := FALSE;
  554: 				G.return_cmd[i].ErrorId := 0;
  555: 			ELSE
  556: 				G.return_cmd[i].Done := TRUE;
  557: 				G.return_cmd[i].Error := FALSE;
  558: 				G.return_cmd[i].ErrorId := 0;
  559: 			END_IF
  560: 			return_finish_error[i] := FALSE;
  561: 			return_cancelled[i] := FALSE;
  562: 			return_error_id_pending[i] := 0;
  563: 			return_state[i] := 0;
  564: 		END_IF
  565: 
  566: 	40:
  567: 		G.return_cmd[i].Busy := TRUE;
  568: 		G.return_cmd[i].Done := FALSE;
  569: 		return_exec[i] := FALSE;
  570: 		spg_enable_exec[i] := FALSE;
  571: 		spg_disable_exec[i] := FALSE;
  572: 		// 状态40仍以实际停稳为退出条件。若原运动或此前的MC_Stop已经报错，
  573: 		// 在继续受控停止期间保持错误对上位机可见，不能用Busy掩盖故障。
  574: 		IF return_finish_error[i] THEN
  575: 			G.return_cmd[i].Error := TRUE;
  576: 			G.return_cmd[i].ErrorId := return_error_id_pending[i];
  577: 		END_IF
  578: 
  579: 		IF host_return_stop_exec[i] THEN
  580: 			IF fb_host_return_stop[i].Done THEN
  581: 				// 停稳后释放 MC_Stop，并复用状态 30/31 按实际位置重建和恢复外部设定值。
  582: 				host_return_stop_exec[i] := FALSE;
  583: 				host_return_stop_active[i] := FALSE;
  584: 				return_state[i] := 30;
  585: 			ELSIF fb_host_return_stop[i].Error THEN
  586: 				// 原运动错误优先；否则锁存首次MC_Stop错误并立即通知上位机。
  587: 				// Busy和停止任务保持有效，Execute拉低一拍后仍继续尝试受控停止，
  588: 				// 只有真实Done才能进入位置重建，绝不把停止失败伪装成完成。
  589: 				IF NOT return_finish_error[i] THEN
  590: 					return_finish_error[i] := TRUE;
  591: 					return_error_id_pending[i] := fb_host_return_stop[i].ErrorID;
  592: 				END_IF
  593: 				G.return_cmd[i].Error := TRUE;
  594: 				G.return_cmd[i].ErrorId := return_error_id_pending[i];
  595: 				host_return_stop_exec[i] := FALSE;
  596: 			END_IF
  597: 		ELSIF NOT fb_host_return_stop[i].Error THEN
  598: 			// 首次进入或错误已由低电平清除后，启动（或重试）受控停止。
  599: 			host_return_stop_exec[i] := TRUE;
  600: 		END_IF
  601: 	END_CASE
  602: END_FOR
  603: 
  604: axis4_request_dir_now := 0;
  605: IF G.axis4_fwd_req AND NOT G.axis4_rev_req THEN
  606: 	axis4_request_dir_now := 1;
  607: ELSIF G.axis4_rev_req AND NOT G.axis4_fwd_req THEN
  608: 	axis4_request_dir_now := -1;
  609: END_IF
  610: 
  611: IF axis4_manual_state = 0 THEN
  612: 	G.axis4_manual_busy := FALSE;
  613: 	IF axis4_request_dir_now = 0 THEN
  614: 		G.axis4_manual_done := FALSE;
  615: 		G.axis4_manual_error := FALSE;
  616: 		G.axis4_manual_error_id := 0;
  617: 	ELSIF NOT G.axis4_manual_error THEN
  618: 		G.axis4_manual_done := FALSE;
  619: 		G.axis4_manual_error := FALSE;
  620: 		G.axis4_manual_error_id := 0;
  621: 		axis4_requested_dir := axis4_request_dir_now;
  622: 		axis4_pending_dir := 0;
  623: 		axis4_finish_error := FALSE;
  624: 		axis4_error_id_pending := 0;
  625: 		axis4_manual_state := 10;
  626: 	END_IF
  627: END_IF
  628: 
  629: CASE axis4_manual_state OF
  630: 10:
  631: 	G.axis4_manual_busy := TRUE;
  632: 	spg_disable_exec[4] := TRUE;
  633: 	IF G.SetPointGenDisable[4].Done OR NOT G.SetPointGenDisable[4].Enabled THEN
  634: 		ext_setpoint_enabled[4] := FALSE;
  635: 		axis4_manual_state := 20;
  636: 	END_IF
  637: 
  638: 20:
  639: 	G.axis4_manual_busy := TRUE;
  640: 	axis4_pending_dir := axis4_request_dir_now;
  641: 	IF axis4_requested_dir >= 0 THEN
  642: 		axis4_manual_direction := MC_Direction.MC_Positive_Direction;
  643: 	ELSE
  644: 		axis4_manual_direction := MC_Direction.MC_Negative_Direction;
  645: 	END_IF
  646: 
  647: 	IF axis4_pending_dir = 0 THEN
  648: 		axis4_manual_state := 30;
  649: 	ELSIF axis4_pending_dir <> axis4_requested_dir THEN
  650: 		axis4_manual_state := 30;
  651: 	ELSE
  652: 		axis4_move_exec := TRUE;
  653: 	END_IF
  654: 
  655: 	IF fb_axis4_move_velocity.Error OR fb_axis4_move_velocity.CommandAborted THEN
  656: 		axis4_finish_error := TRUE;
  657: 		axis4_error_id_pending := fb_axis4_move_velocity.ErrorID;
  658: 		G.axis4_manual_error_id := fb_axis4_move_velocity.ErrorID;
  659: 		axis4_pending_dir := 0;
  660: 		axis4_manual_state := 40;
  661: 	END_IF
  662: 
  663: 30:
  664: 	G.axis4_manual_busy := TRUE;
  665: 	axis4_pending_dir := axis4_request_dir_now;
  666: 	axis4_stop_exec := TRUE;
  667: 
  668: 	IF fb_axis4_stop.Error THEN
  669: 		axis4_finish_error := TRUE;
  670: 		axis4_error_id_pending := fb_axis4_stop.ErrorID;
  671: 		axis4_pending_dir := 0;
  672: 		axis4_manual_state := 39;
  673: 	ELSIF fb_axis4_stop.Done OR (ABS(G.axis[4].NcToPlc.ActVelo) <= 0.05) THEN
  674: 		IF axis4_pending_dir <> 0 THEN
  675: 			axis4_requested_dir := axis4_pending_dir;
  676: 			axis4_manual_state := 20;
  677: 		ELSE
  678: 			axis4_requested_dir := 0;
  679: 			axis4_manual_state := 39;
  680: 		END_IF
  681: 	END_IF
  682: 
  683: 39:
  684: G.axis4_manual_busy := TRUE;
  685: // MC_Stop needs one cycle with Execute=FALSE before SetPointGenEnable can get a clean rising edge
  686: axis4_manual_state := 40;
  687: 	
  688: 40:
  689: 	G.axis4_manual_busy := TRUE;
  690: 	G.Act_pos[4] := G.axis[4].NcToPlc.ActPos - G.init_pos[4];
  691: 	G.act_pos_from_left[4] := G.axis[4].NcToPlc.ActPos - G.leftlimit[4];
  692: 	G.refer[4] := G.Act_pos[4];
  693: 	G.ref_slow[4] := G.Act_pos[4];
  694: 	G.act_h[4] := G.Act_pos[4];
  695: 	G.act_hf[4] := G.Act_pos[4];
  696: 	G.refer_from_left[4] := G.act_pos_from_left[4];
  697: 	FOR k:=1 TO 30 DO
  698: 		act_hh[4][k] := G.Act_pos[4];
  699: 	END_FOR
  700: 	act_h_prev[4] := G.Act_pos[4];
  701: 	traj_pos[4] := G.Act_pos[4];
  702: 	traj_vel[4] := 0.0;
  703: 	traj_acc[4] := 0.0;
  704: 	spg_enable_exec[4] := TRUE;
  705: 	IF G.SetPointGenEnable[4].Done THEN
  706: 		ext_setpoint_enabled[4] := TRUE;
  707: 		G.axis4_manual_busy := FALSE;
  708: 		IF axis4_finish_error THEN
  709: 			G.axis4_manual_done := FALSE;
  710: 			G.axis4_manual_error := TRUE;
  711: 			G.axis4_manual_error_id := axis4_error_id_pending;
  712: 		ELSE
  713: 			G.axis4_manual_done := TRUE;
  714: 			G.axis4_manual_error := FALSE;
  715: 			G.axis4_manual_error_id := 0;
  716: 		END_IF
  717: 		axis4_finish_error := FALSE;
  718: 		axis4_error_id_pending := 0;
  719: 		axis4_manual_state := 0;
  720: 	END_IF
  721: END_CASE
  722: 
  723: FOR i:=1 TO 7 DO
  724: 	IF i <> 3 AND G.power_output[i].Error THEN
  725: 		G.axis_Reset[i].Execute := TRUE;
  726: 	END_IF
  727: END_FOR
  728: 
  729: FOR i:=1 TO 7 DO
  730: 	IF i = 3 THEN
  731: 		G.act_hf[i] := G.axis[i].NcToPlc.ActPos - G.init_pos[i];
  732: 	ELSIF ext_setpoint_enabled[i] OR (return_state[i] = 31) OR spg_enable_exec[i] THEN
  733: 		IF traj_enable[i] AND NOT G.startup_smoothing_bypass THEN
  734: 			pos_abs := traj_pos[i] + G.init_pos[i];
  735: 			MC_ExtSetPointGenFeed(
  736: 				Position     := pos_abs,
  737: 				Velocity     := traj_vel[i],
  738: 				Acceleration := traj_acc[i],
  739: 				Direction    := 1,
  740: 				Axis         := G.axis[i]);
  741: 		ELSE
  742: 			pos_abs := G.act_h[i] + G.init_pos[i];
  743: 			legacy_vel := (G.act_h[i] - act_h_prev[i]) / 0.001;
  744: 			MC_ExtSetPointGenFeed(
  745: 				Position     := pos_abs,
  746: 				Velocity     := legacy_vel,
  747: 				Acceleration := 0,
  748: 				Direction    := 1,
  749: 				Axis         := G.axis[i]);
  750: 		END_IF
  751: 		G.act_hf[i] := G.act_h[i];
  752: 	ELSE
  753: 		G.act_hf[i] := G.axis[i].NcToPlc.ActPos - G.init_pos[i];
  754: 	END_IF
  755: 	act_h_prev[i] := G.act_h[i];
  756: END_FOR
  757: 
  758: FOR i:=1 TO 7 DO
  759: 	G.SetPointGenEnable[i](Axis:=G.axis[i], Execute:=spg_enable_exec[i]);
  760: 	G.SetPointGenDisable[i](Axis:=G.axis[i], Execute:=spg_disable_exec[i]);
  761: 	fb_return_move_abs[i](
  762: 		Axis:=G.axis[i],
  763: 		Position:=return_target_abs[i],
  764: 		Velocity:=return_velocity[i],
  765: 		Acceleration:=return_acc[i],
  766: 		Deceleration:=return_dec[i],
  767: 		Jerk:=return_jerk[i],
  768: 		Execute:=return_exec[i]
  769: 	);
  770: 	fb_host_return_stop[i](
  771: 		Axis:=G.axis[i],
  772: 		Execute:=host_return_stop_exec[i],
  773: 		Deceleration:=100.0,
  774: 		Jerk:=500.0
  775: 	);
  776: END_FOR
  777: 
  778: fb_axis4_move_velocity(
  779: 	Axis:=G.axis[4],
  780: 	Execute:=axis4_move_exec,
  781: 	Velocity:=axis4_manual_velocity,
  782: 	Acceleration:=axis4_manual_acc,
  783: 	Deceleration:=axis4_manual_dec,
  784: 	Jerk:=axis4_manual_jerk,
  785: 	Direction:=axis4_manual_direction
  786: );
  787: fb_axis4_stop(
  788: 	Axis:=G.axis[4],
  789: 	Execute:=axis4_stop_exec,
  790: 	Deceleration:=axis4_manual_dec,
  791: 	Jerk:=axis4_manual_jerk
  792: );
  793: 