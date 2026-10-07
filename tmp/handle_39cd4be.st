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
   76: 	all_enabled := TRUE;
   77: 	FOR i:=1 TO 7 DO
   78: 		G.SetPointGenEnable[i](Axis:=G.axis[i], Execute:=TRUE);
   79: 		IF NOT G.SetPointGenEnable[i].Done THEN
   80: 			all_enabled := FALSE;
   81: 		END_IF
   82: 	END_FOR
   83: 	
   84: 	IF all_enabled THEN
   85: 		FOR j:=1 TO 7 DO
   86: 			G.Act_pos[j] := G.axis[j].NcToPlc.ActPos - G.init_pos[j];
   87: 			G.act_pos_from_left[j] := G.axis[j].NcToPlc.ActPos - G.leftlimit[j];
   88: 			G.ref_slow[j] := G.Act_pos[j];
   89: 			G.act_h[j] := G.Act_pos[j];
   90: 			G.act_hf[j] := G.Act_pos[j];
   91: 			G.refer[j] := G.Act_pos[j];
   92: 			G.refer_from_left[j] := G.act_pos_from_left[j];
   93: 			ext_setpoint_enabled[j] := TRUE;
   94: 
   95: 			FOR k:=1 TO 30 DO
   96: 				act_hh[j][k] := G.Act_pos[j];
   97: 			END_FOR
   98: 			act_h_prev[j] := G.Act_pos[j];
   99: 			traj_pos[j] := G.Act_pos[j];
  100: 			traj_vel[j] := 0.0;
  101: 			traj_acc[j] := 0.0;
  102: 			IF slimit_enable[j] THEN
  103: 				right_slimit[j] := G.leftlimit[j] + rslimit_distance[j];
  104: 			END_IF
  105: 			G.return_cmd[j].Busy := FALSE;
  106: 			G.return_cmd[j].Done := FALSE;
  107: 			G.return_cmd[j].Error := FALSE;
  108: 			G.return_cmd[j].ErrorId := 0;
  109: 		END_FOR
  110: 
  111: 		G.axis4_manual_busy := FALSE;
  112: 		G.axis4_manual_done := FALSE;
  113: 		G.axis4_manual_error := FALSE;
  114: 		G.axis4_manual_error_id := 0;
  115: 
  116: 		init_done := TRUE;
  117: 		// 通信超时未解除时不提前确认恢复；正式握手后会再次进入本初始化流程。
  118: 		G.handle_reinit_done := NOT G.host_comm_timeout;
  119: 	END_IF
  120: 	RETURN;
  121: END_IF
  122: 
  123: all_powered := TRUE;
  124: FOR i:=1 TO 7 DO
  125: 	IF NOT G.power_output[i].Done THEN
  126: 		all_powered := FALSE;
  127: 		EXIT;
  128: 	END_IF
  129: END_FOR
  130: 
  131: has_axis_error := FALSE;
  132: FOR i:=1 TO 7 DO
  133: 	IF G.axis[i].Status.Error OR G.power_output[i].Error OR G.reset_output[i].Error THEN
  134: 		has_axis_error := TRUE;
  135: 		EXIT;
  136: 	END_IF
  137: END_FOR
  138: 
  139: IF NOT all_powered THEN
  140: 	G.estop_hold_req := TRUE;
  141: 	hold_active := TRUE;
  142: 	FOR j:=1 TO 7 DO
  143: 		G.Act_pos[j] := G.axis[j].NcToPlc.ActPos - G.init_pos[j];
  144: 		G.act_pos_from_left[j] := G.axis[j].NcToPlc.ActPos - G.leftlimit[j];
  145: 		G.refer[j] := G.Act_pos[j];
  146: 		G.ref_slow[j] := G.Act_pos[j];
  147: 		G.act_h[j] := G.Act_pos[j];
  148: 		G.act_hf[j] := G.Act_pos[j];
  149: 		G.refer_from_left[j] := G.act_pos_from_left[j];
  150: 		FOR k:=1 TO 30 DO
  151: 			act_hh[j][k] := G.Act_pos[j];
  152: 		END_FOR
  153: 		act_h_prev[j] := G.Act_pos[j];
  154: 		traj_pos[j] := G.Act_pos[j];
  155: 		traj_vel[j] := 0.0;
  156: 		traj_acc[j] := 0.0;
  157: 	END_FOR
  158: 	IF has_axis_error THEN
  159: 		G.gen_state := _err;
  160: 	END_IF
  161: 	RETURN;
  162: END_IF
  163: 
  164: IF hold_active THEN
  165: 	hold_active := FALSE;
  166: 	G.handle_reinit_req := TRUE;
  167: 	G.estop_hold_req := TRUE;
  168: 	IF has_axis_error THEN
  169: 		G.gen_state := _err;
  170: 	END_IF
  171: 	RETURN;
  172: END_IF
  173: 
  174: IF has_axis_error THEN
  175: 	G.estop_hold_req := TRUE;
  176: 	G.gen_state := _err;
  177: 	RETURN;
  178: END_IF
  179: 
  180: IF G.host_comm_timeout THEN
  181: 	G.estop_hold_req := TRUE;
  182: 	G.handle_reinit_done := FALSE;
  183: 
  184: 	IF (NOT host_timeout_prev) OR (NOT host_hold_captured) THEN
  185: 		// 仅在进入超时保持时采样一次。之后始终维持同一绝对位置，
  186: 		// 避免 Actual -> Set Position 的逐周期回灌导致低频来回摆动。
  187: 		FOR i:=1 TO 7 DO
  188: 			host_hold_abs[i] := G.axis[i].NcToPlc.ActPos;
  189: 		END_FOR
  190: 		host_hold_captured := TRUE;
  191: 	END_IF
  192: 
  193: 	IF NOT host_timeout_prev THEN
  194: 		// 仅捕获超时发生时已经在运行的独立运动，禁止恢复后自动续跑。
  195: 		FOR i:=1 TO 7 DO
  196: 			host_return_stop_active[i] := return_enable[i] AND
  197: 				((return_state[i] <> 0) OR G.return_cmd[i].Busy OR fb_return_move_abs[i].Busy);
  198: 			host_return_stop_exec[i] := FALSE;
  199: 		END_FOR
  200: 		host_axis4_stop_active := (axis4_manual_state <> 0) OR fb_axis4_move_velocity.Busy;
  201: 	END_IF
  202: 
  203: 	// 超时期间持续清除运动请求，避免上位机重连时把旧请求重新送入状态机。
  204: 	G.axis1_fast_return := FALSE;
  205: 	G.axis6_fast_retract := FALSE;
  206: 	G.axis4_fwd_req := FALSE;
  207: 	G.axis4_rev_req := FALSE;
  208: 	G.startup_smoothing_bypass := FALSE;
  209: 	axis4_move_exec := FALSE;
  210: 	host_all_stopped := TRUE;
  211: 
  212: 	FOR i:=1 TO 7 DO
  213: 		spg_enable_exec[i] := FALSE;
  214: 		spg_disable_exec[i] := FALSE;
  215: 		return_exec[i] := FALSE;
  216: 		G.return_cmd[i].Req := FALSE;
  217: 		G.return_cmd[i].Done := FALSE;
  218: 
  219: 		// 实际位置仍实时刷新用于诊断；外部参考始终保持超时边沿的锁存位置。
  220: 		G.Act_pos[i] := G.axis[i].NcToPlc.ActPos - G.init_pos[i];
  221: 		G.act_pos_from_left[i] := G.axis[i].NcToPlc.ActPos - G.leftlimit[i];
  222: 		G.refer[i] := host_hold_abs[i] - G.init_pos[i];
  223: 		G.ref_slow[i] := G.refer[i];
  224: 		G.act_h[i] := G.refer[i];
  225: 		G.act_hf[i] := G.refer[i];
  226: 		G.refer_from_left[i] := host_hold_abs[i] - G.leftlimit[i];
  227: 		FOR k:=1 TO 30 DO
  228: 			act_hh[i][k] := G.refer[i];
  229: 		END_FOR
  230: 		act_h_prev[i] := G.refer[i];
  231: 		traj_pos[i] := G.refer[i];
  232: 		traj_vel[i] := 0.0;
  233: 		traj_acc[i] := 0.0;
  234: 
  235: 		IF host_return_stop_active[i] THEN
  236: 			IF fb_host_return_stop[i].Done THEN
  237: 				host_return_stop_exec[i] := FALSE;
  238: 				host_return_stop_active[i] := FALSE;
  239: 			ELSIF fb_host_return_stop[i].Error THEN
  240: 				// 拉低一个扫描周期清除错误，下一周期继续尝试受控停止。
  241: 				host_return_stop_exec[i] := FALSE;
  242: 				host_all_stopped := FALSE;
  243: 			ELSE
  244: 				host_return_stop_exec[i] := TRUE;
  245: 				host_all_stopped := FALSE;
  246: 			END_IF
  247: 		ELSE
  248: 			host_return_stop_exec[i] := FALSE;
  249: 		END_IF
  250: 		G.return_cmd[i].Busy := host_return_stop_active[i];
  251: 
  252: 		IF ext_setpoint_enabled[i] THEN
  253: 			MC_ExtSetPointGenFeed(
  254: 				Position:=host_hold_abs[i],
  255: 				Velocity:=0.0,
  256: 				Acceleration:=0.0,
  257: 				Direction:=1,
  258: 				Axis:=G.axis[i]);
  259: 		END_IF
  260: 
  261: 		// 所有运动功能块在看门狗分支中仍保持每扫描调用。
  262: 		G.SetPointGenEnable[i](Axis:=G.axis[i], Execute:=FALSE);
  263: 		G.SetPointGenDisable[i](Axis:=G.axis[i], Execute:=FALSE);
  264: 		fb_return_move_abs[i](
  265: 			Axis:=G.axis[i],
  266: 			Position:=return_target_abs[i],
  267: 			Velocity:=return_velocity[i],
  268: 			Acceleration:=return_acc[i],
  269: 			Deceleration:=return_dec[i],
  270: 			Jerk:=return_jerk[i],
  271: 			Execute:=FALSE
  272: 		);
  273: 		fb_host_return_stop[i](
  274: 			Axis:=G.axis[i],
  275: 			Execute:=host_return_stop_exec[i],
  276: 			Deceleration:=100.0,
  277: 			Jerk:=500.0
  278: 		);
  279: 	END_FOR
  280: 
  281: 	IF host_axis4_stop_active THEN
  282: 		IF fb_axis4_stop.Done THEN
  283: 			axis4_stop_exec := FALSE;
  284: 			host_axis4_stop_active := FALSE;
  285: 		ELSIF fb_axis4_stop.Error THEN
  286: 			// 拉低一个扫描周期清除错误，下一周期继续尝试受控停止。
  287: 			axis4_stop_exec := FALSE;
  288: 			host_all_stopped := FALSE;
  289: 		ELSE
  290: 			axis4_stop_exec := TRUE;
  291: 			host_all_stopped := FALSE;
  292: 		END_IF
  293: 	ELSE
  294: 		axis4_stop_exec := FALSE;
  295: 	END_IF
  296: 	G.axis4_manual_busy := host_axis4_stop_active;
  297: 	G.axis4_manual_done := FALSE;
  298: 
  299: 	fb_axis4_move_velocity(
  300: 		Axis:=G.axis[4],
  301: 		Execute:=FALSE,
  302: 		Velocity:=axis4_manual_velocity,
  303: 		Acceleration:=axis4_manual_acc,
  304: 		Deceleration:=axis4_manual_dec,
  305: 		Jerk:=axis4_manual_jerk,
  306: 		Direction:=axis4_manual_direction
  307: 	);
  308: 	fb_axis4_stop(
  309: 		Axis:=G.axis[4],
  310: 		Execute:=axis4_stop_exec,
  311: 		Deceleration:=axis4_manual_dec,
  312: 		Jerk:=axis4_manual_jerk
  313: 	);
  314: 
  315: 	// 清请求并解除超时即为 PLC 对恢复请求的确认；随后复用既有重初始化流程。
  316: 	IF G.host_recover_req AND host_heartbeat_seen AND
  317: 		(NOT host_watchdog_timer.Q) AND host_all_stopped THEN
  318: 		host_timeout_latched := FALSE;
  319: 		G.host_comm_timeout := FALSE;
  320: 		G.host_recover_req := FALSE;
  321: 		G.handle_reinit_req := TRUE;
  322: 	END_IF
  323: 
  324: 	host_timeout_prev := G.host_comm_timeout;
  325: 	RETURN;
  326: END_IF
  327: 
  328: host_timeout_prev := FALSE;
  329: host_hold_captured := FALSE;
  330: // 非超时状态收到的恢复请求没有业务含义，清除以免成为下一次超时的陈旧握手。
  331: G.host_recover_req := FALSE;
  332: 
  333: G.estop_hold_req := FALSE;
  334: 
  335: IF G.cylinder5_press_req THEN
  336: 	G.cylinder5_value := 0;
  337: ELSE
  338: 	G.cylinder5_value := 2000;
  339: END_IF
  340: 
  341: FOR i:=1 TO 7 DO
  342: 	spg_enable_exec[i] := FALSE;
  343: 	spg_disable_exec[i] := FALSE;
  344: 	return_exec[i] := FALSE;
  345: END_FOR
  346: axis4_move_exec := FALSE;
  347: axis4_stop_exec := FALSE;
  348: 
  349: FOR j:=1 TO 7 DO
  350: 	G.Act_pos[j] := G.axis[j].NcToPlc.ActPos - G.init_pos[j];
  351: 	G.act_pos_from_left[j] := G.axis[j].NcToPlc.ActPos - G.leftlimit[j];
  352: 	G.refer_from_left[j] := (G.refer[j] + G.init_pos[j]) - G.leftlimit[j];
  353: 
  354: 	v_step := G.v_limit[j];
  355: 	// 回退接管旁路：回退 Busy 期间跳过斜坡/均值，避免回退末端重新进入慢链路
  356: 	return_handoff_bypass := ((j = 1) AND (G.axis1_fast_return OR G.return_cmd[1].Busy))
  357: 		OR ((j = 6) AND (G.axis6_fast_retract OR G.return_cmd[6].Busy));
  358: 	IF return_handoff_bypass THEN
  359: 		avg_window := 1;
  360: 		ramp_counter[j] := 0;
  361: 	ELSE
  362: 		ramp_counter[j] := MIN(ramp_counter[j] + 1, avg_window_size);
  363: 		avg_window := MAX(1, ramp_counter[j]);
  364: 	END_IF
  365: 
  366: 	FOR k:=2 TO 20 DO
  367: 		act_hh[j][22-k] := act_hh[j][21-k];
  368: 	END_FOR
  369: 
  370: 	IF slimit_enable[j] THEN
  371: 		IF (G.axis[j].NcToPlc.ActPos >= right_slimit[j]) AND (G.refer[j] > G.ref_slow[j]) THEN
  372: 			G.refer[j] := G.ref_slow[j];
  373: 		END_IF
  374: 	END_IF
  375: 
  376: 	IF return_handoff_bypass THEN
  377: 		G.ref_slow[j] := G.refer[j];
  378: 	ELSIF (G.refer[j] - G.ref_slow[j]) > v_step THEN
  379: 		G.ref_slow[j] := G.ref_slow[j] + v_step;
  380: 	ELSIF (G.refer[j] - G.ref_slow[j]) < -v_step THEN
  381: 		G.ref_slow[j] := G.ref_slow[j] - v_step;
  382: 	ELSE
  383: 		G.ref_slow[j] := G.refer[j];
  384: 	END_IF
  385: 
  386: 	IF return_handoff_bypass THEN
  387: 		FOR i:=1 TO 20 DO
  388: 			act_hh[j][i] := G.ref_slow[j];
  389: 		END_FOR
  390: 		G.act_h[j] := G.ref_slow[j];
  391: 	ELSE
  392: 		act_hh[j][1] := G.ref_slow[j];
  393: 		G.act_h[j] := 0;
  394: 		FOR i:=1 TO avg_window DO
  395: 			G.act_h[j] := G.act_h[j] + act_hh[j][i] / TO_LREAL(avg_window);
  396: 		END_FOR
  397: 	END_IF
  398: 
  399: 	IF traj_enable[j] AND NOT return_handoff_bypass AND NOT G.startup_smoothing_bypass THEN
  400: 		acc_raw := traj_wn * traj_wn * (G.refer[j] - traj_pos[j]) - 2.0 * traj_wn * traj_vel[j];
  401: 		IF acc_raw > traj_amax[j] THEN
  402: 			acc_raw := traj_amax[j];
  403: 		ELSIF acc_raw < -traj_amax[j] THEN
  404: 			acc_raw := -traj_amax[j];
  405: 		END_IF
  406: 		traj_acc[j] := acc_raw;
  407: 		traj_vel[j] := traj_vel[j] + 0.001 * traj_acc[j];
  408: 		IF traj_vel[j] > traj_vmax[j] THEN
  409: 			traj_vel[j] := traj_vmax[j];
  410: 		ELSIF traj_vel[j] < -traj_vmax[j] THEN
  411: 			traj_vel[j] := -traj_vmax[j];
  412: 		END_IF
  413: 		traj_pos[j] := traj_pos[j] + 0.001 * traj_vel[j];
  414: 		IF ABS(G.refer[j] - traj_pos[j]) < 0.001 AND ABS(traj_vel[j]) < 0.01 THEN
  415: 			traj_pos[j] := G.refer[j];
  416: 			traj_vel[j] := 0.0;
  417: 			traj_acc[j] := 0.0;
  418: 		END_IF
  419: 	ELSE
  420: 		traj_pos[j] := G.act_h[j];
  421: 		traj_vel[j] := 0.0;
  422: 		traj_acc[j] := 0.0;
  423: 	END_IF
  424: END_FOR
  425: 
  426: FOR i:=1 TO 7 DO
  427: 	IF NOT return_enable[i] THEN
  428: 		CONTINUE;
  429: 	END_IF
  430: 
  431: 	IF return_state[i] = 0 THEN
  432: 		G.return_cmd[i].Busy := FALSE;
  433: 		host_return_stop_active[i] := FALSE;
  434: 		host_return_stop_exec[i] := FALSE;
  435: 		IF NOT G.return_cmd[i].Req THEN
  436: 			G.return_cmd[i].Done := FALSE;
  437: 			G.return_cmd[i].Error := FALSE;
  438: 			G.return_cmd[i].ErrorId := 0;
  439: 			return_finish_error[i] := FALSE;
  440: 			return_cancelled[i] := FALSE;
  441: 			return_error_id_pending[i] := 0;
  442: 		ELSIF (NOT G.return_cmd[i].Done) AND (NOT G.return_cmd[i].Error) THEN
  443: 			// Req 由上位机在完整写入参数后最后置位；此处只锁存一次，
  444: 			// 本次 MC_MoveAbsolute 后续不再直接读取可被 ADS 改写的命令参数。
  445: 			return_target_abs[i] := G.return_cmd[i].TargetAbs;
  446: 			return_velocity[i] := G.return_cmd[i].Velocity;
  447: 			return_acc[i] := G.return_cmd[i].Acc;
  448: 			return_dec[i] := G.return_cmd[i].Dec;
  449: 			return_jerk[i] := G.return_cmd[i].Jerk;
  450: 			return_cancelled[i] := FALSE;
  451: 			return_state[i] := 10;
  452: 		END_IF
  453: 	END_IF
  454: 
  455: 	// 上位机在任何尚未完成的阶段撤销请求时，都先受控停止轴。
  456: 	// return_cancelled防止状态40复用30/31收尾后因Req仍为FALSE再次进入停止环。
  457: 	IF ((return_state[i] = 10) OR (return_state[i] = 20) OR
  458: 		(return_state[i] = 30) OR (return_state[i] = 31)) AND
  459: 		(NOT G.return_cmd[i].Req) AND (NOT return_cancelled[i]) THEN
  460: 		return_exec[i] := FALSE;
  461: 		spg_enable_exec[i] := FALSE;
  462: 		spg_disable_exec[i] := FALSE;
  463: 		return_cancelled[i] := TRUE;
  464: 		host_return_stop_active[i] := TRUE;
  465: 		// 清除上一次执行沿；状态 40 首次处理时不会采信旧的 Done/Error。
  466: 		host_return_stop_exec[i] := FALSE;
  467: 		G.return_cmd[i].Done := FALSE;
  468: 		G.return_cmd[i].Error := FALSE;
  469: 		G.return_cmd[i].ErrorId := 0;
  470: 		return_state[i] := 40;
  471: 	END_IF
  472: 
  473: 	CASE return_state[i] OF
  474: 	10:
  475: 		G.return_cmd[i].Busy := TRUE;
  476: 		spg_disable_exec[i] := TRUE;
  477: 		IF G.SetPointGenDisable[i].Done OR NOT G.SetPointGenDisable[i].Enabled THEN
  478: 			ext_setpoint_enabled[i] := FALSE;
  479: 			return_state[i] := 20;
  480: 		END_IF
  481: 
  482: 	20:
  483: 		G.return_cmd[i].Busy := TRUE;
  484: 		return_exec[i] := TRUE;
  485: 		G.refer_from_left[i] := return_target_abs[i] - G.leftlimit[i];
  486: 
  487: 		IF fb_return_move_abs[i].Done OR
  488: 		   ((NOT fb_return_move_abs[i].Busy) AND (ABS(G.axis[i].NcToPlc.ActPos - return_target_abs[i]) <= 0.05)) THEN
  489: 			return_finish_error[i] := FALSE;
  490: 			return_error_id_pending[i] := 0;
  491: 			return_state[i] := 30;
  492: 		ELSIF fb_return_move_abs[i].Error OR fb_return_move_abs[i].CommandAborted THEN
  493: 			return_finish_error[i] := TRUE;
  494: 			return_error_id_pending[i] := fb_return_move_abs[i].ErrorID;
  495: 			return_state[i] := 30;
  496: 		END_IF
  497: 
  498: 	30:
  499: 		G.return_cmd[i].Busy := TRUE;
  500: 		IF ABS(G.axis[i].NcToPlc.ActVelo) <= 0.5 THEN
  501: 			G.Act_pos[i] := G.axis[i].NcToPlc.ActPos - G.init_pos[i];
  502: 			G.act_pos_from_left[i] := G.axis[i].NcToPlc.ActPos - G.leftlimit[i];
  503: 			G.refer[i] := G.Act_pos[i];
  504: 			G.ref_slow[i] := G.Act_pos[i];
  505: 			G.act_h[i] := G.Act_pos[i];
  506: 			G.act_hf[i] := G.Act_pos[i];
  507: 			G.refer_from_left[i] := G.act_pos_from_left[i];
  508: 			FOR k:=1 TO 30 DO
  509: 				act_hh[i][k] := G.Act_pos[i];
  510: 			END_FOR
  511: 			act_h_prev[i] := G.Act_pos[i];
  512: 			traj_pos[i] := G.Act_pos[i];
  513: 			traj_vel[i] := 0.0;
  514: 			traj_acc[i] := 0.0;
  515: 			// 先留出一个完整 PLC 扫描周期预送当前设定值，再重新使能外部设定值发生器。
  516: 			return_state[i] := 31;
  517: 		END_IF
  518: 
  519: 	31:
  520: 		G.return_cmd[i].Busy := TRUE;
  521: 		spg_enable_exec[i] := TRUE;
  522: 		IF G.SetPointGenEnable[i].Done THEN
  523: 			ext_setpoint_enabled[i] := TRUE;
  524: 			G.return_cmd[i].Busy := FALSE;
  525: 			IF return_finish_error[i] THEN
  526: 				G.return_cmd[i].Done := FALSE;
  527: 				G.return_cmd[i].Error := TRUE;
  528: 				G.return_cmd[i].ErrorId := return_error_id_pending[i];
  529: 			ELSIF return_cancelled[i] THEN
  530: 				// 主动取消只表示已经安全停稳，不把取消伪装成原计划动作成功完成。
  531: 				G.return_cmd[i].Done := FALSE;
  532: 				G.return_cmd[i].Error := FALSE;
  533: 				G.return_cmd[i].ErrorId := 0;
  534: 			ELSE
  535: 				G.return_cmd[i].Done := TRUE;
  536: 				G.return_cmd[i].Error := FALSE;
  537: 				G.return_cmd[i].ErrorId := 0;
  538: 			END_IF
  539: 			return_finish_error[i] := FALSE;
  540: 			return_cancelled[i] := FALSE;
  541: 			return_error_id_pending[i] := 0;
  542: 			return_state[i] := 0;
  543: 		END_IF
  544: 
  545: 	40:
  546: 		G.return_cmd[i].Busy := TRUE;
  547: 		G.return_cmd[i].Done := FALSE;
  548: 		return_exec[i] := FALSE;
  549: 		spg_enable_exec[i] := FALSE;
  550: 		spg_disable_exec[i] := FALSE;
  551: 		// 状态40仍以实际停稳为退出条件。若原运动或此前的MC_Stop已经报错，
  552: 		// 在继续受控停止期间保持错误对上位机可见，不能用Busy掩盖故障。
  553: 		IF return_finish_error[i] THEN
  554: 			G.return_cmd[i].Error := TRUE;
  555: 			G.return_cmd[i].ErrorId := return_error_id_pending[i];
  556: 		END_IF
  557: 
  558: 		IF host_return_stop_exec[i] THEN
  559: 			IF fb_host_return_stop[i].Done THEN
  560: 				// 停稳后释放 MC_Stop，并复用状态 30/31 按实际位置重建和恢复外部设定值。
  561: 				host_return_stop_exec[i] := FALSE;
  562: 				host_return_stop_active[i] := FALSE;
  563: 				return_state[i] := 30;
  564: 			ELSIF fb_host_return_stop[i].Error THEN
  565: 				// 原运动错误优先；否则锁存首次MC_Stop错误并立即通知上位机。
  566: 				// Busy和停止任务保持有效，Execute拉低一拍后仍继续尝试受控停止，
  567: 				// 只有真实Done才能进入位置重建，绝不把停止失败伪装成完成。
  568: 				IF NOT return_finish_error[i] THEN
  569: 					return_finish_error[i] := TRUE;
  570: 					return_error_id_pending[i] := fb_host_return_stop[i].ErrorID;
  571: 				END_IF
  572: 				G.return_cmd[i].Error := TRUE;
  573: 				G.return_cmd[i].ErrorId := return_error_id_pending[i];
  574: 				host_return_stop_exec[i] := FALSE;
  575: 			END_IF
  576: 		ELSIF NOT fb_host_return_stop[i].Error THEN
  577: 			// 首次进入或错误已由低电平清除后，启动（或重试）受控停止。
  578: 			host_return_stop_exec[i] := TRUE;
  579: 		END_IF
  580: 	END_CASE
  581: END_FOR
  582: 
  583: axis4_request_dir_now := 0;
  584: IF G.axis4_fwd_req AND NOT G.axis4_rev_req THEN
  585: 	axis4_request_dir_now := 1;
  586: ELSIF G.axis4_rev_req AND NOT G.axis4_fwd_req THEN
  587: 	axis4_request_dir_now := -1;
  588: END_IF
  589: 
  590: IF axis4_manual_state = 0 THEN
  591: 	G.axis4_manual_busy := FALSE;
  592: 	IF axis4_request_dir_now = 0 THEN
  593: 		G.axis4_manual_done := FALSE;
  594: 		G.axis4_manual_error := FALSE;
  595: 		G.axis4_manual_error_id := 0;
  596: 	ELSIF NOT G.axis4_manual_error THEN
  597: 		G.axis4_manual_done := FALSE;
  598: 		G.axis4_manual_error := FALSE;
  599: 		G.axis4_manual_error_id := 0;
  600: 		axis4_requested_dir := axis4_request_dir_now;
  601: 		axis4_pending_dir := 0;
  602: 		axis4_finish_error := FALSE;
  603: 		axis4_error_id_pending := 0;
  604: 		axis4_manual_state := 10;
  605: 	END_IF
  606: END_IF
  607: 
  608: CASE axis4_manual_state OF
  609: 10:
  610: 	G.axis4_manual_busy := TRUE;
  611: 	spg_disable_exec[4] := TRUE;
  612: 	IF G.SetPointGenDisable[4].Done OR NOT G.SetPointGenDisable[4].Enabled THEN
  613: 		ext_setpoint_enabled[4] := FALSE;
  614: 		axis4_manual_state := 20;
  615: 	END_IF
  616: 
  617: 20:
  618: 	G.axis4_manual_busy := TRUE;
  619: 	axis4_pending_dir := axis4_request_dir_now;
  620: 	IF axis4_requested_dir >= 0 THEN
  621: 		axis4_manual_direction := MC_Direction.MC_Positive_Direction;
  622: 	ELSE
  623: 		axis4_manual_direction := MC_Direction.MC_Negative_Direction;
  624: 	END_IF
  625: 
  626: 	IF axis4_pending_dir = 0 THEN
  627: 		axis4_manual_state := 30;
  628: 	ELSIF axis4_pending_dir <> axis4_requested_dir THEN
  629: 		axis4_manual_state := 30;
  630: 	ELSE
  631: 		axis4_move_exec := TRUE;
  632: 	END_IF
  633: 
  634: 	IF fb_axis4_move_velocity.Error OR fb_axis4_move_velocity.CommandAborted THEN
  635: 		axis4_finish_error := TRUE;
  636: 		axis4_error_id_pending := fb_axis4_move_velocity.ErrorID;
  637: 		G.axis4_manual_error_id := fb_axis4_move_velocity.ErrorID;
  638: 		axis4_pending_dir := 0;
  639: 		axis4_manual_state := 40;
  640: 	END_IF
  641: 
  642: 30:
  643: 	G.axis4_manual_busy := TRUE;
  644: 	axis4_pending_dir := axis4_request_dir_now;
  645: 	axis4_stop_exec := TRUE;
  646: 
  647: 	IF fb_axis4_stop.Error THEN
  648: 		axis4_finish_error := TRUE;
  649: 		axis4_error_id_pending := fb_axis4_stop.ErrorID;
  650: 		axis4_pending_dir := 0;
  651: 		axis4_manual_state := 39;
  652: 	ELSIF fb_axis4_stop.Done OR (ABS(G.axis[4].NcToPlc.ActVelo) <= 0.05) THEN
  653: 		IF axis4_pending_dir <> 0 THEN
  654: 			axis4_requested_dir := axis4_pending_dir;
  655: 			axis4_manual_state := 20;
  656: 		ELSE
  657: 			axis4_requested_dir := 0;
  658: 			axis4_manual_state := 39;
  659: 		END_IF
  660: 	END_IF
  661: 
  662: 39:
  663: G.axis4_manual_busy := TRUE;
  664: // MC_Stop needs one cycle with Execute=FALSE before SetPointGenEnable can get a clean rising edge
  665: axis4_manual_state := 40;
  666: 	
  667: 40:
  668: 	G.axis4_manual_busy := TRUE;
  669: 	G.Act_pos[4] := G.axis[4].NcToPlc.ActPos - G.init_pos[4];
  670: 	G.act_pos_from_left[4] := G.axis[4].NcToPlc.ActPos - G.leftlimit[4];
  671: 	G.refer[4] := G.Act_pos[4];
  672: 	G.ref_slow[4] := G.Act_pos[4];
  673: 	G.act_h[4] := G.Act_pos[4];
  674: 	G.act_hf[4] := G.Act_pos[4];
  675: 	G.refer_from_left[4] := G.act_pos_from_left[4];
  676: 	FOR k:=1 TO 30 DO
  677: 		act_hh[4][k] := G.Act_pos[4];
  678: 	END_FOR
  679: 	act_h_prev[4] := G.Act_pos[4];
  680: 	traj_pos[4] := G.Act_pos[4];
  681: 	traj_vel[4] := 0.0;
  682: 	traj_acc[4] := 0.0;
  683: 	spg_enable_exec[4] := TRUE;
  684: 	IF G.SetPointGenEnable[4].Done THEN
  685: 		ext_setpoint_enabled[4] := TRUE;
  686: 		G.axis4_manual_busy := FALSE;
  687: 		IF axis4_finish_error THEN
  688: 			G.axis4_manual_done := FALSE;
  689: 			G.axis4_manual_error := TRUE;
  690: 			G.axis4_manual_error_id := axis4_error_id_pending;
  691: 		ELSE
  692: 			G.axis4_manual_done := TRUE;
  693: 			G.axis4_manual_error := FALSE;
  694: 			G.axis4_manual_error_id := 0;
  695: 		END_IF
  696: 		axis4_finish_error := FALSE;
  697: 		axis4_error_id_pending := 0;
  698: 		axis4_manual_state := 0;
  699: 	END_IF
  700: END_CASE
  701: 
  702: FOR i:=1 TO 7 DO
  703: 	IF G.power_output[i].Error THEN
  704: 		G.axis_Reset[i].Execute := TRUE;
  705: 	END_IF
  706: END_FOR
  707: 
  708: FOR i:=1 TO 7 DO
  709: 	// 计划回退结束后，状态 31 的预送周期和 SetPointGenEnable 过渡周期都必须喂入当前设定值。
  710: 	// 否则 NC 重新接管时会短暂使用禁用前遗留的左窗口设定值，造成约一个窗口宽度的反向尖峰。
  711: 	IF ext_setpoint_enabled[i] OR (return_state[i] = 31) OR spg_enable_exec[i] THEN
  712: 		IF traj_enable[i] AND NOT G.startup_smoothing_bypass THEN
  713: 			pos_abs := traj_pos[i] + G.init_pos[i];
  714: 			MC_ExtSetPointGenFeed(
  715: 				Position     := pos_abs,
  716: 				Velocity     := traj_vel[i],
  717: 				Acceleration := traj_acc[i],
  718: 				Direction    := 1,
  719: 				Axis         := G.axis[i]);
  720: 		ELSE
  721: 			pos_abs := G.act_h[i] + G.init_pos[i];
  722: 			legacy_vel := (G.act_h[i] - act_h_prev[i]) / 0.001;
  723: 			MC_ExtSetPointGenFeed(
  724: 				Position     := pos_abs,
  725: 				Velocity     := legacy_vel,
  726: 				Acceleration := 0,
  727: 				Direction    := 1,
  728: 				Axis         := G.axis[i]);
  729: 		END_IF
  730: 		G.act_hf[i] := G.act_h[i];
  731: 	ELSE
  732: 		G.act_hf[i] := G.axis[i].NcToPlc.ActPos - G.init_pos[i];
  733: 	END_IF
  734: 	act_h_prev[i] := G.act_h[i];
  735: END_FOR
  736: 
  737: FOR i:=1 TO 7 DO
  738: 	G.SetPointGenEnable[i](Axis:=G.axis[i], Execute:=spg_enable_exec[i]);
  739: 	G.SetPointGenDisable[i](Axis:=G.axis[i], Execute:=spg_disable_exec[i]);
  740: 	fb_return_move_abs[i](
  741: 		Axis:=G.axis[i],
  742: 		Position:=return_target_abs[i],
  743: 		Velocity:=return_velocity[i],
  744: 		Acceleration:=return_acc[i],
  745: 		Deceleration:=return_dec[i],
  746: 		Jerk:=return_jerk[i],
  747: 		Execute:=return_exec[i]
  748: 	);
  749: 	fb_host_return_stop[i](
  750: 		Axis:=G.axis[i],
  751: 		Execute:=host_return_stop_exec[i],
  752: 		Deceleration:=100.0,
  753: 		Jerk:=500.0
  754: 	);
  755: END_FOR
  756: 
  757: fb_axis4_move_velocity(
  758: 	Axis:=G.axis[4],
  759: 	Execute:=axis4_move_exec,
  760: 	Velocity:=axis4_manual_velocity,
  761: 	Acceleration:=axis4_manual_acc,
  762: 	Deceleration:=axis4_manual_dec,
  763: 	Jerk:=axis4_manual_jerk,
  764: 	Direction:=axis4_manual_direction
  765: );
  766: fb_axis4_stop(
  767: 	Axis:=G.axis[4],
  768: 	Execute:=axis4_stop_exec,
  769: 	Deceleration:=axis4_manual_dec,
  770: 	Jerk:=axis4_manual_jerk
  771: );
  772: 