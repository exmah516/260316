const fs=require('fs');
const f='64位ADS - 相对路径 - 传数组 - 加上手柄/main.cpp';
let s=fs.readFileSync(f,'utf8');
function end(i){let d=0,q='';for(;i<s.length;i++){let c=s[i],n=s[i+1];if(q==='//' ){if(c=='\n')q='';continue}if(q==='/*'){if(c=='*'&&n=='/'){q='';i++}continue}if(q){if(c=='\\')i++;else if(c===q)q='';continue}if(c=='/'&&(n=='/'||n=='*')){q=c+n;i++;continue}if(c=='"'||c=="'"){q=c;continue}if(c=='{')d++;if(c=='}'&&!--d)return i}throw Error('unbalanced')}
function rm(a){let i=s.indexOf(a);if(i<0)return;let o=s.indexOf('{',i),e=end(o);s=s.slice(0,i)+s.slice(e+1)}
function line(re){s=s.split(/(?<=\n)/).filter(x=>!re.test(x)).join('')}
rm('auto cancel_cooperative_delivery =');
rm('auto cooperative_direction_text =');
rm('auto reset_cooperative_direction_guards =');
rm('auto validate_cooperative_entry =');
rm('if (guidewire_mode == GuidewireMode::Cooperative)');
rm('if (cooperative_direction_requested != CooperativeDirection::None && dual_handle_ready)');
rm('if (cooperative_request)');
rm('else if (requested_guidewire_mode == GuidewireMode::Cooperative)');
rm('if (cooperative_request && !mode_ok)');
rm('if (!cooperative_transition_failed &&');
rm('if (cooperative_mode)');
line(/ft_exp|ft_v_limit|cooperative_direction|dual_handle_ready|axis6_coop|CooperativeDirection|GuidewireMode::Cooperative|ModeSelection::Cooperative|cooperative_request|cooperative_transition_failed|cooperative_mode_active|cooperative_retraction_active|cooperative_axis1_locked|cooperative_axis6_mode|cooperative_relative_window_control|cooperative_trigger_from_far_edge|cooperative_axis5_increment|cooperative_follow_active|current_cooperative_return_owner/);
s=s.replace(/planned_return\.cooperative\(\)/g,'false');
s=s.replace(/planned_return\.mode == PlannedReturnMode::Cooperative\w+/g,'false');
s=s.replace(/const bool axis1_return_couples_axis6 = !cooperative_mode;/g,'const bool axis1_return_couples_axis6 = true;');
s=s.replace(/const bool cooperative_mode = .*;\r?\n/g,'');
s=s.replace(/requested_guidewire_mode_prev = \(cooperative_transition_failed \|\| physical_mode_transition_rejected\)/g,'requested_guidewire_mode_prev = physical_mode_transition_rejected');
s=s.replace(/if \(planned_return\.active\(\) \|\| ft_exp\.active\(\)\)/g,'if (planned_return.active())');
fs.writeFileSync(f,s,'utf8');
