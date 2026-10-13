const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');

const header = fs.readFileSync(path.resolve(__dirname, '../../src/Port/DebugWebPage.h'), 'utf8');
const raw = header.match(/R"APPLEPY\(([\s\S]*)\)APPLEPY";/);
if (!raw) throw new Error('DebugWebPage.h HTML not found');
const token = 'LOCAL-MOCK-TOKEN';
const banner = '<div style="background:#7b2f2f;color:white;padding:12px;border-radius:9px;margin-bottom:12px;font-weight:700">本地模拟，不是真板：不会访问串口、PWM 或 NVS。</div>';
const page = raw[1].replace('__TOKEN__', token).replace('<main>', '<main>' + banner);

let saved = [17.9, 12.8, 0.5, 114, 0, 2];
let mockGeneration = 0;
let driveLeaseTimer = null;
let driveLeaseGeneration = 0;
const motion = {
  mode:'idle', error:'none', pending:false, transition:false, phase:0,
  parameter_state:'loaded_nvs', parameters_saved:true, parameter_revision:0,
  diagnostics:{frame_count:0,last_gap_ms:20,max_gap_ms:20,missed_deadlines:0},
  parameters:[...saved], feet:[[0,114],[0,114],[0,114],[0,114]],
  command_deg:[90,90,90,90,90,90,90,90],
  body:{pitch_deg:0,roll_deg:0,x_mm:8,cg_enabled:false,cg_shift_mm:2.5,revision:0,
    length_mm:142,width_mm:92.8,dimensions_measured:false,cg_actual_x_min_mm:0,cg_actual_x_max_mm:0},
  balance:{capture_pending:false,capture_samples:0,capture_required:20,reference_ready:false,
    enabled:false,enable_pending:false,reference_pitch_deg:0,reference_roll_deg:0,
    pitch_correction_deg:0,roll_correction_deg:0,roll_sensor_sign:-1,error:'disabled_on_boot'},
  preview:{error:'not_run',leg:-1,joint:-1,phase:0,min_support_margin_mm:0,
    pending:false,stage:'idle',result:'not_run',frames_checked:0,max_slice_ms:0}
};
motion.drive={forward:1,turn:0,stepping:false,active:false,epoch:1,seq:0,
  straight_trim:0.25,effective_turn:0,
  trot_pitch_offset_deg:0,trot_effective_pitch_deg:0,trot_pitch_revision:0,
  lease_remaining_ms:0,reason:'boot',moving_cg_enabled:false,moving_cg_pending:false,
  moving_cg_shift_mm:0,moving_cg_error:'disabled',phase_waiting:false,cg_stage:0,
  nominal_elapsed_s:0,actual_elapsed_s:0};
motion.profile={state:'nvs_missing_using_default',saved:false,revision:0,planner_slew_mm_s:20,
  wait_timeout_ms:15000,moving_angle_gain:1.1,moving_rate_gain:5,moving_tau_s:0.1,
  moving_max_shift_mm:0.5,moving_slew_mm_s:2,pitch_kp:0.8,pitch_kd_s:0.08,
  roll_kp:0.8,roll_kd_s:0.08};
Object.assign(motion.profile,{trot_ff_enabled:false,trot_ff_forward:4,trot_ff_backward:4,
  trot_ff_turn:2,trot_ff_max_mm:2,trot_ff_current_mm:0});
let savedProfile={...motion.profile};
let savedBody={...motion.body};
const state = {
  firmware_version:'0.8.14-mock', pca:true, armed:false, calibrating:false,
  calibration_pending:false, calibration_mask:0, fault:false, imu_ok:true,
  calibration_samples:100, calibration_required:100, gyro_calibrated:true,
  accel_norm_mps2:9.8, active_channel:-1, active_pulse_us:0, remaining_ms:0,
  manual_hold:true, extended_range:false, pulse_min_us:500, pulse_max_us:2500,
  motion, imu:{accel:[0,0,9.8],gyro:[0,0,0]}, attitude_ok:true,
  attitude:{roll_deg:0,pitch_deg:0,yaw_deg:0,yaw_reference_ms:1000},
  channels:[0,1,2,3,12,9,10,11].map((channel,index)=>({
    servo:channel+1,channel,center_us:1500,center_deg:90,
    joint:['L1H','L1S','L2H','L2S','L3H','L3S','L4H','L4S'][index],
    relative_available:true,shank:index%2===1,min_offset_deg:index%2===1?(index===1||index===7?-50:0):-90,
    max_offset_deg:index%2===1?(index===1||index===7?0:50):90,
    shank_q_available:index%2===1,shank_q_photo_candidate:index%2===1,
    shank_direction:index===1||index===7?-1:index%2===1?1:0,max_shank_q_deg:index%2===1?23:0,
    shank_map_points:index%2===1?[{u_deg:0,q_deg:0},{u_deg:30,q_deg:9},{u_deg:50,q_deg:23}]:[]
  }))
};

function reply(res,status,body,type='application/json; charset=utf-8') {
  res.writeHead(status,{'Content-Type':type,'Cache-Control':'no-store'});
  res.end(type.startsWith('application/json')?JSON.stringify(body):body);
}
function cancelDrive(reason,clearMoving=true){
  ++driveLeaseGeneration;if(driveLeaseTimer)clearTimeout(driveLeaseTimer);driveLeaseTimer=null;
  motion.drive.active=false;motion.drive.seq=0;motion.drive.epoch++;motion.drive.reason=reason;
  motion.drive.forward=1;motion.drive.turn=0;motion.drive.effective_turn=0;motion.drive.stepping=false;
  if(clearMoving){motion.drive.moving_cg_enabled=false;motion.drive.moving_cg_pending=false;
    motion.drive.moving_cg_shift_mm=0;motion.drive.moving_cg_error=reason;}
}
function scheduleDriveLease(){
  if(driveLeaseTimer)clearTimeout(driveLeaseTimer);
  const generation=++driveLeaseGeneration,epoch=motion.drive.epoch;
  driveLeaseTimer=setTimeout(()=>{if(generation!==driveLeaseGeneration||epoch!==motion.drive.epoch)return;
    driveLeaseTimer=null;motion.drive.active=false;motion.drive.seq=0;motion.drive.epoch++;
    motion.drive.reason='lease_expired_hold';motion.drive.moving_cg_enabled=false;motion.mode='hold';},500);
}
function sameParameters(expected){
  return motion.parameters.length===expected.length&&
    motion.parameters.every((value,index)=>Math.abs(value-expected[index])<0.0001);
}
function mockPreflightFixture(mode){
  const pass=(margin=0,actual=0)=>({supported:true,ok:true,error:'none',leg:-1,joint:-1,
    phase:0,min_support_margin_mm:margin,actual_cycle_s:actual});
  if(mode==='walk'){
    if(motion.body.cg_enabled&&sameParameters([6,2,3.2,108,0,2]))return pass(1.094,3.82);
    if(!motion.body.cg_enabled&&sameParameters([25.3,7.1,0.5,108,0,2]))return pass();
    if(!motion.body.cg_enabled&&sameParameters([6,2,1.2,108,0,2]))return pass();
    if(!motion.body.cg_enabled&&sameParameters([6,2,1.0,108,0,2]))return pass();
    if(!motion.body.cg_enabled&&sameParameters([3,1,3,108,0,1]))return pass();
  }
  if(mode==='trot'){
    if(sameParameters([17.9,12.8,0.5,114,0,2]))return pass();
    if(sameParameters([6,2,1.2,108,0,2]))return pass();
    if(sameParameters([3,1,3,108,0,1]))return pass();
    if(sameParameters([20,6,0.7,115,0,2]))return pass();
  }
  return {supported:false,ok:false,error:'mock_unsupported',leg:-1,joint:-1,phase:0,
    min_support_margin_mm:0,actual_cycle_s:0};
}
function publishMockPreview(fixture){
  motion.preview={error:fixture.error,leg:fixture.leg,joint:fixture.joint,phase:fixture.phase,
    min_support_margin_mm:fixture.min_support_margin_mm,
    pending:false,stage:'complete',result:fixture.ok?'passed':'rejected',
    frames_checked:Math.ceil(motion.parameters[2]*200),max_slice_ms:1,
    actual_cycle_s:fixture.actual_cycle_s};
}
function action(form) {
  const cmd=form.get('cmd');
  if(cmd==='motion_set') {
    const values=['step','lift','period','height','forward','shift'].map(name=>Number(form.get(name)));
    if(values.some(value=>!Number.isFinite(value)) || values[0]<0 || values[0]>30 ||
       values[1]<0 || values[1]>30 || values[2]<0.4 || values[2]>10 ||
       values[3]<100 || values[3]>115 || Math.abs(values[4])>5 || values[5]<0 || values[5]>3)
      return [409,{ok:false,message:'模拟：参数范围/Stand检查拒绝，RAM未变'}];
    if(motion.pending||motion.transition||['walk','trot'].includes(motion.mode))
      return [409,{ok:false,message:'模拟：运动/过渡中拒绝，请先Stop'}];
    motion.parameters=values; motion.parameters_saved=values.every((v,i)=>v===saved[i]);
    motion.parameter_state=motion.parameters_saved?'applied_matches_nvs':'applied_ram';
    motion.parameter_revision++;
    motion.balance.reference_ready=false;motion.balance.enabled=false;
    return [200,{ok:true,message:'模拟：参数已应用RAM，零PWM'}];
  }
  if(cmd==='motion_preview') {
    const mode=form.get('mode');
    const fixture=mockPreflightFixture(mode);publishMockPreview(fixture);
    if(!fixture.supported)return[409,{ok:false,message:'模拟器未覆盖该参数组合；请以真实固件预检为准'}];
    if(!fixture.ok)return[409,{ok:false,message:'模拟fixture：'+mode+'预检未通过'}];
    return [200,{ok:true,message:'模拟fixture：'+mode+'整周期预检通过，零PWM'}];
  }
  if(cmd==='arm_stand') {
    if(state.armed) return [409,{ok:false,message:'模拟：已使能，重复请求拒绝'}];
    const generation=++mockGeneration;
    state.armed=true; motion.pending=true; motion.mode='idle';
    setTimeout(()=>{if(generation!==mockGeneration)return;motion.pending=false;motion.mode='stand';motion.diagnostics.frame_count+=76;},300);
    return [200,{ok:true,message:'模拟：已使能并排队Stand'}];
  }
  if(cmd==='motion_stop') { ++mockGeneration;cancelDrive('motion_stop');motion.pending=false;motion.transition=false;motion.balance.enabled=false;motion.balance.enable_pending=false;motion.balance.capture_pending=false;motion.balance.error='stopped_by_user';motion.mode=state.armed?'hold':'idle'; return [200,{ok:true,message:'模拟：停止并保持'}]; }
  if(cmd==='off') { ++mockGeneration;cancelDrive('global_off');state.armed=false;state.calibrating=false;
    state.calibration_pending=false;state.calibration_mask=0;state.active_channel=-1;state.active_pulse_us=0;
    motion.pending=false;motion.transition=false;motion.mode='idle';motion.balance.enabled=false;
    motion.balance.enable_pending=false;motion.balance.capture_pending=false;motion.balance.reference_ready=false;
    motion.balance.error='cleared_by_global_off';return [200,{ok:true,message:'模拟：全部输出已关闭'}]; }
  if(cmd==='cal_start') {
    if(form.get('confirm')!=='LOOSE_LINKAGES')return[400,{ok:false,message:'模拟：缺少标定安全确认'}];
    if(state.armed||state.fault||!state.pca)return[409,{ok:false,message:'模拟：开始标定前必须关闭输出且PCA正常'}];
    state.armed=true;state.calibrating=true;state.calibration_pending=true;state.calibration_mask=0xff;
    state.active_channel=-1;state.active_pulse_us=0;motion.mode='idle';
    return[200,{ok:true,message:'模拟：八路已回当前内存中位（不访问硬件）'}];
  }
  if(cmd==='nudge') {
    const channel=Number(form.get('channel')),delta=Number(form.get('delta'));
    const entry=state.channels.find(item=>item.channel===channel);
    if(!state.calibrating||!state.calibration_pending)return[409,{ok:false,message:'模拟：请先开始八路标定'}];
    if(!entry||![-1,1].includes(delta))return[400,{ok:false,message:'模拟：接口或微调值无效'}];
    const next=entry.center_deg+delta;
    if(next<0||next>180)return[409,{ok:false,message:'模拟：中位超出0–180°'}];
    entry.center_deg=next;entry.center_us=Math.round(500+next*2000/180);
    return[200,{ok:true,message:'模拟：'+(entry.joint||('ch'+channel))+' 中位 '+(delta>0?'+1°':'-1°')+'（仅内存）'}];
  }
  if(cmd==='cal_save_all') {
    if(!state.calibration_pending||!state.calibrating)return[409,{ok:false,message:'模拟：没有待保存的八路标定'}];
    state.armed=false;state.calibrating=false;state.calibration_pending=false;state.calibration_mask=0;
    state.active_channel=-1;state.active_pulse_us=0;
    return[200,{ok:true,message:'模拟：已全关并在mock内存中保存八路中位（不写NVS）'}];
  }
  if(cmd==='motion_save') {
    if(state.armed) return [409,{ok:false,message:'模拟：保存要求全关'}];
    saved=[...motion.parameters];motion.parameters_saved=true;motion.parameter_state='saved_nvs';
    return [200,{ok:true,message:'模拟：已保存并回读NVS记录'}];
  }
  if(cmd==='motion_load') {
    if(state.armed) return [409,{ok:false,message:'模拟：恢复要求全关'}];
    motion.parameters=[...saved];motion.parameters_saved=true;motion.parameter_state='loaded_nvs';motion.parameter_revision++;
    return [200,{ok:true,message:'模拟：已从NVS恢复到RAM'}];
  }
  if(cmd==='control_set') {
    const names=['planner_slew','wait_s','angle_gain','rate_gain','tau','max_shift','feedback_slew',
      'pitch_kp','pitch_kd','roll_kp','roll_kd'];
    const v=names.map(name=>Number(form.get(name)));if(v.some(x=>!Number.isFinite(x)))return[409,{ok:false,message:'模拟：控制参数无效'}];
    Object.assign(motion.profile,{planner_slew_mm_s:v[0],wait_timeout_ms:v[1]*1000,
      moving_angle_gain:v[2],moving_rate_gain:v[3],moving_tau_s:v[4],moving_max_shift_mm:v[5],
      moving_slew_mm_s:v[6],pitch_kp:v[7],pitch_kd_s:v[8],roll_kp:v[9],roll_kd_s:v[10],state:'applied_ram',saved:false,
      revision:motion.profile.revision+1});return[200,{ok:true,message:'模拟：控制Profile已应用RAM'}];
  }
  if(cmd==='trot_ff_set'){
    Object.assign(motion.profile,{trot_ff_enabled:form.get('enable')==='1',
      trot_ff_forward:Number(form.get('forward_gain')),trot_ff_backward:Number(form.get('backward_gain')),
      trot_ff_turn:Number(form.get('turn_gain')),trot_ff_max_mm:Number(form.get('max_abs')),
      state:'applied_ram',saved:false,revision:motion.profile.revision+1});
    return[200,{ok:true,message:'模拟：Trot方向前馈已应用RAM'}];
  }
  if(cmd==='straight_trim_set'){
    const value=Number(form.get('value'));
    if(!Number.isFinite(value)||value < -0.30||value > 0.30)
      return[400,{ok:false,message:'模拟：直行修正范围为-0.30..+0.30'}];
    if(motion.pending||motion.transition||['walk','trot'].includes(motion.mode))
      return[409,{ok:false,message:'模拟：运动中拒绝直行修正'}];
    motion.drive.straight_trim=value;
    return[200,{ok:true,message:'模拟：直行修正已应用RAM，零PWM，重启恢复默认0.25'}];
  }
  if(cmd==='trot_pitch_set'){
    const value=Number(form.get('value'));
    if(!Number.isFinite(value)||value < -0.5||value > 0||Math.abs(value*10-Math.round(value*10))>0.001)
      return[400,{ok:false,message:'模拟：Trot Pitch范围为-0.5..0.0°'}];
    if(motion.pending||motion.transition||['walk','trot'].includes(motion.mode))
      return[409,{ok:false,message:'模拟：运动中拒绝Trot Pitch补偿'}];
    if(value<0&& !sameParameters([17.9,12.8,0.5,114,0,2]))
      return[409,{ok:false,message:'模拟：当前Trot参数预检拒绝，RAM未变'}];
    motion.drive.trot_pitch_offset_deg=value;
    motion.drive.trot_effective_pitch_deg=motion.body.pitch_deg+value;
    motion.drive.trot_pitch_revision++;
    return[200,{ok:true,message:'模拟：Trot Pitch补偿已应用RAM，零PWM，重启归零'}];
  }
  if(cmd==='profile_save'){if(state.armed)return[409,{ok:false,message:'模拟：Profile保存要求全关'}];
    savedProfile={...motion.profile};savedBody={...motion.body};motion.profile.saved=true;motion.profile.state='saved_nvs';
    return[200,{ok:true,message:'模拟：Profile已保存并回读'}];}
  if(cmd==='profile_load'){if(state.armed)return[409,{ok:false,message:'模拟：Profile恢复要求全关'}];
    Object.assign(motion.profile,savedProfile,{saved:true,state:'loaded_nvs',revision:motion.profile.revision+1});
    Object.assign(motion.body,savedBody,{revision:motion.body.revision+1});
    return[200,{ok:true,message:'模拟：Profile已恢复RAM，不自动运动'}];}
  if(cmd==='body_set') {
    const values=['pitch','roll','x','cg','cg_shift'].map(name=>Number(form.get(name)));
    if(values.some(v=>!Number.isFinite(v))||values[0]<-7||values[0]>5||Math.abs(values[1])>10||
       Math.abs(values[2])>8||![0,1].includes(values[3])||values[4]<0||values[4]>3)
      return [409,{ok:false,message:'模拟：机身/CG范围拒绝'}];
    Object.assign(motion.body,{pitch_deg:values[0],roll_deg:values[1],x_mm:values[2],
      cg_enabled:values[3]===1,cg_shift_mm:values[4],revision:motion.body.revision+1});
    motion.drive.trot_effective_pitch_deg=values[0]+motion.drive.trot_pitch_offset_deg;
    const front=motion.parameters[4]-values[4],rear=motion.parameters[4]+motion.parameters[0]+values[4];
    motion.body.cg_actual_x_min_mm=values[3]?Math.min(values[2],front,rear):values[2];
    motion.body.cg_actual_x_max_mm=values[3]?Math.max(values[2],front,rear):values[2];
    motion.balance.reference_ready=false;motion.balance.enabled=false;
    motion.profile.saved=false;motion.profile.state='applied_ram';
    return [200,{ok:true,message:'模拟：机身/CG已应用RAM，零PWM'}];
  }
  if(cmd==='pose_live_set') {
    const values=['height','pitch','roll','x'].map(name=>Number(form.get(name)));
    if(values.some(v=>!Number.isFinite(v))||values[0]<100||values[0]>115||
       values[1]<-7||values[1]>5||Math.abs(values[2])>10||Math.abs(values[3])>8)
      return[409,{ok:false,message:'模拟：实时姿态硬范围拒绝，原值保持'}];
    if(!((!state.armed&&motion.mode==='idle')||
         (state.armed&&['stand','hold'].includes(motion.mode))))
      return[409,{ok:false,message:'模拟：实时姿态仅允许OFF或Stand/Hold'}];
    motion.parameters[3]=values[0];motion.parameter_revision++;
    motion.body.pitch_deg=values[1];motion.body.roll_deg=values[2];motion.body.x_mm=values[3];motion.body.revision++;
    motion.profile.saved=false;motion.profile.state='applied_ram';
    if(state.armed){motion.mode='stand';motion.pending=true;setTimeout(()=>{motion.pending=false;},150);}
    return[200,{ok:true,message:state.armed?'模拟：实时姿态已平滑重定位':'模拟：输出关闭，仅更新RAM'}];
  }
  if(cmd==='balance_roll_sign') {
    if(state.armed&&motion.mode!=='hold')return [409,{ok:false,message:'模拟：Roll符号需OFF或Hold'}];
    motion.balance.roll_sensor_sign=Number(form.get('sign'));motion.balance.reference_ready=false;
    return [200,{ok:true,message:'模拟：Roll符号已应用，需重抓参考'}];
  }
  if(cmd==='balance_capture') {
    if(motion.mode!=='stand')return [409,{ok:false,message:'模拟：仅稳定Stand可抓取'}];
    const generation=++mockGeneration;
    motion.balance.capture_pending=true;motion.balance.capture_samples=0;
    const timer=setInterval(()=>{if(generation!==mockGeneration){clearInterval(timer);return;}
      motion.balance.capture_samples+=5;if(motion.balance.capture_samples>=20){
      clearInterval(timer);motion.balance.capture_samples=20;motion.balance.capture_pending=false;
      motion.balance.reference_ready=true;motion.balance.error='none';}},60);
    return [200,{ok:true,message:'模拟：开始抓取20个稳定样本'}];
  }
  if(cmd==='balance_on') {
    if(motion.mode!=='stand'||!motion.balance.reference_ready)return [409,{ok:false,message:'模拟：需稳定Stand和参考'}];
    const generation=++mockGeneration;
    motion.balance.enable_pending=true;setTimeout(()=>{if(generation!==mockGeneration)return;motion.balance.enable_pending=false;motion.balance.enabled=true;motion.balance.error='none';},100);
    return [200,{ok:true,message:'模拟：静态自稳待新样本开启'}];
  }
  if(cmd==='balance_off') {++mockGeneration;motion.pending=false;motion.transition=false;motion.balance.enabled=false;
    motion.balance.enable_pending=false;motion.balance.capture_pending=false;motion.balance.error='disabled_by_user';
    cancelDrive('balance_off');
    motion.mode=state.armed?'hold':'idle';return [200,{ok:true,message:'模拟：关闭自稳并保持'}];}
  if(cmd==='moving_cg_on'){if(motion.mode!=='stand'||!motion.balance.reference_ready)return[409,{ok:false,message:'模拟：需Stand参考'}];
    const generation=++mockGeneration;motion.drive.moving_cg_pending=true;setTimeout(()=>{if(generation!==mockGeneration)return;
      motion.drive.moving_cg_pending=false;motion.drive.moving_cg_enabled=true;
      motion.drive.moving_cg_error='none';},80);return[200,{ok:true,message:'模拟：移动CG待新样本开启'}];}
  if(cmd==='moving_cg_off'){++mockGeneration;cancelDrive('moving_cg_off');
    motion.balance.enabled=false;motion.balance.enable_pending=false;
    motion.mode=state.armed?'hold':'idle';return[200,{ok:true,message:'模拟：停止移动CG/方向并保持'}];}
  if(cmd==='drive_begin'){
    const expected=Number(form.get('epoch')),seq=Number(form.get('seq'));
    if(!state.armed||motion.drive.active||expected!==motion.drive.epoch)return[409,{ok:false,message:'模拟：drive begin拒绝'}];
    const fixture=mockPreflightFixture(form.get('mode'));publishMockPreview(fixture);
    if(!fixture.supported)return[409,{ok:false,message:'模拟器未覆盖该方向参数组合'}];
    if(!fixture.ok)return[409,{ok:false,message:'模拟fixture预检拒绝'}];
    motion.drive.active=true;motion.drive.epoch++;motion.drive.seq=seq;motion.drive.forward=Number(form.get('forward'));
    const rawTurn=Number(form.get('turn')),forward=Number(form.get('forward'));
    motion.drive.stepping=form.get('step')==='1';
    motion.drive.turn=motion.drive.stepping||Math.abs(rawTurn)>0.0001||Math.abs(forward)<0.0001?
      rawTurn:(forward>0?-motion.drive.straight_trim:motion.drive.straight_trim);
    motion.drive.effective_turn=motion.drive.turn;motion.drive.reason='active';
    motion.mode=form.get('mode');scheduleDriveLease();
    return[200,{ok:true,message:'模拟：方向会话开始'}];
  }
  if(cmd==='drive_renew'){
    const epoch=Number(form.get('epoch')),seq=Number(form.get('seq'));
    if(!motion.drive.active||epoch!==motion.drive.epoch||seq<=motion.drive.seq)return[409,{ok:false,message:'模拟：续租拒绝'}];
    motion.drive.seq=seq;scheduleDriveLease();
    return[200,{ok:true,message:'模拟：仅续租，不重启'}];
  }
  if(cmd==='drive_end'){
    if(Number(form.get('epoch'))!==motion.drive.epoch)return[409,{ok:false,message:'模拟：释放epoch错误'}];
    cancelDrive('released_hold');motion.mode='hold';return[200,{ok:true,message:'模拟：释放后保持最后帧'}];
  }
  if(['stand','walk','trot'].includes(cmd)) {
    if(!state.armed) return [409,{ok:false,message:'模拟：请先使能并站立'}];
    if(cmd!=='stand'){
      const fixture=mockPreflightFixture(cmd);publishMockPreview(fixture);
      if(!fixture.supported)return[409,{ok:false,message:'模拟器未覆盖该参数组合；未启动'}];
      if(!fixture.ok)return[409,{ok:false,message:'模拟fixture预检拒绝'}];
    }
    ++mockGeneration;cancelDrive('manual_'+cmd,cmd==='stand');motion.balance.enabled=false;motion.balance.enable_pending=false;
    motion.balance.capture_pending=false;motion.balance.error='paused_for_motion_transition';
    if(cmd!=='stand'){motion.drive.forward=1;motion.drive.turn=-motion.drive.straight_trim;
      motion.drive.effective_turn=motion.drive.turn;motion.drive.stepping=false;}
    motion.mode=cmd;motion.pending=false;return [200,{ok:true,message:'模拟：'+cmd+'已启动'}];
  }
  if(cmd==='arm') { state.armed=true; return [200,{ok:true,message:'模拟：单路解锁，不运动'}]; }
  return [200,{ok:true,message:'模拟：操作已接受（不访问硬件）'}];
}

const port=Number(process.env.APPLEPY_MOCK_PORT||8765);
function createMockServer(){return http.createServer((req,res)=>{
  if(req.method==='GET' && req.url==='/') return reply(res,200,page,'text/html; charset=utf-8');
  if(req.method==='GET' && req.url==='/api/state') return reply(res,200,state);
  if(req.method==='POST' && req.url==='/api/action') {
    if(req.headers['x-applepy-token']!==token) return reply(res,403,{ok:false,message:'mock token rejected'});
    let body=''; req.setEncoding('utf8'); req.on('data',chunk=>body+=chunk);
    req.on('end',()=>{const [status,result]=action(new URLSearchParams(body));reply(res,status,result);});
    return;
  }
  reply(res,404,{ok:false,message:'not found'});
});}
if(require.main===module)createMockServer().listen(port,'127.0.0.1',()=>{
  process.stdout.write(`Applepy local mock UI: http://127.0.0.1:${port}/ (no hardware)\n`);
});
module.exports={action,state,motion,mockPreflightFixture,createMockServer};
