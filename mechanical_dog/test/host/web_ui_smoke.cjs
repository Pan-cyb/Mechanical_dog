const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const header = fs.readFileSync(
  path.resolve(__dirname, '../../src/Port/DebugWebPage.h'), 'utf8');
const server = fs.readFileSync(
  path.resolve(__dirname, '../../src/Port/DebugWebServer.cpp'), 'utf8');
const mockServer = fs.readFileSync(
  path.resolve(__dirname, 'mock_web_ui_server.cjs'), 'utf8');
const script = header.split('<script>')[1]?.split('</script>')[0];
assert.ok(script, 'embedded browser script missing');
new vm.Script(script, { filename: 'DebugWebPage.inline.js' });

assert.match(header, /<h1>机械狗控制<\/h1>/);
assert.match(header, /id="viewControl"[^>]*aria-selected="true"[^>]*>控制<\/button>/);
assert.match(header, /id="viewCalibration"[^>]*aria-selected="false"[^>]*>标定<\/button>/);
assert.match(header, /id="calibrationView"[^>]*hidden/);
assert.match(header, /id="mStand"[^>]*>站立<\/button>[\s\S]*id="mStop"[^>]*>停止<\/button>[\s\S]*id="mOff"[^>]*>关闭输出<\/button>/);
assert.match(header, /停止仍保持力矩；关页面不会自动停止；关闭输出不是物理断电/);
assert.match(header, /id="mApply"[^>]*>应用其它步态参数（RAM）<\/button>/);
assert.match(header, /id="mSave"[^>]*>保存已应用值到NVS<\/button>/);
assert.match(header, /高度滑块实时更新；步长\/抬脚\/周期\/高级步态项仍需点“应用其它步态参数”/);
assert.match(header, /id="dMode">[\s\S]*value="walk"[\s\S]*value="trot"/);
assert.equal((header.match(/id="dMode"/g)||[]).length,1,'only one operational gait selector');
assert.match(header, /id="mStart"[^>]*>开始行走<\/button>/);
assert.doesNotMatch(header, /id="mWalk"|id="mTrot"|id="mArmStand"/);
for(const id of ['mStep','mLift','mPeriod','mHeight']) assert.match(header,new RegExp(`id="${id}"`));
assert.match(header, /id="mHeightRange" type="range" min="100" max="115" step="0\.1" value="114"/);
assert.match(header, /站高目标 \/ down mm/);
assert.match(header, /id="bPitchRange" type="range" min="-7" max="5" step="0\.1" value="0"/);
assert.match(header, /id="bRollRange" type="range" min="-10" max="10" step="0\.1" value="0"/);
assert.match(header, /id="bXRange" type="range" min="-8" max="8" step="0\.1" value="8"/);
assert.match(header, /前后重心 \/ body_x mm（\+向前）/);
assert.match(header, /Pitch °（正值抬头）/);
assert.match(header, /Roll °（正值左侧高）/);
assert.match(header, /拖动高度、Pitch、Roll、body_x会合并发送原子实时姿态命令/);
assert.match(header, /方向按钮点击一次持续运动/);
assert.match(server, /op == "pose_live_set"/);
assert.match(server, /const char\* names\[4\]=\{"height","pitch","roll","x"\}/);
assert.match(script, /cmd:'pose_live_set'/);
assert.match(script, /setTimeout\(flushLivePose,150\)/);
assert.doesNotMatch(script, /addEventListener\('pointerdown'/);
assert.match(header, /id="mStep"[^>]*max="30"[^>]*value="17\.9"/);
assert.match(header, /id="mLift"[^>]*max="30"[^>]*value="12\.8"/);
assert.match(header, /id="mPeriod"[^>]*value="0\.5"/);
assert.match(header, /data-mode="walk" data-values="25\.3,7\.1,0\.5,108,0,2">Walk参数/);
assert.match(header, /data-mode="trot" data-values="17\.9,12\.8,0\.5,114,0,2">Trot参数/);
assert.match(mockServer, /let saved = \[17\.9, 12\.8, 0\.5, 114, 0, 2\]/);
assert.match(header, /id="dStraightTrim"[^>]*min="-0\.30"[^>]*max="0\.30"[^>]*step="0\.01"[^>]*value="0\.25"/);
assert.match(header, /内置默认0\.25；\+纠正左偏（减右侧），−纠正右偏（减左侧）；手动值仅RAM，重启恢复0\.25/);
assert.match(server, /op == "straight_trim_set"/);
assert.match(header, /id="dTrotPitch"[^>]*min="-0\.5"[^>]*max="0"[^>]*step="0\.1"/);
assert.match(header, /负值压低前端，正值不允许；仅Trot生效，建议从−0\.5°开始。仅RAM，重启归零/);
assert.match(server, /op == "trot_pitch_set"/);
assert.match(script, /submittedTrotPitchRevision===trotPitchDraftRevision/);
assert.match(mockServer, /values\[0\]>30[\s\S]*values\[1\]>30/);
assert.match(mockServer, /function mockPreflightFixture\(mode\)/);
assert.doesNotMatch(mockServer, /2\.33|62\.7/);
assert.match(header, />慢速<\/button>[\s\S]*>Walk参数<\/button>[\s\S]*>Trot参数<\/button>/);
assert.match(header, /id="posturePanel"><summary>姿态与水平校准<\/summary>/);
assert.match(header, /id="bX"[^>]*value="8"/);
assert.match(header, /id="controlAdvanced"><summary>高级设置与遥测<\/summary>/);
assert.match(header, /id="bCgApply"[^>]*>应用重心设置<\/button>/);
assert.match(header, /id="calAdvanced"><summary>高级调试<\/summary>/);
assert.doesNotMatch(header, /<details[^>]*\sopen(?:\s|>)/);
assert.match(header, /开始会让八路回到当前已存\/内存中位，不会重置为固定90°/);
assert.match(header, /id="calStart"[^>]*>开始标定<\/button>/);
assert.match(header, /id="calSaveAll"[^>]*>保存并退出<\/button>/);
assert.match(header, /id="calConfirm" type="checkbox"/);
assert.match(header, /@media\(max-width:700px\)[\s\S]*\.control-grid,\.grid\{grid-template-columns:1fr\}[\s\S]*\.fields4\{grid-template-columns:repeat\(2,minmax\(0,1fr\)\)\}/);
assert.match(header, /id="motionFeet"[^>]*aria-label="四腿足端命令"/);
assert.match(header, /id="motionAngles"[^>]*aria-label="八路舵机角度命令"/);
assert.match(header, /足端和角度均为命令值，不是反馈/);
assert.doesNotMatch(header, /V7\.3|\/api\/gait|motion_heartbeat/);
assert.match(server, /op == "motion_set"/);
assert.match(server, /op == "motion_preview"/);
assert.match(server, /op == "body_set"/);
assert.match(server, /op == "balance_capture"/);
assert.match(server, /op == "balance_on"/);
assert.match(server, /op == "balance_off"/);
assert.match(server, /op == "balance_roll_sign"/);
assert.match(server, /op == "drive_begin"/);
assert.match(server, /op == "drive_renew"/);
assert.match(server, /op == "drive_end"/);
assert.match(server, /op == "moving_cg_on"/);
assert.match(server, /op == "profile_save"/);
assert.match(server, /op == "profile_load"/);
assert.match(server, /op == "control_set"/);
assert.match(server, /op == "trot_ff_set"/);
assert.match(server, /min_support_margin_mm/);
assert.match(server, /s\.preview_pending/);
assert.match(server, /s\.preview_stage/);
assert.match(server, /s\.preview_result/);
assert.match(server, /s\.preview_frames_checked/);
assert.match(server, /s\.preview_max_slice_ms/);
assert.match(server, /op == "arm_stand"/);
assert.match(server, /op == "motion_save"/);
assert.match(server, /op == "motion_load"/);
assert.match(server, /mode=="stand"[\s\S]*mode=="walk"[\s\S]*mode=="trot"[\s\S]*mode=="leg1"[\s\S]*mode=="leg4"/);
assert.match(server, /op == "leg_test"/);
assert.match(server, /\\\"motion\\\"/);
assert.match(server, /server_\.on\("\/api\/state", HTTP_GET/);
assert.match(server, /server_\.on\("\/api\/action", HTTP_POST/);
assert.match(server, /server_\.on\("\/", HTTP_GET, \[this\] \{ page\(\); \}\)/);
const pageHandler = server.split('void DebugWebServer::page()')[1]
  ?.split('void DebugWebServer::state()')[0];
assert.ok(pageHandler, 'page handler missing');
assert.match(pageHandler, /sendFlashWebPage/);
assert.doesNotMatch(pageHandler, /app_|\.tick\(|\.execute\(/);
assert.match(pageHandler, /TransportError[\s\S]*server_\.client\(\)\.stop\(\)/);
assert.match(server, /server_\.header\("X-Applepy-Token"\) != csrf_token_/);
assert.match(server, /server_\.arg\("confirm"\) != "LOOSE_LINKAGES"/);
assert.match(server, /server_\.arg\("confirm"\) != "EXTENDED_RANGE_RISK"/);
assert.match(server, /CommandType::RangeExtended/);
assert.match(server, /pulse_min_us/);
assert.match(header, /解锁 500–2500 µs/);
assert.match(header, /id="calAdvanced"[\s\S]*id="pulse"[\s\S]*id="angleNumber"/);
assert.match(header, /相对中位偏移/);
assert.match(header, /id="angleNumber"[^>]*min="0" max="180"/);
assert.match(server, /CommandType::Offset/);
assert.match(server, /parseServoOffset\(server_\.arg\("deg"\)\.c_str\(\), offset\)/);
assert.match(server, /CommandType::Shank/);
assert.match(server, /parseShankAngle\(server_\.arg\("q_deg"\)\.c_str\(\), q\)/);
assert.match(server, /shank_map_points/);
assert.match(server, /max_shank_q_deg/);
assert.match(server, /shank_direction/);
assert.match(server, /firmware_version/);
assert.match(mockServer, /cmd==='cal_start'[\s\S]*LOOSE_LINKAGES/);
assert.match(mockServer, /cmd==='nudge'[\s\S]*entry\.center_deg=next/);
assert.match(mockServer, /cmd==='cal_save_all'[\s\S]*state\.armed=false[\s\S]*不写NVS/);
assert.match(mockServer, /cmd==='off'[\s\S]*state\.calibrating=false[\s\S]*state\.calibration_pending=false/);
assert.match(mockServer, /channels:\[0,1,2,3,12,9,10,11\]/);
assert.match(script, /action\('offset', \{deg:String\(offset\)\}\)/);
assert.match(script, /action\('shank', \{q_deg:String\(q\)\}\)/);
assert.match(script, /action\('range_extended', \{confirm:'EXTENDED_RANGE_RISK'\}\)/);
assert.match(server, /calibration_pending/);
assert.match(server, /op == "cal_save_all"\) command\.type = CommandType::CalSaveAll/);
assert.match(server, /op == "off_channel"\) command\.type = CommandType::OffChannel/);
assert.match(server, /op == "off"\) command\.type = CommandType::Off/);
assert.match(script, /s\.manual_hold\s*\?/);
assert.match(script, /else if \(s\.calibrating\) badge\(els\.armed, '8 路中位标定输出', 'bad'\)/);
assert.match(script, /els\.send\.disabled[^;]*calibrating/);
assert.match(script, /action\('cal_start', \{confirm:'LOOSE_LINKAGES'\}\)/);
assert.match(script, /action\('cal_save_all'\)/);
assert.match(script, /!state\.calibration_pending/);
assert.match(script, /els\.offSelected\.disabled[^;]*state\.active_channel !== selected\(\)/);
assert.match(script, /els\.offSelected\.addEventListener\('click',\s*\(\) => action\('off_channel'\)\)/);
assert.match(script, /els\.mOff\.disabled = false/);
assert.match(script, /els\.calStop\.disabled = false/);
assert.match(script, /cmd === 'off' \|\| cmd === 'motion_stop'/);
assert.match(script, /if \(epoch !== requestEpoch\) return; \/\/ A later OFF supersedes this reply/);
assert.match(script, /activeController\.abort\(\)/);
assert.match(script, /els\.arm\.disabled = blocked \|\| !ready \|\| armed;/);
assert.doesNotMatch(script, /els\.arm\.disabled[^;]*!chosen/);
assert.match(script, /const verified=Array\.isArray\(actual\)/);
assert.match(script, /if \(verified && submittedDraftRevision===motionDraftRevision\) motionDraftDirty=false/);
assert.match(script, /submittedDraftRevision===motionDraftRevision/);
assert.match(script, /submittedProfileRevision===profileDraftRevision/);
assert.match(script, /bCgApply\.addEventListener\('click',\(\)=>action\('body_set',bodyFields\(\)\)\)/);
assert.match(script, /addEventListener\('click',event=>\{event\.preventDefault\(\);selectDrive\(button\);\}\)/);
assert.match(script, /addEventListener\('contextmenu'/);
assert.match(script, /visibilitychange/);
assert.match(script, /driveGestureGeneration/);
assert.match(script, /driveRenewInFlight/);
assert.match(script, /停止周期推进并保持最后帧；舵机仍有力/);
assert.match(script, /软件全关/);
assert.match(script, /正在检查步态… 原运动状态继续/);
assert.match(script, /const blocked = busy \|\| livePoseInFlight \|\| livePoseQueued \|\|[\s\S]*Date\.now\(\) < stopUntilMs \|\| previewChecking/);
assert.match(script, /检查已取消/);
assert.match(script, /检查未通过/);
assert.match(script, /mReset\.addEventListener\('click',\(\)=>\{motionDraftDirty=false;syncMotionDraft\(state\);/);
assert.match(script, /bindDraftPair\(els\.mHeight,els\.mHeightRange,queueLivePose\)/);
assert.match(script, /bindDraftPair\(els\.bPitch,els\.bPitchRange,queueLivePose\)/);
assert.match(script, /bindDraftPair\(els\.bRoll,els\.bRollRange,queueLivePose\)/);
assert.match(script, /bindDraftPair\(els\.bX,els\.bXRange,queueLivePose\)/);
assert.doesNotMatch(header, /id="bApply"/);
assert.match(script, /const draftBlocked = motionDraftDirty \|\| bodyDraftDirty \|\| profileDraftDirty/);
assert.match(script, /els\.mStart\.disabled = startBlocked/);
assert.match(script, /mPreviewMode\.value/);
assert.match(script, /m\.command_deg/);
assert.match(script, /if\s*\(s\.fault\)\s*badge\(els\.armed,\s*'故障：输出可能仍在持续',\s*'bad'\)/);
assert.match(script, /if\s*\(s\.fault\)\s*badge\(els\.pca,\s*'PCA9685\/输出状态未确认',\s*'bad'\)/);
assert.match(script, /s\.fault\s*\?\s*'未知（可能仍在输出）'/);
assert.match(script, /s\.fault\s*\?\s*'故障：输出状态未知'/);

// The page script must not reach for removed DOM controls.
const idList = [...header.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]);
const ids = new Set(idList);
assert.equal(ids.size,idList.length,'all DOM ids must be unique');
for (const [, id] of script.matchAll(/\bels\.([A-Za-z][A-Za-z0-9]*)/g)) {
  assert.ok(ids.has(id), `script references a missing #${id}`);
}

// A pending write or status poll must not hold back the global OFF action.
// Abort on the browser side does not prove MCU cancellation: a late reply
// must never overwrite OFF feedback or re-enable writes from stale state.
async function stopPreemptsPendingCalibration() {
  const elements = new Map();
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      value:'', checked:false, disabled:false, dataset:{}, textContent:'', className:'',
      addEventListener(){}, replaceChildren(){}, add(){}
    });
    return elements.get(id);
  };
  let releaseCalibration;
  let pollAborted = false;
  const sent = [];
  const fakeFetch = (url, options = {}) => {
    if (url === '/api/state') return new Promise((resolve, reject) => {
      options.signal.addEventListener('abort', () => {
        pollAborted = true;
        reject(new Error('aborted status GET'));
      });
    });
    if (url !== '/api/action') throw new Error('unexpected URL');
    const cmd = new URLSearchParams(options.body).get('cmd');
    sent.push(cmd);
    if (cmd === 'cal_start') return new Promise(resolve => { releaseCalibration = resolve; });
    if (cmd === 'off') return Promise.resolve({ok:true, json:async () => ({ok:true,message:'OFF accepted'})});
    throw new Error('unexpected command: ' + cmd);
  };
  const simulated = script.replace(
    /  controls\(\); poll\(\);\s*\}\)\(\);/,
    '  globalThis.__testUi = {action, controls, poll, setState:s=>{state=s;}};\n})();');
  assert.notEqual(simulated, script, 'inline script test hook insertion failed');
  const context = {
    document:{getElementById:element,querySelectorAll:()=>[]},
    fetch:fakeFetch, AbortController, URLSearchParams,
    setTimeout:()=>1, clearTimeout(){}, setInterval:()=>2, clearInterval(){},
  };
  vm.runInNewContext(simulated, context, {filename:'DebugWebPage.stop-smoke.js'});
  const ui = context.__testUi;
  ui.setState({pca:true, imu_ok:true, fault:false, armed:false, calibrating:false,
    calibration_pending:false, channels:[], active_channel:-1});
  const pendingGet = ui.poll();
  const pendingStart = ui.action('cal_start', {confirm:'LOOSE_LINKAGES'});
  assert.deepEqual(sent, ['cal_start']);
  ui.controls();
  assert.equal(element('mOff').disabled, false, 'global OFF must remain enabled during pending POST');
  assert.equal(element('calStop').disabled, false, 'calibration OFF must remain enabled');
  await ui.action('off');
  await pendingGet;
  assert.equal(pollAborted, true, 'priority OFF must release a hanging status GET');
  assert.deepEqual(sent, ['cal_start', 'off'], 'OFF must dispatch before calibration request resolves');
  releaseCalibration({ok:true, json:async () => ({ok:true,message:'old calibration accepted'})});
  await pendingStart;
  assert.notEqual(element('message').textContent, 'old calibration accepted', 'late response must not overwrite OFF');
  assert.equal(element('arm').disabled, true, 'stale state must not permit new writes after OFF');
}

function rangeAndAngleDraftsNeverMove() {
  const elements = new Map();
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      value:'', checked:false, disabled:false, dataset:{}, textContent:'', className:'',
      handlers:{}, addEventListener(type, fn){this.handlers[type] = fn;},
      replaceChildren(){}, add(){}
    });
    return elements.get(id);
  };
  let calls = 0;
  const simulated = script.replace(
    /  controls\(\); poll\(\);\s*\}\)\(\);/,
    '  globalThis.__testUi = {syncRange, syncRelativeDraft, offsetPreview, syncShankDraft, shankPreview, controls, setState:s=>{state=s;}};\n})();');
  const context = {document:{getElementById:element,querySelectorAll:()=>[]},
    fetch:()=>{calls++; throw new Error('draft must not send HTTP');},
    AbortController, URLSearchParams, setTimeout:()=>1, clearTimeout(){},
    setInterval:()=>2, clearInterval(){}};
  vm.runInNewContext(simulated, context);
  const ui = context.__testUi;
  const base = {pca:true, imu_ok:true, fault:false, armed:false,
    channels:[{channel:0, center_us:1489}], active_channel:-1,
    extended_range:false, pulse_min_us:1000, pulse_max_us:2000};
  element('channel').value = '0';
  ui.setState(base); ui.syncRange(); ui.controls();
  assert.equal(element('pulseNumber').value, '1489');
  assert.equal(element('rangeExtended').disabled, true);
  element('rangeConfirm').checked = true; ui.controls();
  assert.equal(element('rangeExtended').disabled, false);
  ui.setState({...base, armed:true}); ui.controls();
  assert.equal(element('rangeExtended').disabled, true);
  ui.setState({...base, extended_range:true, pulse_min_us:500, pulse_max_us:2500});
  ui.syncRange(); ui.controls();
  assert.equal(element('pulse').min, '500');
  assert.equal(element('pulseNumber').max, '2500');
  assert.equal(element('angleNumber').min, '0');
  assert.equal(element('angleNumber').max, '180');
  assert.equal(element('rangeNormal').disabled, false);
  for (const [degrees, us] of [['0','500'], ['70','1278'], ['90','1500'], ['180','2500']]) {
    element('angleNumber').value = degrees;
    element('angleNumber').handlers.input();
    assert.equal(element('pulseNumber').value, us);
  }
  element('angleNumber').value = '181'; element('angleNumber').handlers.input();
  assert.equal(element('pulseNumber').value, '', 'invalid angle must invalidate stale pulse');
  element('pulseNumber').value = '1200'; element('pulseNumber').handlers.input();
  assert.equal(element('angleNumber').value, '63.0');
  element('pulseNumber').value = ''; element('pulseNumber').handlers.input();
  assert.equal(element('angleNumber').value, '');
  ui.setState(base); ui.syncRange();
  assert.equal(element('pulseNumber').value, '1489');
  ui.setState({...base, channels:[{channel:0, center_us:500}]});
  element('channel').handlers.change();
  assert.equal(element('pulseNumber').value, '1500', 'locked range must not silently use wide saved center');
  assert.equal(calls, 0, 'editing, selection, range sync and polling UI never send PWM');

  const currentBoard = {...base, pulse_min_us:500, pulse_max_us:2500,
    channels:[{channel:3, joint:'L2S', center_deg:136, center_us:2011,
      relative_available:true, shank:true, min_offset_deg:0, max_offset_deg:44}]};
  element('channel').value = '3';
  ui.setState(currentBoard); ui.syncRange(); ui.syncRelativeDraft(); ui.controls();
  assert.equal(element('rangeUnlock').hidden, true, 'full-range board must not require hidden unlock');
  assert.equal(element('angleNumber').max, '180');
  assert.equal(element('offsetNumber').value, '0');
  assert.equal(element('offsetNumber').max, '44');
  assert.match(element('offsetPreview').textContent, /136\.0°/);
  element('offsetNumber').value = '30'; element('offsetNumber').handlers.input();
  assert.match(element('offsetPreview').textContent, /166\.0°.*2344/);
  ui.syncRelativeDraft();
  assert.equal(element('offsetNumber').value, '30', 'ordinary polls preserve valid offset draft');
  element('offsetNumber').value = '-1'; assert.equal(ui.offsetPreview(), null);
  element('offsetNumber').value = '50'; assert.equal(ui.offsetPreview(), null);
  element('offsetNumber').value = '1e1'; assert.equal(ui.offsetPreview(), null);
  ui.setState({...currentBoard, channels:[{...currentBoard.channels[0], center_deg:120,
    center_us:1833, max_offset_deg:50}]});
  ui.syncRelativeDraft();
  assert.equal(element('offsetNumber').value, '0', 'a changed calibrated center resets draft, not PWM');
  element('offsetNumber').value = '30'; element('offsetNumber').handlers.input();
  assert.match(element('offsetPreview').textContent, /150\.0°.*2167/);
  const knots = [{u_deg:0,q_deg:0},{u_deg:30,q_deg:9},{u_deg:50,q_deg:23}];
  const shankCases = [
    {channel:1,joint:'L1S',center_deg:95,center_us:1556,shank_direction:-1,command:55,pulse:1111},
    {channel:3,joint:'L2S',center_deg:105,center_us:1667,shank_direction:1,command:145,pulse:2111},
    {channel:9,joint:'L3S',center_deg:110,center_us:1722,shank_direction:1,command:150,pulse:2167},
    {channel:11,joint:'L4S',center_deg:100,center_us:1611,shank_direction:-1,command:60,pulse:1167},
  ];
  for (const sample of shankCases) {
    element('channel').value = String(sample.channel);
    ui.setState({...currentBoard, channels:[{...sample, relative_available:true, shank:true,
      min_offset_deg:sample.shank_direction < 0 ? -50 : 0,
      max_offset_deg:sample.shank_direction < 0 ? 0 : 50,
      shank_q_available:true, shank_q_photo_candidate:true, max_shank_q_deg:23,
      shank_map_points:knots}]});
    ui.syncShankDraft();
    assert.equal(element('shankNumber').value, '', 'channel change clears q draft');
    assert.equal(element('shankNumber').max, '23');
    element('shankNumber').value = '0'; assert.equal(ui.shankPreview(), 0);
    assert.match(element('shankPreview').textContent,
      new RegExp('命令 ' + sample.center_deg.toFixed(2) + '°.*' + sample.center_us));
    element('shankNumber').value = '16'; assert.equal(ui.shankPreview(), 16);
    assert.match(element('shankPreview').textContent,
      new RegExp('u=40\\.00°.*命令 ' + sample.command.toFixed(2) + '°.*' + sample.pulse));
  }
  for (const edge of [
    {channel:1,joint:'L1S',center_deg:37,center_us:911,shank_direction:-1,command:0,pulse:500},
    {channel:3,joint:'L2S',center_deg:143,center_us:2089,shank_direction:1,command:180,pulse:2500},
  ]) {
    element('channel').value = String(edge.channel);
    ui.setState({...currentBoard, channels:[{...edge, relative_available:true, shank:true,
      min_offset_deg:edge.shank_direction < 0 ? -37 : 0,
      max_offset_deg:edge.shank_direction < 0 ? 0 : 37,
      shank_q_available:true, shank_q_photo_candidate:true, max_shank_q_deg:13.9,
      shank_map_points:knots}]});
    ui.syncShankDraft();
    assert.equal(element('shankNumber').max, '13.9');
    element('shankNumber').value = '13.9'; assert.equal(ui.shankPreview(), 13.9);
    assert.match(element('shankPreview').textContent,
      new RegExp('u=37\\.00°.*命令 ' + edge.command.toFixed(2) + '°.*' + edge.pulse));
    element('shankNumber').value = '13.91'; assert.equal(ui.shankPreview(), null);
  }
  // Direction remains valid when electrical clipping leaves zero travel.
  for (const zero of [
    {channel:11,joint:'L4S',center_deg:0,center_us:500,shank_direction:-1},
    {channel:9,joint:'L3S',center_deg:180,center_us:2500,shank_direction:1},
  ]) {
    element('channel').value = String(zero.channel);
    ui.setState({...currentBoard, channels:[{...zero, relative_available:true, shank:true,
      min_offset_deg:0,max_offset_deg:0,shank_q_available:true,
      shank_q_photo_candidate:true,max_shank_q_deg:0,shank_map_points:knots}]});
    ui.syncShankDraft();
    element('shankNumber').value = '0'; assert.equal(ui.shankPreview(), 0);
    assert.match(element('shankPreview').textContent, new RegExp(zero.center_us + ' µs'));
  }
  element('shankNumber').value = '1e1'; assert.equal(ui.shankPreview(), null);
  element('channel').value = '0';
  ui.setState({...currentBoard,channels:[{channel:0,center_us:1500,center_deg:90}]});
  ui.syncShankDraft();
  assert.equal(element('shankNumber').value, '', 'switching to a hip clears q draft');
  assert.equal(calls, 0, 'relative draft and new-center sync must never move or save');
}

function motionDraftAndTelemetryNeverMove() {
  const elements = new Map();
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      value:'', checked:false, disabled:false, dataset:{}, textContent:'', className:'',
      children:[], handlers:{},
      addEventListener(type, fn){this.handlers[type] = fn;},
      replaceChildren(...children){this.children = children;}, add(){}
    });
    return elements.get(id);
  };
  const preset = (values,mode='') => ({dataset:{values,mode},disabled:false,handlers:{},
    addEventListener(type,fn){this.handlers[type]=fn;}});
  const slowPreset=preset('3,1,3,108,0,1');
  const walkPreset=preset('6,7.1,0.5,108,0,2','walk');
  const trotPreset=preset('17.9,12.8,0.5,114,0,2','trot');
  const presets=[slowPreset,walkPreset,trotPreset];
  let calls = 0;
  const simulated = script.replace(
    /  controls\(\); poll\(\);\s*\}\)\(\);/,
    '  globalThis.__testUi = {syncMotionDraft, renderMotion, setState:s=>{state=s;}};\n})();');
  const context = {
    document:{getElementById:element,querySelectorAll:s=>s==='.mPreset'?presets:[],createElement:()=>({textContent:''})},
    Option:function(text,value){this.text=text;this.value=value;},
    fetch:()=>{calls++; throw new Error('editing and rendering must not send HTTP');},
    AbortController, URLSearchParams, setTimeout:()=>1, clearTimeout(){},
    setInterval:()=>2, clearInterval(){}, Date
  };
  vm.runInNewContext(simulated, context, {filename:'DebugWebPage.motion-smoke.js'});
  const ui=context.__testUi;
  const first={motion:{mode:'idle',error:'none',pending:false,transition:false,phase:0,
    parameters:[6,2,4,108,0,2],feet:[[1,108],[2,107],[3,106],[4,105]],
    command_deg:[81,82,83,84,85,86,87,88]}};
  ui.setState(first); ui.syncMotionDraft(first);
  assert.deepEqual(['mStep','mLift','mPeriod','mHeight','mForward','mShift'].map(id=>element(id).value),
    ['6','2','4','108','0','2']);
  assert.equal(element('mHeightRange').value,'108','status readback syncs the height slider');
  element('dMode').value='trot';walkPreset.handlers.click();
  assert.equal(element('dMode').value,'walk','Walk preset selects Walk without starting it');
  trotPreset.handlers.click();
  assert.equal(element('dMode').value,'trot','Trot preset selects Trot without starting it');
  slowPreset.handlers.click();
  assert.equal(element('dMode').value,'trot','slow preset keeps the current gait');
  trotPreset.handlers.click();
  assert.equal(element('mHeightRange').value,'114','preset syncs the height slider');
  element('mStep').value='7.5'; element('mStep').handlers.input();
  const polled={motion:{...first.motion,parameters:[5,1,6,105,-1,1]}};
  ui.setState(polled); ui.syncMotionDraft(polled);
  assert.equal(element('mStep').value,'7.5','poll must preserve an edited motion draft');
  assert.equal(element('mLift').value,'12.8','poll must preserve the whole draft, not mix fields');
  assert.equal(element('mHeightRange').value,'114','poll must preserve the unapplied preset draft');
  element('mReset').handlers.click();
  assert.deepEqual(['mStep','mLift','mPeriod','mHeight','mForward','mShift'].map(id=>element(id).value),
    ['5','1','6','105','-1','1'],'explicit reset must resync firmware parameters');
  assert.equal(element('mHeightRange').value,'105','explicit reset resyncs the height slider');
  ui.renderMotion(first);
  assert.equal(element('motionFeet').children.length,4);
  assert.match(element('motionFeet').children[0].textContent,/腿1 F 1\.0 \/ D 108\.0 mm/);
  assert.equal(element('motionAngles').children.length,8);
  assert.match(element('motionAngles').children[7].textContent,/L4S 88\.0°/);
  assert.match(element('motionState').textContent,/当前 已关闭/);
  ui.renderMotion({motion:{...first.motion,preview:{pending:true,stage:'trajectory',frames_checked:12}}});
  assert.match(element('motionState').textContent,/正在检查步态… 原运动状态继续/);
  ui.renderMotion({motion:{...first.motion,preview:{pending:false,result:'cancelled_motion_stop',error:'cancelled'}}});
  assert.match(element('motionState').textContent,/检查已取消/);
  ui.renderMotion({motion:{...first.motion,preview:{pending:false,result:'rejected_imu_stale',error:'imu_stale'}}});
  assert.match(element('motionState').textContent,/检查未通过：IMU样本过期/);
  assert.doesNotMatch(element('motionState').textContent,/rejected_imu_stale/);
  assert.equal(calls,0,'motion editing, reset, and telemetry rendering must cause zero actions');
}

async function motionApplyAndPreviewSync() {
  const elements = new Map();
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      value:'', checked:false, disabled:false, hidden:false, dataset:{},
      textContent:'', className:'', children:[], handlers:{},
      addEventListener(type, fn){this.handlers[type] = fn;},
      replaceChildren(...children){this.children = children;}, add(){}
    });
    return elements.get(id);
  };
  const posted=[];
  let releaseDelayedApply;
  let releaseDelayedPitch;
  const status={firmware_version:'0.7.1',pca:true,imu_ok:true,fault:false,
    armed:false,calibrating:false,calibration_pending:false,active_channel:-1,
    calibration_mask:0,manual_hold:false,remaining_ms:0,calibration_samples:0,
    calibration_required:100,attitude_ok:false,gyro_calibrated:false,
    accel_norm_mps2:9.8,channels:[],extended_range:false,
    pulse_min_us:500,pulse_max_us:2500,
    imu:{accel:[0,0,9.8],gyro:[0,0,0]},
    motion:{mode:'idle',error:'none',pending:false,transition:false,phase:0,
      parameter_state:'applied_ram',parameters_saved:false,parameter_revision:1,
      diagnostics:{frame_count:0,last_gap_ms:0,max_gap_ms:0,missed_deadlines:0},
      parameters:[5,1,6,105,-1,1],feet:[],command_deg:[],
      drive:{straight_trim:0.25,effective_turn:0,trot_pitch_offset_deg:0,
        trot_effective_pitch_deg:0,trot_pitch_revision:0,active:false}}};
  const fakeFetch=async (url,options={})=>{
    if(url==='/api/action') {
      const form=new URLSearchParams(options.body); posted.push(Object.fromEntries(form));
      if(form.get('cmd')==='motion_set' && form.get('step')!=='9') status.motion.parameters=
        ['step','lift','period','height','forward','shift'].map(name=>Number(form.get(name)));
      if(form.get('cmd')==='motion_set' && form.get('step')==='8')
        return new Promise(resolve=>{releaseDelayedApply=()=>resolve({ok:true,json:async()=>({ok:true,message:'delayed accepted'})});});
      if(form.get('cmd')==='motion_set' && form.get('step')==='9')
        return {ok:false,json:async()=>({ok:false,message:'rejected; unchanged'})};
      if(form.get('cmd')==='trot_pitch_set'&&form.get('value')==='-0.4')
        return new Promise(resolve=>{releaseDelayedPitch=()=>{
          status.motion.drive.trot_pitch_offset_deg=-0.4;
          status.motion.drive.trot_effective_pitch_deg=-0.4;
          status.motion.drive.trot_pitch_revision++;
          resolve({ok:true,json:async()=>({ok:true,message:'delayed pitch accepted'})});};});
      if(form.get('cmd')==='trot_pitch_set'){
        status.motion.drive.trot_pitch_offset_deg=Number(form.get('value'));
        status.motion.drive.trot_effective_pitch_deg=Number(form.get('value'));
        status.motion.drive.trot_pitch_revision++;
      }
      return {ok:true,json:async()=>({ok:true,message:'accepted; no PWM sent'})};
    }
    if(url==='/api/state') return {ok:true,json:async()=>status};
    throw new Error('unexpected URL');
  };
  const simulated=script.replace(
    /  controls\(\); poll\(\);\s*\}\)\(\);/,
    '  globalThis.__testUi={action,controls,render,setState:s=>{state=s;}};\n})();');
  const context={document:{getElementById:element,querySelectorAll:()=>[],
      createElement:()=>({textContent:''})},
    Option:function(text,value){this.text=text;this.value=value;},fetch:fakeFetch,
    AbortController,URLSearchParams,setTimeout:()=>1,clearTimeout(){},
    setInterval:()=>2,clearInterval(){},Date};
  vm.runInNewContext(simulated,context,{filename:'DebugWebPage.motion-action-smoke.js'});
  const ui=context.__testUi;
  ui.setState({...status,motion:{...status.motion,parameters:[6,2,4,108,0,2]}});
  ui.controls();
  assert.equal(element('arm').disabled,false,'global ARM must be enabled without a selected single channel');
  await ui.action('arm');
  assert.deepEqual(posted[0],{cmd:'arm'},'ARM form must not require or include a channel');
  ui.render({...status,armed:true,motion:{...status.motion,mode:'walk'}});
  assert.equal(element('armed').textContent,'八路运动输出');
  ui.render({...status,armed:true,motion:{...status.motion,mode:'hold'}});
  assert.equal(element('armed').textContent,'八路姿态保持');
  ui.setState({...status,motion:{...status.motion,parameters:[6,2,4,108,0,2]}});
  ['mStep','mLift','mPeriod','mHeight','mForward','mShift'].forEach((id,i)=>{
    element(id).value=['7','2.5','5','107','0.5','1.5'][i];
  });
  element('mStep').handlers.input(); // Mark the complete parameter draft dirty.
  await ui.action('motion_set',{step:'7',lift:'2.5',period:'5',height:'107',forward:'0.5',shift:'1.5'});
  assert.deepEqual(['mStep','mLift','mPeriod','mHeight','mForward','mShift'].map(id=>element(id).value),
    ['7','2.5','5','107','0.5','1.5'],'successful Apply must verify and retain matching firmware readback');
  element('mPreviewMode').value='leg4';
  await ui.action('motion_preview',{mode:element('mPreviewMode').value});
  assert.equal(posted[1].cmd,'motion_set');
  assert.deepEqual(posted[1],{cmd:'motion_set',step:'7',lift:'2.5',period:'5',height:'107',forward:'0.5',shift:'1.5'});
  assert.deepEqual(posted[2],{cmd:'motion_preview',mode:'leg4'});
  element('mStep').value='9'; element('mStep').handlers.input();
  await ui.action('motion_set',{step:'9',lift:'2.5',period:'5',height:'107',forward:'0.5',shift:'1.5'});
  assert.equal(element('mStep').value,'9','failed Apply must preserve the complete draft');
  ui.setState({...status,armed:true,motion:{...status.motion,mode:'hold'}}); ui.controls();
  assert.equal(element('mApply').disabled,false,'stable Hold permits zero-PWM parameter apply');
  assert.equal(element('mSave').disabled,true,'NVS save still requires all outputs OFF');
  assert.equal(element('mStart').disabled,true,'dirty draft blocks motion start');
  // Even if a scripted/programmatic edit bypasses disabled controls while an
  // old POST/GET is pending, draft revision prevents stale success from
  // clearing or overwriting the newer draft.
  element('mStep').value='8'; element('mStep').handlers.input();
  const delayed=ui.action('motion_set',{step:'8',lift:'2',period:'1.2',height:'108',forward:'0',shift:'2'});
  assert.equal(element('mStep').disabled,true,'motion inputs are disabled while request is busy');
  element('mStep').value='8.5'; element('mStep').handlers.input();
  releaseDelayedApply(); await delayed;
  assert.equal(element('mStep').value,'8.5','late success must not overwrite a newer draft');
  assert.equal(element('mStart').disabled,true,'late success must not unlock motion with a newer draft');
  element('dTrotPitch').value='-0.5';element('dTrotPitch').handlers.input();ui.controls();
  assert.equal(element('mStart').disabled,true,'dirty Trot Pitch draft blocks start');
  await ui.action('trot_pitch_set',{value:'-0.5'});
  assert.equal(element('dTrotPitch').value,'-0.5');
  element('dTrotPitch').value='-0.4';element('dTrotPitch').handlers.input();
  const delayedPitch=ui.action('trot_pitch_set',{value:'-0.4'});
  element('dTrotPitch').value='-0.3';element('dTrotPitch').handlers.input();
  releaseDelayedPitch();await delayedPitch;
  assert.equal(element('dTrotPitch').value,'-0.3','late Pitch ACK must preserve newer draft');
  assert.equal(element('mStart').disabled,true,'late Pitch ACK must keep start locked');
}

async function simplifiedViewsAndStandBranching() {
  const elements=new Map();
  const element=id=>{
    if(!elements.has(id))elements.set(id,{value:id==='dMode'?'trot':'',checked:false,disabled:false,
      hidden:id==='calibrationView',dataset:{},textContent:'',className:'',children:[],handlers:{},attributes:{},
      addEventListener(type,fn){this.handlers[type]=fn;},replaceChildren(...children){this.children=children;},add(){},
      setAttribute(name,value){this.attributes[name]=value;}});
    return elements.get(id);
  };
  const posted=[];
  const status={firmware_version:'0.8.14-test',pca:true,imu_ok:true,fault:false,armed:false,
    calibrating:false,calibration_pending:false,calibration_mask:0,active_channel:-1,
    manual_hold:false,remaining_ms:0,calibration_samples:100,calibration_required:100,
    attitude_ok:true,gyro_calibrated:true,accel_norm_mps2:9.8,channels:[],extended_range:false,
    pulse_min_us:500,pulse_max_us:2500,imu:{accel:[0,0,9.8],gyro:[0,0,0]},
    attitude:{roll_deg:0,pitch_deg:0,yaw_deg:0,yaw_reference_ms:1},
    motion:{mode:'idle',error:'none',pending:false,transition:false,phase:0,
      parameter_state:'loaded_nvs',parameters_saved:true,parameter_revision:0,
      diagnostics:{frame_count:0,last_gap_ms:0,max_gap_ms:0,missed_deadlines:0},
      parameters:[6,2,1.2,108,0,2],feet:[],command_deg:[],
      body:{pitch_deg:0,roll_deg:0,x_mm:0,cg_enabled:false,cg_shift_mm:2.5,revision:0,
        length_mm:142,width_mm:92.8,dimensions_measured:false,cg_actual_x_min_mm:0,cg_actual_x_max_mm:0},
      balance:{capture_pending:false,capture_samples:0,capture_required:20,reference_ready:false,
        enabled:false,enable_pending:false,pitch_correction_deg:0,roll_correction_deg:0,roll_sensor_sign:-1,error:'none'},
      preview:{error:'not_run',leg:-1,joint:-1,phase:0,min_support_margin_mm:0},
      drive:{active:false,epoch:1,seq:0,lease_remaining_ms:0,reason:'idle',forward:0,turn:0,stepping:false,
        straight_trim:0.25,effective_turn:0,trot_pitch_offset_deg:0,trot_effective_pitch_deg:0,trot_pitch_revision:0,
        moving_cg_enabled:false,moving_cg_pending:false,moving_cg_shift_mm:0,moving_cg_error:'none',
        phase_waiting:false,cg_stage:0,nominal_elapsed_s:0,actual_elapsed_s:0}}};
  const fakeFetch=async(url,options={})=>{
    if(url==='/api/action'){
      const form=new URLSearchParams(options.body);posted.push(Object.fromEntries(form));
      if(form.get('cmd')==='cal_save_all')status.calibration_pending=false;
      if(form.get('cmd')==='straight_trim_set'){
        status.motion.drive.straight_trim=Number(form.get('value'));
        status.motion.drive.effective_turn=-Number(form.get('value'));
      }
      if(form.get('cmd')==='trot_pitch_set'){
        status.motion.drive.trot_pitch_offset_deg=Number(form.get('value'));
        status.motion.drive.trot_effective_pitch_deg=Number(form.get('value'));
        status.motion.drive.trot_pitch_revision++;
      }
      return{ok:true,json:async()=>({ok:true,message:'accepted'})};
    }
    if(url==='/api/state')return{ok:true,json:async()=>status};
    throw new Error('unexpected URL');
  };
  const simulated=script.replace(/  controls\(\); poll\(\);\s*\}\)\(\);/,
    '  globalThis.__testUi={controls,showView,render,setState:s=>{state=s;}};\n})();');
  const context={document:{getElementById:element,querySelectorAll:()=>[],createElement:()=>({textContent:''})},
    Option:function(text,value){this.text=text;this.value=value;},fetch:fakeFetch,AbortController,URLSearchParams,Date,
    setTimeout:()=>1,clearTimeout(){},setInterval:()=>2,clearInterval(){}};
  vm.runInNewContext(simulated,context,{filename:'DebugWebPage.simple-ui-smoke.js'});
  const ui=context.__testUi;

  element('message').textContent='正在读取状态；页面不会自动使能或运动。';
  ui.render(status);
  assert.equal(element('message').textContent,'先点站立，再选择步态开始。');
  assert.equal(element('dStraightTrim').value,'0.25','board readback restores the built-in trim draft');
  assert.match(element('straightTrimState').textContent,/板端RAM 0\.25.*重启恢复默认0\.25/);
  element('message').textContent='保留最近操作反馈';ui.render(status);
  assert.equal(element('message').textContent,'保留最近操作反馈','poll must not overwrite later action feedback');

  ui.setState(null);ui.controls();
  assert.equal(element('mStand').disabled,true,'unknown status must disable Stand');
  await element('mStand').handlers.click();
  assert.equal(posted.length,0,'unknown status must not write');

  ui.setState(status);ui.controls();
  status.motion.preview.pending=true;ui.controls();
  assert.equal(element('arm').disabled,true,'checking locks ordinary ARM');
  assert.equal(element('mApply').disabled,true,'checking locks ordinary parameter actions');
  assert.equal(element('mStop').disabled,false);assert.equal(element('mOff').disabled,false);
  assert.equal(element('bOff').disabled,false);assert.equal(element('dMovingOff').disabled,false);
  status.motion.preview.pending=false;ui.controls();
  await element('mStand').handlers.click();
  assert.equal(posted.at(-1).cmd,'arm_stand','Stand while OFF explicitly uses arm_stand');
  status.armed=true;status.motion.mode='stand';ui.setState(status);ui.controls();
  await element('mStand').handlers.click();
  assert.equal(posted.at(-1).cmd,'stand','Stand while armed uses stand without re-arming');
  element('dStraightTrim').value='0.05';element('dStraightTrim').handlers.input();ui.controls();
  assert.equal(element('mStart').disabled,true,'unapplied trim draft locks gait start');
  await element('dStraightApply').handlers.click();
  assert.equal(posted.at(-1).cmd,'straight_trim_set');
  assert.equal(posted.at(-1).value,'0.05');
  assert.equal(element('mStart').disabled,false,'verified trim readback unlocks gait start');
  assert.match(element('straightTrimState').textContent,/板端RAM 0\.05.*当前有效turn -0\.05/);
  element('dTrotPitch').value='-0.5';element('dTrotPitch').handlers.input();ui.controls();
  assert.equal(element('mStart').disabled,true,'unapplied Trot Pitch draft locks gait start');
  await element('dTrotPitchApply').handlers.click();
  assert.equal(posted.at(-1).cmd,'trot_pitch_set');
  assert.equal(posted.at(-1).value,'-0.5');
  assert.equal(element('mStart').disabled,false,'verified Trot Pitch readback unlocks gait start');
  assert.match(element('trotPitchState').textContent,/板端RAM -0\.5°.*Trot有效Pitch -0\.5°.*rev 1/);
  element('dMode').value='walk';await element('mStart').handlers.click();
  assert.equal(posted.at(-1).cmd,'walk','single gait selector drives the start command');

  const beforeView=posted.length;ui.showView('calibration');
  assert.equal(element('controlView').hidden,true);assert.equal(element('calibrationView').hidden,false);
  ui.showView('control');assert.equal(posted.length,beforeView,'view switching alone performs zero actions');

  element('mStep').value='7';element('mStep').handlers.input();ui.controls();
  assert.equal(element('mStart').disabled,true,'dirty motion draft blocks start');
  assert.equal(element('mSave').disabled,true,'dirty motion draft cannot save stale applied values');
  element('bCgShift').value='1';element('bCgShift').handlers.input();ui.controls();
  assert.equal(element('dirtyNotice').hidden,false,'hidden advanced dirty state remains visible on main view');
  await element('bCgApply').handlers.click();
  assert.equal(posted.at(-1).cmd,'body_set','CG settings change only on explicit Apply');

  status.armed=false;status.motion.mode='idle';status.calibration_pending=true;ui.setState(status);ui.controls();
  ui.showView('calibration');await element('calSaveAll').handlers.click();
  assert.equal(posted.at(-1).cmd,'cal_save_all');
  assert.equal(element('controlView').hidden,false,'successful calibration save exits to control view');
}

function bodyBalanceDraftNeverMoves() {
  const elements=new Map();
  const element=id=>{
    if(!elements.has(id))elements.set(id,{value:'',checked:false,disabled:false,dataset:{},
      textContent:'',className:'',handlers:{},addEventListener(type,fn){this.handlers[type]=fn;},
      replaceChildren(){},add(){}});
    return elements.get(id);
  };
  let calls=0;
  const simulated=script.replace(/  controls\(\); poll\(\);\s*\}\)\(\);/,
    '  globalThis.__testUi={syncBodyDraft,renderAdvanced,controls,setState:s=>{state=s;}};\n})();');
  const context={document:{getElementById:element,querySelectorAll:()=>[],createElement:()=>({})},
    fetch:()=>{calls++;throw new Error('draft must not send');},AbortController,URLSearchParams,
    setTimeout:()=>1,clearTimeout(){},setInterval:()=>2,clearInterval(){},Date};
  vm.runInNewContext(simulated,context,{filename:'DebugWebPage.body-smoke.js'});
  const ui=context.__testUi;
  const base={pca:true,imu_ok:true,fault:false,armed:false,calibrating:false,
    active_channel:-1,channels:[],motion:{mode:'idle',pending:false,transition:false,
      body:{pitch_deg:0,roll_deg:0,x_mm:0,cg_enabled:false,cg_shift_mm:2.5,revision:0,
        length_mm:142,width_mm:92.8,dimensions_measured:false,cg_actual_x_min_mm:0,cg_actual_x_max_mm:0},
      balance:{capture_pending:false,capture_samples:0,capture_required:20,reference_ready:false,
        enabled:false,enable_pending:false,reference_pitch_deg:0,reference_roll_deg:0,
        pitch_correction_deg:0,roll_correction_deg:0,roll_sensor_sign:-1,error:'disabled_on_boot'},
      preview:{error:'not_run',leg:-1,joint:-1,phase:0,min_support_margin_mm:0}}};
  ui.setState(base);ui.syncBodyDraft(base);ui.renderAdvanced(base);ui.controls();
  assert.equal(element('bCgShift').value,'2.5');
  assert.equal(element('bPitchRange').value,'0');assert.equal(element('bRollRange').value,'0');
  assert.equal(element('bXRange').value,'0');
  assert.equal(element('bRollSign').value,'-1','first board state must select confirmed default without POST');
  assert.match(element('balanceState').textContent,/Roll已确认 body=-sensor/);
  element('bRollSign').value='1';element('bRollSign').handlers.change();
  ui.renderAdvanced(base);
  assert.equal(element('bRollSign').value,'1','poll must preserve unapplied Roll sign draft');
  element('bCgShift').value='1.5';element('bCgShift').handlers.input();
  const polled={...base,motion:{...base.motion,body:{...base.motion.body,pitch_deg:-1}}};
  ui.setState(polled);ui.syncBodyDraft(polled);
  assert.equal(element('bCgShift').value,'1.5','poll must preserve CG body draft');
  element('bCgPreset').handlers.click();
  assert.equal(element('mPeriod').value,'3.2');
  assert.equal(element('bCg').checked,true);
  assert.equal(calls,0,'body/CG edits and preset never send HTTP');
  element('bReset').handlers.click();
  assert.equal(element('bPitch').value,'-1');
  assert.equal(element('bPitchRange').value,'-1');assert.equal(element('bRollRange').value,'0');
  assert.equal(element('bRollSign').value,'-1');
  const moving={...base,motion:{...base.motion,mode:'trot'}};ui.setState(moving);ui.controls();
  assert.equal(element('bPitch').disabled,element('bPitchRange').disabled);
  assert.equal(element('bRoll').disabled,element('bRollRange').disabled);
  assert.equal(calls,0,'sliders, status sync, reset and disabled-state refresh must send no HTTP');
}

function profileDraftNeverMoves() {
  const elements=new Map();const element=id=>{if(!elements.has(id))elements.set(id,{value:'',checked:false,
    disabled:false,dataset:{},textContent:'',className:'',handlers:{},addEventListener(type,fn){this.handlers[type]=fn;},
    replaceChildren(){},add(){}});return elements.get(id);};
  let calls=0;
  const simulated=script.replace(/  controls\(\); poll\(\);\s*\}\)\(\);/,
    '  globalThis.__testUi={syncProfileDraft,renderAdvanced,setState:s=>{state=s;}};\n})();');
  const context={document:{getElementById:element,querySelectorAll:()=>[],createElement:()=>({})},
    fetch:()=>{calls++;throw new Error('profile draft must not send');},AbortController,URLSearchParams,
    setTimeout:()=>1,clearTimeout(){},setInterval:()=>2,clearInterval(){},Date};
  vm.runInNewContext(simulated,context);
  const profile={state:'built_in_default',saved:false,revision:0,planner_slew_mm_s:20,wait_timeout_ms:15000,
    moving_angle_gain:1.1,moving_rate_gain:5,moving_tau_s:0.1,moving_max_shift_mm:0.5,moving_slew_mm_s:2,
    pitch_kp:0.8,pitch_kd_s:0.08,roll_kp:0.9,roll_kd_s:0.04,trot_ff_enabled:false,
    trot_ff_forward:4,trot_ff_backward:4,trot_ff_turn:2,trot_ff_max_mm:2};
  const base={motion:{profile,body:{pitch_deg:0,roll_deg:0,x_mm:0,cg_enabled:false,cg_shift_mm:2.5,
    revision:0,length_mm:142,width_mm:92.8,dimensions_measured:false,cg_actual_x_min_mm:0,cg_actual_x_max_mm:0},
    balance:{roll_sensor_sign:-1,pitch_correction_deg:0,roll_correction_deg:0,error:'none'},
    preview:{error:'not_run',leg:-1,joint:-1,phase:0,min_support_margin_mm:0},drive:{}}};
  const ui=context.__testUi;ui.setState(base);ui.syncProfileDraft(base);
  assert.equal(element('pRollKp').value,'0.9');assert.equal(element('pPitchKp').value,'0.8');
  element('pPitchKp').value='0.7';element('pPitchKp').handlers.input();
  ui.setState({...base,motion:{...base.motion,profile:{...profile,pitch_kp:1.2}}});ui.syncProfileDraft(base);
  assert.equal(element('pPitchKp').value,'0.7','poll preserves complete profile draft');
  assert.equal(calls,0);
}

async function livePoseCoalescesAndLatestWins() {
  const elements=new Map();const element=id=>{if(!elements.has(id))elements.set(id,{value:id==='dMode'?'trot':'',checked:false,
    disabled:false,hidden:false,dataset:{},textContent:'',className:'',handlers:{},attributes:{},children:[],
    addEventListener(type,fn){this.handlers[type]=fn;},replaceChildren(...v){this.children=v;},add(){},
    setAttribute(name,value){this.attributes[name]=value;}});return elements.get(id);};
  let nextTimer=1;const timers=new Map();
  const fireTimer=async()=>{const entry=[...timers.entries()][0];assert.ok(entry,'expected pending timer');timers.delete(entry[0]);await entry[1]();};
  let delayed=false,reject=false,releaseAction=null,delayedOrdinary=false,releaseOrdinary=null;
  let concurrent=0,maxConcurrent=0;const posted=[];
  const status={firmware_version:'0.8.14-test',pca:true,imu_ok:true,fault:false,armed:false,calibrating:false,
    calibration_pending:false,active_channel:-1,manual_hold:false,channels:[],attitude_ok:false,
    motion:{mode:'idle',pending:false,transition:false,error:'none',parameter_state:'applied_ram',parameters_saved:false,
      parameter_revision:1,parameters:[17.9,12.8,.5,114,0,2],feet:[],command_deg:[],diagnostics:{},
      body:{pitch_deg:0,roll_deg:0,x_mm:8,cg_enabled:false,cg_shift_mm:2.5,revision:1,
        length_mm:142,width_mm:92.8,dimensions_measured:false,cg_actual_x_min_mm:0,cg_actual_x_max_mm:0},
      balance:{capture_pending:false,enabled:false,enable_pending:false,reference_ready:false,roll_sensor_sign:-1,
        pitch_correction_deg:0,roll_correction_deg:0,error:'none'},
      preview:{pending:false,error:'not_run',leg:-1,joint:-1,phase:0,min_support_margin_mm:0},profile:{},
      drive:{active:false,epoch:1,seq:0,straight_trim:.25,effective_turn:0,trot_pitch_offset_deg:0,
        trot_effective_pitch_deg:0,trot_pitch_revision:0,moving_cg_enabled:false,moving_cg_pending:false,
        moving_cg_shift_mm:0,moving_cg_error:'none'}}};
  const applyForm=form=>{status.motion.parameters[3]=Number(form.get('height'));
    status.motion.body.pitch_deg=Number(form.get('pitch'));status.motion.body.roll_deg=Number(form.get('roll'));
    status.motion.body.x_mm=Number(form.get('x'));};
  const fakeFetch=async(url,options={})=>{
    if(url==='/api/state')return{ok:true,json:async()=>status};
    const form=new URLSearchParams(options.body),cmd=form.get('cmd');
    posted.push(Object.fromEntries(form));concurrent++;maxConcurrent=Math.max(maxConcurrent,concurrent);
    if(cmd==='profile_load'){
      if(delayedOrdinary)return new Promise(resolve=>{releaseOrdinary=()=>{concurrent--;
        resolve({ok:true,json:async()=>({ok:true,message:'profile loaded'})});};});
      concurrent--;return{ok:true,json:async()=>({ok:true,message:'profile loaded'})};
    }
    assert.equal(cmd,'pose_live_set');
    if(delayed)return new Promise(resolve=>{releaseAction=()=>{if(!reject)applyForm(form);concurrent--;
      resolve({ok:!reject,json:async()=>({ok:!reject,message:reject?'tuple rejected':'live applied'})});};});
    if(!reject)applyForm(form);concurrent--;
    return{ok:!reject,json:async()=>({ok:!reject,message:reject?'tuple rejected':'live applied'})};
  };
  const simulated=script.replace(/  controls\(\); poll\(\);\s*\}\)\(\);/,
    '  globalThis.__testUi={action,render,flushLivePose,controls,setState:s=>{state=s;},live:()=>({queued:livePoseQueued,inFlight:livePoseInFlight,bodyDirty:bodyDraftDirty})};\n})();');
  const context={document:{hidden:false,getElementById:element,querySelectorAll:()=>[],createElement:()=>({textContent:''}),addEventListener(){}},
    Option:function(){},fetch:fakeFetch,AbortController,URLSearchParams,Date,
    setTimeout:fn=>{const id=nextTimer++;timers.set(id,fn);return id;},clearTimeout:id=>timers.delete(id),
    setInterval:()=>999,clearInterval(){}};
  vm.runInNewContext(simulated,context,{filename:'DebugWebPage.live-pose-smoke.js'});
  const ui=context.__testUi;ui.setState(status);ui.render(status);
  const liveIds=['mHeight','mHeightRange','bPitch','bPitchRange','bRoll','bRollRange','bX','bXRange'];
  const assertLiveDisabled=message=>liveIds.forEach(id=>assert.equal(element(id).disabled,true,message+' #'+id));
  status.motion.drive.moving_cg_enabled=true;ui.controls();assertLiveDisabled('moving CG disables every live pose control');
  status.motion.drive.moving_cg_enabled=false;status.motion.mode='walk';ui.controls();assertLiveDisabled('non-stable gait disables every live pose control');
  status.motion.mode='idle';status.calibrating=true;ui.controls();assertLiveDisabled('calibration disables every live pose control');
  status.calibrating=false;ui.controls();liveIds.forEach(id=>assert.equal(element(id).disabled,false,'OFF/idle enables '+id));
  element('mHeightRange').value='113';element('mHeightRange').handlers.input();
  element('bPitchRange').value='-2';element('bPitchRange').handlers.input();
  element('bRollRange').value='3';element('bRollRange').handlers.input();
  element('bXRange').value='4';element('bXRange').handlers.input();
  assert.equal(element('bX').value,'4','body_x range updates number');
  element('bX').value='3.5';element('bX').handlers.input();
  assert.equal(element('bXRange').value,'3.5','body_x number updates range');
  assert.equal(ui.live().bodyDirty,false,'realtime body_x does not create a body/CG dirty draft');
  assert.equal(timers.size,1,'rapid slider inputs share one 150ms debounce timer');
  await fireTimer();
  assert.equal(posted.length,1);assert.deepEqual(posted[0],{cmd:'pose_live_set',height:'113',pitch:'-2',roll:'3',x:'3.5'});
  assert.equal(element('mHeight').value,'113');assert.equal(element('bPitch').value,'-2');assert.equal(element('bRoll').value,'3');
  assert.equal(element('bX').value,'3.5');

  delayed=true;element('mHeightRange').value='112';element('mHeightRange').handlers.input();
  const first=fireTimer();await Promise.resolve();
  element('mHeightRange').value='110.5';element('mHeightRange').handlers.input();
  element('bPitchRange').value='-3';element('bPitchRange').handlers.input();
  element('bXRange').value='2';element('bXRange').handlers.input();
  await fireTimer(); // In-flight: keep only the latest tuple queued.
  assert.equal(posted.length,2);assert.equal(maxConcurrent,1,'live pose requests never overlap');
  delayed=false;releaseAction();await first;
  assert.equal(timers.size,1,'completion immediately schedules the latest queued tuple');
  await fireTimer();
  assert.equal(posted.length,3);assert.equal(posted[2].height,'110.5');assert.equal(posted[2].pitch,'-3');
  assert.equal(element('mHeight').value,'110.5','late first ACK cannot overwrite the newer height');
  assert.equal(element('bX').value,'2','late first ACK cannot overwrite newer body_x');

  reject=true;element('bXRange').value='7';element('bXRange').handlers.input();await fireTimer();
  assert.equal(element('bX').value,'2','rejected tuple rolls body_x back to board readback');
  assert.equal(element('bRoll').value,'3','rejected tuple rolls controls back to board readback');
  assert.match(element('message').textContent,/tuple rejected|恢复板端有效值/);
  assert.equal(maxConcurrent,1);assert.equal(ui.live().inFlight,false);assert.equal(ui.live().queued,false);

  // A queued live tuple blocks ordinary actions; conversely an ordinary
  // request already in flight makes the live flush retain/requeue its latest
  // tuple instead of overlapping HTTP mutations.
  reject=false;element('mHeightRange').value='110';element('mHeightRange').handlers.input();
  const beforeBlockedAction=posted.length;
  assert.equal(await ui.action('profile_load'),undefined);
  assert.equal(posted.length,beforeBlockedAction,'ordinary action cannot start while live pose is queued');
  await fireTimer();
  delayedOrdinary=true;const ordinary=ui.action('profile_load');await Promise.resolve();
  assert.equal(posted.at(-1).cmd,'profile_load');
  element('bPitchRange').value='-1';element('bPitchRange').handlers.input();
  await fireTimer();
  assert.equal(posted.at(-1).cmd,'profile_load','busy ordinary action defers live pose POST');
  assert.equal(maxConcurrent,1,'ordinary and live mutations never overlap');
  delayedOrdinary=false;releaseOrdinary();await ordinary;
  assert.equal(timers.size,1,'deferred latest live tuple remains scheduled after ordinary action');
  await fireTimer();
  assert.equal(posted.at(-1).cmd,'pose_live_set');assert.equal(posted.at(-1).pitch,'-1');
  assert.equal(maxConcurrent,1);
}

async function driveGestureRaceSafety() {
  const elements=new Map();
  const makeDrive=dataset=>({value:'',checked:false,disabled:false,dataset,
    textContent:'',className:'',handlers:{},attributes:{},addEventListener(type,fn){this.handlers[type]=fn;},
    replaceChildren(){},add(){},setAttribute(name,value){this.attributes[name]=value;}});
  const forwardButton=makeDrive({forward:'1',turn:'0',step:'0'});
  const stepButton=makeDrive({forward:'0',turn:'0',step:'1'});
  const driveButtons=[forwardButton,stepButton];
  const element=id=>{if(!elements.has(id))elements.set(id,{value:id==='dMode'?'trot':'',checked:false,disabled:false,
    hidden:id==='calibrationView',dataset:{},textContent:'',className:'',handlers:{},attributes:{},
    addEventListener(type,fn){this.handlers[type]=fn;},replaceChildren(){},add(){},setAttribute(name,value){this.attributes[name]=value;}});
    return elements.get(id);};
  const documentHandlers={};let nextTimer=1;const intervals=new Map();let failRenew=false;const posted=[];
  const drive={forward:1,turn:0,stepping:false,active:false,epoch:1,seq:0,lease_remaining_ms:0,
    straight_trim:0.25,effective_turn:0,trot_pitch_offset_deg:0,trot_effective_pitch_deg:0,trot_pitch_revision:0,
    reason:'idle',moving_cg_enabled:false,moving_cg_pending:false,moving_cg_shift_mm:0,moving_cg_error:'disabled',
    phase_waiting:false,cg_stage:0,nominal_elapsed_s:0,actual_elapsed_s:0};
  const status={firmware_version:'0.8.14-mock',pca:true,imu_ok:true,fault:false,armed:true,calibrating:false,
    calibration_pending:false,active_channel:-1,manual_hold:true,remaining_ms:0,channels:[],motion:{mode:'stand',pending:false,
      transition:false,error:'none',parameters:[6,2,3.2,108,0,2],feet:[],command_deg:[],diagnostics:{},
      body:{pitch_deg:0,roll_deg:0,x_mm:0,cg_enabled:false,cg_shift_mm:2.5,revision:0,length_mm:142,width_mm:92.8,
        dimensions_measured:false,cg_actual_x_min_mm:0,cg_actual_x_max_mm:0},
      balance:{reference_ready:true,roll_sensor_sign:-1,pitch_correction_deg:0,roll_correction_deg:0,error:'none'},
      preview:{error:'none',leg:-1,joint:-1,phase:0,min_support_margin_mm:0,duration_ms:1,actual_cycle_s:0},
      profile:{},drive}};
  const fakeFetch=async(url,options={})=>{
    if(url==='/api/state')return{ok:true,json:async()=>status};
    const form=new URLSearchParams(options.body),cmd=form.get('cmd');posted.push(Object.fromEntries(form));
    if(cmd==='drive_begin'){
      drive.active=true;drive.epoch++;drive.seq=1;return{ok:true,json:async()=>({ok:true,message:'begin'})};
    }
    if(cmd==='drive_renew'){
      if(failRenew)throw new Error('network lost');
      drive.seq=Number(form.get('seq'));return{ok:true,json:async()=>({ok:true,message:'renew'})};
    }
    if(cmd==='drive_end'){drive.active=false;drive.epoch++;drive.seq=0;return{ok:true,json:async()=>({ok:true,message:'end'})};}
    if(cmd==='motion_stop'||cmd==='off'){drive.active=false;drive.epoch++;return{ok:true,json:async()=>({ok:true,message:'stop'})};}
    throw new Error('unexpected '+cmd);
  };
  const simulated=script.replace(/  controls\(\); poll\(\);\s*\}\)\(\);/,
    '  globalThis.__testUi={selectDrive,releaseDrive,renewDrive,priorityStop,showView,hasDriveRenew:()=>!!driveRenewTimer,resetForTest:s=>{cancelDriveGestureLocal();if(stopTimer)clearInterval(stopTimer);stopTimer=null;stopUntilMs=0;busy=false;state=s;controls();},setState:s=>{state=s;}};\n})();');
  const context={document:{hidden:false,getElementById:element,querySelectorAll:s=>s==='.drivePad'?driveButtons:[],
      createElement:()=>({}),addEventListener:(type,fn)=>{documentHandlers[type]=fn;}},
    Option:function(){},fetch:fakeFetch,AbortController,URLSearchParams,Date,
    setTimeout:()=>nextTimer++,clearTimeout(){},setInterval:(fn)=>{const id=nextTimer++;intervals.set(id,fn);return id;},
    clearInterval:id=>intervals.delete(id)};
  vm.runInNewContext(simulated,context);const ui=context.__testUi;ui.setState(status);
  await ui.selectDrive(forwardButton);
  assert.equal(posted[0].cmd,'drive_begin');assert.equal(posted[0].step,'0');
  assert.equal(ui.hasDriveRenew(),true,'one click starts persistent renew');
  assert.equal(forwardButton.attributes['aria-pressed'],'true','active direction is visibly pressed');
  assert.ok(forwardButton.handlers.click&&forwardButton.handlers.contextmenu);
  assert.equal(forwardButton.handlers.pointerdown,undefined,'no long-press pointerdown dependency');
  assert.equal(forwardButton.handlers.pointerup,undefined,'pointer release does not stop locked direction');
  await [...intervals.values()][0]();
  assert.equal(posted.at(-1).cmd,'drive_renew','locked direction keeps renewing');

  await ui.selectDrive(stepButton);
  assert.deepEqual(posted.slice(-2).map(x=>x.cmd),['drive_end','drive_begin'],'direction switch ends old lease then begins latest');
  assert.equal(posted.at(-1).step,'1','original stepping intent is used for explicit in-place stepping');
  assert.equal(stepButton.attributes['aria-pressed'],'true');
  assert.equal(forwardButton.attributes['aria-pressed'],'false');

  context.document.hidden=true;const beforeHidden=posted.length;documentHandlers.visibilitychange();
  assert.equal(ui.hasDriveRenew(),false,'hidden page stops local renew');
  assert.equal(posted.length,beforeHidden,'hidden page relies on firmware lease instead of a fragile release POST');
  assert.equal(drive.active,true,'server lease remains active until its 500ms timeout');

  drive.active=false;drive.epoch++;drive.seq=0;status.motion.mode='hold';context.document.hidden=false;
  ui.resetForTest(status);await ui.selectDrive(forwardButton);failRenew=true;await ui.renewDrive();
  assert.equal(ui.hasDriveRenew(),false,'renew network failure stops local renew so firmware lease can expire');
  failRenew=false;drive.active=false;drive.epoch++;drive.seq=0;ui.resetForTest(status);
  await ui.selectDrive(forwardButton);ui.priorityStop('off');
  assert.equal(ui.hasDriveRenew(),false,'priority Stop/Off cancels click-locked renewal');
  assert.ok(documentHandlers.visibilitychange,'visibility lease fallback handler installed');
}

rangeAndAngleDraftsNeverMove();
motionDraftAndTelemetryNeverMove();
bodyBalanceDraftNeverMoves();
profileDraftNeverMoves();
motionApplyAndPreviewSync().then(() => simplifiedViewsAndStandBranching()).then(() => livePoseCoalescesAndLatestWins()).then(() => driveGestureRaceSafety()).then(() => stopPreemptsPendingCalibration()).then(() => {
  process.stdout.write('V10 Web UI syntax, q/range/angle drafts, calibration, restricted HTTP and priority OFF OK\n');
}).catch(error => { process.stderr.write(String(error.stack || error) + '\n'); process.exitCode = 1; });
