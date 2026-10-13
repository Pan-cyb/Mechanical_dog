#pragma once

#include <Arduino.h>

namespace dog {

// The HTTP adapter replaces __TOKEN__ before serving this page. Only explicit
// user input may dispatch actions; polling itself must never change PWM state.
static const char kDebugWebPage[] PROGMEM = R"APPLEPY(<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>机械狗 · 板级调试</title>
<style>
:root{color-scheme:dark;--bg:#0e141b;--panel:#17212c;--soft:#202d3a;--line:#344556;--ink:#f1f5f8;--muted:#a7b7c4;--accent:#66d0ad;--danger:#ff8f87}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.45 system-ui,-apple-system,"Microsoft YaHei",sans-serif}button,input,select{font:inherit}
main{max-width:980px;margin:auto;padding:18px 16px 42px}h1{font-size:1.45rem;margin:0}h2{font-size:1.08rem;margin:0 0 12px}h3{font-size:1rem;margin:18px 0 10px}p{margin:7px 0}.muted,small{color:var(--muted)}[hidden]{display:none!important}
.top{display:flex;align-items:center;justify-content:space-between;gap:14px;margin-bottom:12px}.top p{margin:2px 0 0}.statuses{display:flex;flex-wrap:wrap;justify-content:flex-end;gap:6px}.badge{border:1px solid var(--line);border-radius:99px;padding:4px 9px;color:var(--muted);font-size:.82rem}.badge.ok{color:var(--accent);border-color:#3b7969}.badge.bad{color:var(--danger);border-color:#845052}
.safetybar{position:sticky;top:0;z-index:5;display:flex;align-items:center;justify-content:space-between;gap:12px;background:#121b24ed;border:1px solid var(--line);border-radius:12px;padding:9px 11px;margin-bottom:10px;backdrop-filter:blur(8px)}.safetybar small{flex:1}.actions{display:flex;flex-wrap:wrap;gap:8px}
button{border:1px solid #587087;border-radius:9px;background:#25384a;color:var(--ink);padding:9px 13px;min-height:41px;font-weight:650;cursor:pointer}button:hover:not(:disabled){filter:brightness(1.14)}button:disabled{cursor:not-allowed;opacity:.42}.primary{background:#1f6752;border-color:#399a7b}.stop{background:#6d373a;border-color:#a76266}.quiet{background:transparent}.nudge{min-width:92px}
.tabs{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin:10px 0}.tabs button[aria-selected=true]{background:#2d5261;border-color:var(--accent)}.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:17px;margin-bottom:12px}.control-grid{display:grid;grid-template-columns:minmax(0,1fr) minmax(320px,.9fr);gap:18px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}.fields4{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}.fields4 .slider-field{grid-column:1/-1}
label{display:block;font-weight:650;margin-bottom:6px}select,input[type=number]{width:100%;background:#101a24;color:var(--ink);border:1px solid #506477;border-radius:8px;padding:8px 9px;min-height:40px}input[type=range]{width:100%;accent-color:var(--accent);margin:15px 0 10px}input:focus-visible,select:focus-visible,button:focus-visible,summary:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
.drive{display:grid;grid-template-columns:repeat(3,74px);grid-template-areas:". up ." "left step right" ". down .";justify-content:center;gap:7px;margin:12px 0}.drivePad{touch-action:manipulation;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none;padding:8px}.drivePad[aria-pressed=true]{background:#1f6752;border-color:var(--accent);box-shadow:0 0 0 2px #66d0ad33}.drive .up{grid-area:up}.drive .down{grid-area:down}.drive .left{grid-area:left}.drive .right{grid-area:right}.drive .step{grid-area:step}.gait-row{display:grid;grid-template-columns:1fr 1fr;gap:8px;align-items:end}
.message{min-height:43px;border:1px solid var(--line);border-radius:9px;padding:9px 11px;color:var(--muted);overflow-wrap:anywhere}.message.ok,.draft-note.ok{color:var(--accent)}.message.bad,.draft-note.bad{color:var(--danger)}.warning{border:1px solid #865052;background:#3c2529;border-radius:9px;padding:9px 11px;color:#ffd7d2}.draft-note{margin:8px 0;color:var(--muted)}
details{border-top:1px solid var(--line);margin-top:14px;padding-top:10px}summary{cursor:pointer;color:#dce7ed;font-weight:700;padding:3px 0}.details-body{padding-top:12px}.kv{display:grid;grid-template-columns:auto 1fr;gap:6px 12px;margin:8px 0}.kv dt{color:var(--muted)}.kv dd{margin:0;text-align:right;font-variant-numeric:tabular-nums}.rpy{display:flex;gap:14px;flex-wrap:wrap;color:var(--muted);margin:10px 0}.rpy output{color:var(--accent);font-variant-numeric:tabular-nums}.center-list{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin:8px 0}.center-list span{border:1px solid var(--line);border-radius:7px;padding:6px;text-align:center;font-variant-numeric:tabular-nums;color:var(--muted)}.field{margin-bottom:14px}.range-row{display:grid;grid-template-columns:1fr 105px;gap:10px;align-items:center}.slider-field{min-width:0}.slider-field>label{margin-bottom:2px}.slider-field input[type=range]{margin:10px 0 8px;min-height:32px}.ticks{display:flex;justify-content:space-between;color:var(--muted);font-size:.78rem}.sr-only{position:absolute!important;width:1px;height:1px;padding:0;margin:-1px;overflow:hidden;clip:rect(0,0,0,0);white-space:nowrap;border:0}
@media(max-width:700px){main{padding:12px 10px 34px}.top{align-items:flex-start;flex-direction:column}.statuses{justify-content:flex-start}.safetybar{align-items:flex-start;flex-direction:column}.safetybar .actions{width:100%}.safetybar button{flex:1}.control-grid,.grid{grid-template-columns:1fr}.fields4{grid-template-columns:repeat(2,minmax(0,1fr))}.gait-row{grid-template-columns:1fr 1fr}.card{padding:14px}.actions button{flex:1 1 auto}.center-list{grid-template-columns:repeat(2,1fr)}.drive{grid-template-columns:repeat(3,minmax(64px,84px))}}
</style>
</head>
<body>
<main>
<header class="top"><div><h1>机械狗控制</h1><p class="muted">固件 <span id="firmwareVersion">—</span></p></div><div class="statuses" aria-label="硬件状态"><span id="link" class="badge">连接中</span><span id="pca" class="badge">PCA —</span><span id="armed" class="badge">输出 —</span><span id="imuStatus" class="badge">IMU —</span></div></header>
<div id="fault" class="warning" role="alert" hidden>状态异常：输出可能仍在持续，请立即核对并准备物理断电。</div>
<div class="safetybar"><div class="actions"><button id="mStand" class="primary">站立</button><button id="mStop" class="stop">停止</button><button id="mOff" class="stop">关闭输出</button></div><small>停止仍保持力矩；关页面不会自动停止；关闭输出不是物理断电。</small></div>
<nav class="tabs" aria-label="主视图"><button id="viewControl" data-view="control" aria-selected="true">控制</button><button id="viewCalibration" data-view="calibration" aria-selected="false">标定</button></nav>
<div id="message" class="message" role="status" aria-live="polite">正在读取状态；页面不会自动使能或运动。</div>
<div id="controlView" class="view" data-view-panel="control">
<section class="card" aria-labelledby="motionTitle"><h2 id="motionTitle">运动控制</h2><p id="motionState" class="muted">正在读取运动状态。</p>
<div class="control-grid"><div><div class="gait-row"><label>步态<select id="dMode"><option value="walk">Walk</option><option value="trot" selected>Trot</option></select></label><button id="mStart" class="primary">开始行走</button></div>
<div class="drive" aria-label="点击锁定方向控制"><button class="drivePad up" data-forward="1" data-turn="0" data-step="0" aria-pressed="false">前</button><button class="drivePad left" data-forward="0" data-turn="1" data-step="0" aria-pressed="false">左</button><button class="drivePad step" data-forward="0" data-turn="0" data-step="1" aria-pressed="false">原地踏步</button><button class="drivePad right" data-forward="0" data-turn="-1" data-step="0" aria-pressed="false">右</button><button class="drivePad down" data-forward="-1" data-turn="0" data-step="0" aria-pressed="false">后</button></div><p class="muted">方向按钮点击一次持续运动；点击另一方向安全切换，点击当前活动按钮或顶部“停止”结束并保持。页面隐藏/断网时停止续租，固件500ms租约兜底冻结。</p><div class="range-row"><label>直行修正<input id="dStraightTrim" type="number" min="-0.30" max="0.30" step="0.01" value="0.25"></label><button id="dStraightApply">应用修正（RAM）</button></div><small>内置默认0.25；+纠正左偏（减右侧），−纠正右偏（减左侧）；手动值仅RAM，重启恢复0.25。</small><p id="straightTrimState" class="draft-note">等待板端读回。</p><div class="range-row"><label>Trot Pitch补偿 °<input id="dTrotPitch" type="number" min="-0.5" max="0" step="0.1" value="0"></label><button id="dTrotPitchApply">应用Pitch补偿（RAM）</button></div><small>负值压低前端，正值不允许；仅Trot生效，建议从−0.5°开始。仅RAM，重启归零。</small><p id="trotPitchState" class="draft-note">等待板端读回。</p></div>
<div><div class="fields4"><label>步长 mm<input id="mStep" type="number" min="0" max="30" step="0.1" value="17.9"></label><label>抬脚 mm<input id="mLift" type="number" min="0" max="30" step="0.1" value="12.8"></label><label>周期 s<input id="mPeriod" type="number" min="0.4" max="10" step="0.1" value="0.5"></label><div class="slider-field"><label for="mHeightRange">站高目标 / down mm</label><div class="range-row"><input id="mHeightRange" type="range" min="100" max="115" step="0.1" value="114" aria-label="站高目标滑块"><input id="mHeight" type="number" min="100" max="115" step="0.1" value="114" aria-label="站高目标数值"></div><div class="ticks"><span>100</span><span>115 mm</span></div></div></div>
<div class="actions"><button class="mPreset" data-values="3,1,3,108,0,1">慢速</button><button class="mPreset" data-mode="walk" data-values="25.3,7.1,0.5,108,0,2">Walk参数</button><button class="mPreset" data-mode="trot" data-values="17.9,12.8,0.5,114,0,2">Trot参数</button></div>
<div class="actions" style="margin-top:8px"><button id="mApply">应用其它步态参数（RAM）</button><button id="mSave">保存已应用值到NVS</button></div><small>高度滑块实时更新；步长/抬脚/周期/高级步态项仍需点“应用其它步态参数”。保存步骤仍为关闭输出后保存当前已应用六参数，Walk/Trot共用一份记录。</small><p id="motionDraftState" class="draft-note">参数等待板端读回。</p><p id="dirtyNotice" class="draft-note">参数状态等待中。</p></div></div>
<div class="rpy" aria-label="姿态"><span>Roll <output id="roll">—</output></span><span>Pitch <output id="pitch">—</output></span><span>Yaw <output id="yaw">—</output></span></div>
<div id="motionMessage" class="sr-only" aria-hidden="true" hidden></div>
<details id="posturePanel"><summary>姿态与水平校准</summary><div class="details-body"><div class="grid"><div class="slider-field"><label for="bPitchRange">Pitch °（正值抬头）</label><div class="range-row"><input id="bPitchRange" type="range" min="-7" max="5" step="0.1" value="0" aria-label="Pitch滑块"><input id="bPitch" type="number" min="-7" max="5" step="0.1" value="0" aria-label="Pitch数值"></div><div class="ticks"><span>−7° 低头</span><span>+5° 抬头</span></div></div><div class="slider-field"><label for="bRollRange">Roll °（正值左侧高）</label><div class="range-row"><input id="bRollRange" type="range" min="-10" max="10" step="0.1" value="0" aria-label="Roll滑块"><input id="bRoll" type="number" min="-10" max="10" step="0.1" value="0" aria-label="Roll数值"></div><div class="ticks"><span>−10° 右侧高</span><span>+10° 左侧高</span></div></div><div class="slider-field"><label for="bXRange">前后重心 / body_x mm（+向前）</label><div class="range-row"><input id="bXRange" type="range" min="-8" max="8" step="0.1" value="8" aria-label="前后重心滑块"><input id="bX" type="number" min="-8" max="8" step="0.1" value="8" aria-label="前后重心数值"></div><div class="ticks"><span>−8 mm 向后</span><span>+8 mm 向前</span></div></div><div><span class="muted">自稳状态</span><p id="balanceState">关闭</p></div></div><div class="actions"><button id="bReset">放弃CG Walk/Roll符号草稿</button><button id="bCapture">水平校准</button><button id="bOn" class="primary">开启自稳</button><button id="bOff" class="stop">关闭自稳</button></div><p id="livePoseState" class="draft-note">实时姿态等待板端读回。</p><p id="bodyDraftState" class="draft-note">姿态参数等待板端读回。</p><small>拖动高度、Pitch、Roll、body_x会合并发送原子实时姿态命令（约150ms节流）：稳定Stand/Hold会平滑重定位，输出关闭时只更新RAM且绝不隐式ARM。body_x单位mm，正值表示机身目标向前。当前四元组仍须通过完整Stand预检；失败会回到板端有效值。高度用步态NVS保存，Pitch/Roll/body_x用Profile保存；CG Walk开关/余量仍需在高级区明确应用。</small></div></details>
<details id="controlAdvanced"><summary>高级设置与遥测</summary><div class="details-body">
<h3>步态高级参数</h3><div class="grid"><label>站姿前后 mm<input id="mForward" type="number" min="-5" max="5" step="0.1" value="0"></label><label>Walk轨迹偏置 mm<input id="mShift" type="number" min="0" max="3" step="0.1" value="2"></label></div><div class="actions"><button id="mReset">放弃步态修改</button><button id="mLoad">从保存值恢复</button><label>手动预检<select id="mPreviewMode"><option value="stand">Stand</option><option value="walk">Walk</option><option value="trot" selected>Trot</option><option value="leg1">腿1</option><option value="leg2">腿2</option><option value="leg3">腿3</option><option value="leg4">腿4</option></select></label><button id="mPreview">运行预检</button></div>
<div class="actions"><button class="mLeg" data-leg="1">腿1测试</button><button class="mLeg" data-leg="2">腿2测试</button><button class="mLeg" data-leg="3">腿3测试</button><button class="mLeg" data-leg="4">腿4测试</button></div><dl class="kv"><dt>已应用参数</dt><dd id="motionApplied">—</dd><dt>保存状态</dt><dd id="motionPersist">—</dd><dt>调度</dt><dd id="motionDiagnostics">—</dd></dl><small>足端和角度均为命令值，不是反馈。</small><div id="motionFeet" class="center-list" aria-label="四腿足端命令"></div><div id="motionAngles" class="center-list" aria-label="八路舵机角度命令"></div>
<h3>CG与移动补偿</h3><div class="grid"><label><input id="bCg" type="checkbox"> Walk全支撑移身</label><label>支撑余量 mm<input id="bCgShift" type="number" min="0" max="3" step="0.1" value="2.5"></label></div><div class="actions"><button id="bCgPreset">载入CG Walk参数</button><button id="bCgApply">应用重心设置</button><button id="dMovingOn">开启移动补偿</button><button id="dMovingOff" class="stop">关闭移动补偿</button></div><dl class="kv"><dt>机身参数</dt><dd id="bodyApplied">—</dd><dt>CG范围</dt><dd id="bodyCgRange">—</dd><dt>预检</dt><dd id="previewSummary">尚未预检</dd><dt>方向会话</dt><dd id="driveState">未开始</dd><dt>移动补偿</dt><dd id="movingCgState">关闭</dd><dt>周期</dt><dd id="driveTiming">—</dd></dl>
<h3>控制Profile</h3><div class="grid"><div><label>移身速度 mm/s<input id="pPlannerSlew" type="number" min="5" max="30" step="0.5" value="20"></label><label>等待超时 s<input id="pWait" type="number" min="1" max="30" step="0.5" value="15"></label><label>角度增益<input id="pAngle" type="number" min="0" max="2" step="0.05" value="1.1"></label><label>角速度增益<input id="pRate" type="number" min="0" max="10" step="0.1" value="5"></label><label>Pitch Kp / Kd<span class="range-row"><input id="pPitchKp" type="number" min="0" max="2" step="0.05" value="0.8"><input id="pPitchKd" type="number" min="0" max="1" step="0.01" value="0.08"></span></label></div><div><label>滤波τ s<input id="pTau" type="number" min="0" max="1" step="0.01" value="0.1"></label><label>最大修正 mm<input id="pMaxShift" type="number" min="0.01" max="0.5" step="0.01" value="0.5"></label><label>修正速度 mm/s<input id="pFeedbackSlew" type="number" min="0.1" max="2" step="0.1" value="2"></label><label>Roll Kp / Kd<span class="range-row"><input id="pRollKp" type="number" min="0" max="2" step="0.05" value="0.8"><input id="pRollKd" type="number" min="0" max="1" step="0.01" value="0.08"></span></label></div></div>
<div class="grid"><label><input id="pTrotFf" type="checkbox"> Trot方向前馈</label><div class="range-row"><input id="pTrotForward" type="number" min="0" max="5" step="0.1" value="4" aria-label="前进增益"><input id="pTrotBackward" type="number" min="0" max="5" step="0.1" value="4" aria-label="后退增益"></div><div class="range-row"><input id="pTrotTurn" type="number" min="0" max="5" step="0.1" value="2" aria-label="转向增益"><input id="pTrotMax" type="number" min="0" max="3" step="0.1" value="2" aria-label="最大毫米"></div><button id="pTrotApply">应用前馈</button></div>
<label>Roll传感器方向<select id="bRollSign"><option value="-1" selected>-1 body=-sensor</option><option value="0">0 关闭Roll闭环</option><option value="1">+1 body=+sensor</option></select></label><div class="actions"><button id="bSetRollSign">应用Roll方向</button><button id="pApply">应用Profile</button><button id="pReset">放弃Profile修改</button><button id="pSave">保存Profile</button><button id="pLoad">恢复Profile</button></div><p id="profileDraftState" class="draft-note">Profile等待板端读回。</p><dl class="kv"><dt>Profile状态</dt><dd id="profileState">—</dd></dl>
</div></details></section>
</div>
<div id="calibrationView" class="view" data-view-panel="calibration" hidden>
<section class="card" aria-labelledby="calTitle"><h2 id="calTitle">八路中位标定</h2><p>开始会让八路回到当前已存/内存中位，不会重置为固定90°。请先解除连杆受力并准备物理断电。</p>
<label><input id="calConfirm" type="checkbox"> 已解除八处受力并准备物理断电</label><div class="actions"><button id="calStart" class="primary">开始标定</button><button id="calStop" class="stop">退出不保存</button><button id="calSaveAll">保存并退出</button></div>
<div class="grid" style="margin-top:14px"><label>接口 / 关节<select id="channel"><option value="">— 选择接口 —</option></select></label><div><span class="muted">当前中位</span><p id="center">—</p></div></div><div class="actions"><button id="minus" class="nudge">中位 −1°</button><button id="plus" class="nudge">中位 +1°</button></div><div id="centers" class="center-list" aria-live="polite"></div>
<details id="calAdvanced"><summary>高级调试</summary><div class="details-body"><div class="field"><label>相对中位偏移 °<input id="offsetNumber" type="number" min="0" max="0" step="0.1" value="0"></label><p id="offsetBounds" class="muted">等待接口</p><p id="offsetPreview" class="muted">编辑不输出。</p><div class="actions"><button id="sendOffset">输出偏移</button><button id="centerOutput">回当前中位</button></div></div>
<div class="field"><label>小腿相对中位 q °<input id="shankNumber" type="number" min="0" max="0" step="0.1" value=""></label><p id="shankBounds" class="muted">等待接口</p><p id="shankPreview" class="muted">编辑不输出。</p><button id="sendShank">输出小腿角</button></div>
<p id="rangeState">原始电气输入范围</p><div id="rangeUnlock"><label><input id="rangeConfirm" type="checkbox"> 已确认原始电气范围风险</label><div class="actions"><button id="rangeExtended">解锁 500–2500 µs</button><button id="rangeNormal">恢复默认范围</button></div></div>
<div class="field"><label>原始脉宽 µs</label><div class="range-row"><input id="pulse" type="range" min="500" max="2500" step="1" value="1500"><input id="pulseNumber" type="number" min="500" max="2500" step="1" value="1500"></div><div class="ticks"><span id="pulseMin">500</span><span id="pulseMax">2500 µs</span></div><label>名义命令角度 °<input id="angleNumber" type="number" min="0" max="180" step="0.1" value="90"></label></div>
<div class="actions"><button id="arm">单路调试解锁</button><button id="send">输出原始脉宽</button><button id="offSelected">关闭所选通道</button><button id="save">保存所选中位</button></div>
<dl class="kv"><dt>输出通道</dt><dd id="active">—</dd><dt>运行状态</dt><dd id="remaining">—</dd><dt>姿态校准</dt><dd id="calProgress">—</dd><dt>加速度模长</dt><dd id="aNorm">—</dd><dt>Yaw零点</dt><dd id="yawRef">—</dd><dt>加速度 XYZ</dt><dd id="accel">—</dd><dt>角速度 XYZ</dt><dd id="gyro">—</dd></dl></div></details></section>
</div>
</main>
<script>
(() => {
  'use strict';
  const token = '__TOKEN__';
  const $ = id => document.getElementById(id);
  const els = Object.fromEntries(['link','pca','armed','imuStatus','fault','firmwareVersion','viewControl','viewCalibration','controlView','calibrationView','channel','offsetNumber','offsetBounds','offsetPreview','sendOffset','centerOutput','shankNumber','shankBounds','shankPreview','sendShank','pulse','pulseNumber','angleNumber','pulseMin','pulseMax','rangeState','rangeUnlock','rangeConfirm','rangeExtended','rangeNormal','arm','offSelected','send','minus','plus','save','active','remaining','center','centers','roll','pitch','yaw','yawRef','calProgress','aNorm','accel','gyro','message','calConfirm','calStart','calStop','calSaveAll','mStep','mLift','mPeriod','mHeight','mHeightRange','mForward','mShift','mApply','mReset','mSave','mLoad','mPreviewMode','mPreview','mStand','mStart','mStop','mOff','motionState','motionDraftState','dirtyNotice','motionApplied','motionPersist','motionDiagnostics','motionMessage','motionFeet','motionAngles','bPitch','bPitchRange','bRoll','bRollRange','bX','bXRange','bCg','bCgShift','bCgPreset','bCgApply','livePoseState','bodyDraftState','bodyApplied','bodyCgRange','previewSummary','balanceState','bReset','bRollSign','bSetRollSign','bCapture','bOn','bOff','pPlannerSlew','pWait','pAngle','pRate','pTau','pMaxShift','pFeedbackSlew','pPitchKp','pPitchKd','pRollKp','pRollKd','pTrotFf','pTrotForward','pTrotBackward','pTrotTurn','pTrotMax','pTrotApply','profileDraftState','profileState','pApply','pReset','pSave','pLoad','dMode','dStraightTrim','dStraightApply','straightTrimState','dTrotPitch','dTrotPitchApply','trotPitchState','dMovingOn','dMovingOff','driveState','movingCgState','driveTiming'].map(id => [id,$(id)]));
  const jointNames = {L1H:'腿1 大腿',L1S:'腿1 小腿',L2H:'腿2 大腿',L2S:'腿2 小腿',L3H:'腿3 大腿',L3S:'腿3 小腿',L4H:'腿4 大腿',L4S:'腿4 小腿'};
  let state = null, busy = false, motionDraftDirty = false, motionDraftRevision = 0;
  let bodyDraftDirty = false, bodyDraftRevision = 0;
  let rollSignDirty = false, rollSignRevision = 0;
  let profileDraftDirty = false, profileDraftRevision = 0;
  let trimDraftDirty = false, trimDraftRevision = 0;
  let trotPitchDraftDirty = false, trotPitchDraftRevision = 0;
  let livePoseTimer=null,livePoseInFlight=false,livePoseQueued=false,livePoseRevision=0;
  let livePoseController=null;
  let driveRenewTimer = null, driveEpoch = 0, driveSeq = 0;
  let driveGestureGeneration = 0, driveHeld = false, driveRenewInFlight = false, heldDriveButton = null;
  let activeController = null, pollController = null;
  let stopController = null, stopTimer = null;
  let stopInFlight = false, stopConfirmed = false, stopUntilMs = 0, requestEpoch = 0;
  function showView(name) {
    if (name !== 'control' && name !== 'calibration') return;
    const calibration = name === 'calibration';
    const changed = els.controlView.hidden !== calibration;
    els.controlView.hidden = calibration;
    els.calibrationView.hidden = !calibration;
    els.viewControl.setAttribute?.('aria-selected', String(!calibration));
    els.viewCalibration.setAttribute?.('aria-selected', String(calibration));
    if (changed && driveHeld) releaseDrive();
  }
  function feedback(message, kind) {
    els.message.textContent = message;
    els.message.className = 'message' + (kind ? ' ' + kind : '');
    els.motionMessage.textContent = message;
    els.motionMessage.className = 'message' + (kind ? ' ' + kind : '');
  }
  function badge(el, label, kind) {
    el.textContent = label;
    el.className = 'badge' + (kind ? ' ' + kind : '');
  }
  function selected() {
    const n = Number(els.channel.value);
    return els.channel.value !== '' && Number.isInteger(n) && state && Array.isArray(state.channels) && state.channels.some(x => x.channel === n) ? n : null;
  }
  function currentEntry() {
    const n = selected();
    return n === null ? null : state.channels.find(x => x.channel === n);
  }
  function pulseLimits() {
    const lo = state && state.pulse_min_us, hi = state && state.pulse_max_us;
    return Number.isInteger(lo) && Number.isInteger(hi) && lo >= 500 && hi <= 2500 && lo < hi
      ? [lo, hi] : [1000, 2000];
  }
  function relativeEntry() {
    const entry = currentEntry();
    return entry && entry.relative_available && Number.isFinite(entry.center_deg) &&
      Number.isFinite(entry.min_offset_deg) && Number.isFinite(entry.max_offset_deg) &&
      entry.min_offset_deg <= 0 && entry.max_offset_deg >= 0 ? entry : null;
  }
  function offsetPreview() {
    const entry = relativeEntry(), raw = els.offsetNumber.value.trim();
    const offset = Number(raw);
    if (!entry || !/^[+-]?\d+(?:\.\d+)?$/.test(raw) || raw.length > 8 ||
        !Number.isFinite(offset) || offset < entry.min_offset_deg || offset > entry.max_offset_deg) {
      els.offsetPreview.textContent = '偏移超出当前通道范围或尚未配置；不会发送。';
      return null;
    }
    const absolute = entry.center_deg + offset;
    els.offsetPreview.textContent = '中位 ' + entry.center_deg + '° + (' + offset + '°) = ' +
      absolute.toFixed(1) + '° · ' + Math.round(500 + absolute*2000/180) + ' µs';
    return offset;
  }
  function syncRelativeDraft() {
    const entry = relativeEntry();
    const signature = entry ? [entry.channel,entry.center_deg,entry.min_offset_deg,entry.max_offset_deg].join(':') : 'none';
    if (els.offsetNumber.dataset.signature !== signature) {
      els.offsetNumber.value = '0'; // New channel/neutral: draft only, no PWM.
      els.offsetNumber.min = String(entry ? entry.min_offset_deg : 0);
      els.offsetNumber.max = String(entry ? entry.max_offset_deg : 0);
      els.offsetNumber.dataset.signature = signature;
    }
    els.offsetBounds.textContent = entry
      ? (jointNames[entry.joint] || entry.joint) + ' · 偏移 ' + entry.min_offset_deg + '° 到 ' + entry.max_offset_deg + '°' +
        (entry.shank ? '（按本腿方向，与0–180°电气范围取交集）' : '（仅电气范围，大腿机械限位未测）')
      : '该通道未配置相对控制；请确认固件版本和接线。';
    offsetPreview();
  }
  function shankEntry() {
    const entry = currentEntry(), points = entry && entry.shank_map_points;
    return entry && entry.shank_q_available && entry.shank_q_photo_candidate &&
      (entry.shank_direction === -1 || entry.shank_direction === 1) &&
      Number.isFinite(entry.center_deg) && Number.isFinite(entry.max_shank_q_deg) &&
      entry.max_shank_q_deg >= 0 && Array.isArray(points) && points.length >= 2 &&
      points.every((p, i) => Number.isFinite(p.u_deg) && Number.isFinite(p.q_deg) &&
        p.u_deg >= 0 && p.q_deg >= 0 && (!i || (p.u_deg > points[i-1].u_deg && p.q_deg > points[i-1].q_deg)))
      ? entry : null;
  }
  function shankU(entry, q) {
    const points = entry.shank_map_points;
    for (let i = 1; i < points.length; ++i) {
      if (q <= points[i].q_deg) return points[i-1].u_deg +
        (q-points[i-1].q_deg) * (points[i].u_deg-points[i-1].u_deg) /
        (points[i].q_deg-points[i-1].q_deg);
    }
    return null;
  }
  function shankPreview() {
    const entry = shankEntry(), raw = els.shankNumber.value.trim(), q = Number(raw);
    if (!entry || !/^\d+(?:\.\d+)?$/.test(raw) || raw.length > 8 ||
        !Number.isFinite(q) || q < 0 || q > entry.max_shank_q_deg) {
      els.shankPreview.textContent = entry ? '请输入 0–' + entry.max_shank_q_deg + '°；不会发送。' :
        '该通道没有小腿 q 曲线；不会发送。';
      return null;
    }
    const u = shankU(entry, q);
    if (!Number.isFinite(u)) { els.shankPreview.textContent = '曲线数据无效；不会发送。'; return null; }
    const offset = entry.shank_direction * u, absolute = entry.center_deg + offset;
    els.shankPreview.textContent = '当前中位 ' + entry.center_deg + '° · u=' + u.toFixed(2) +
      '° → 偏移 ' + offset.toFixed(2) + '° → 命令 ' + absolute.toFixed(2) + '° · ' +
      Math.round(500 + absolute*2000/180) + ' µs';
    return q;
  }
  function syncShankDraft() {
    const entry = shankEntry();
    const signature = entry ? [entry.channel,entry.center_deg,entry.shank_direction,entry.max_shank_q_deg,
      JSON.stringify(entry.shank_map_points)].join(':') : 'none';
    if (els.shankNumber.dataset.signature !== signature) {
      els.shankNumber.value = ''; // Channel/neutral change clears draft; no PWM.
      els.shankNumber.min = '0';
      els.shankNumber.max = String(entry ? entry.max_shank_q_deg : 0);
      els.shankNumber.dataset.signature = signature;
    }
    els.shankBounds.textContent = entry
      ? (jointNames[entry.joint] || entry.joint) + ' · ' + (entry.shank_direction < 0 ? '中位减 u' : '中位加 u') +
        ' · 当前可用 q：0° 到 ' + entry.max_shank_q_deg + '°（随本路中位／电气边界变化）'
      : '该通道没有小腿 q 曲线；请选择腿1–4的小腿通道。';
    shankPreview();
  }
  function setDraftPulse(us) {
    els.pulse.value = String(us);
    els.pulseNumber.value = String(us);
    els.angleNumber.value = ((us - 500) * 180 / 2000).toFixed(1);
  }
  function syncRange() {
    const [lo, hi] = pulseLimits();
    const signature = lo + ':' + hi;
    if (els.pulse.dataset.range !== signature) {
      els.pulse.min = els.pulseNumber.min = String(lo);
      els.pulse.max = els.pulseNumber.max = String(hi);
      els.angleNumber.min = String((lo - 500) * 180 / 2000);
      els.angleNumber.max = String((hi - 500) * 180 / 2000);
      // Reset the draft on a range change, NEVER dispatch a PWM here.
      const entry = currentEntry();
      const center = entry && entry.center_us;
      setDraftPulse(Number.isInteger(center) && center >= lo && center <= hi ? center : 1500);
      els.pulse.dataset.range = signature;
    }
    els.pulseMin.textContent = String(lo);
    els.pulseMax.textContent = hi + ' µs';
    els.rangeState.textContent = '当前范围：' + lo + '–' + hi + ' µs' +
      (state && state.extended_range ? '（已解锁，不是机械限位）' : '（校准输入，不是机械限位）');
    // Current board is full-range by default. Keep controls only for older /
    // custom constrained configurations; no unnecessary unlock in 0.5.2.
    els.rangeUnlock.hidden = lo === 500 && hi === 2500 && !state.extended_range;
  }
  function controls() {
    const previewChecking=!!(state&&state.motion&&state.motion.preview&&state.motion.preview.pending);
    const blocked = busy || livePoseInFlight || livePoseQueued ||
      Date.now() < stopUntilMs || previewChecking;
    const ready = !!state && !!state.pca && !!state.imu_ok && !state.fault;
    const chosen = selected() !== null;
    const armed = ready && !!state.armed;
    const calibrating = ready && !!state.calibrating;
    const motion = state && state.motion;
    const motionRunning = !!motion && (previewChecking || !!motion.pending || !!motion.transition ||
      ['walk','trot','leg_test_1','leg_test_2','leg_test_3','leg_test_4'].includes(motion.mode));
    const stableMotionHold = !!motion && ['stand','hold'].includes(motion.mode);
    const balance=motion&&motion.balance;
    const balanceBusy=!!balance&&(balance.capture_pending||balance.enabled||balance.enable_pending);
    els.sendOffset.disabled = blocked || motionRunning || stableMotionHold || !armed || calibrating || !relativeEntry();
    els.centerOutput.disabled = els.sendOffset.disabled;
    els.offsetNumber.disabled = blocked || motionRunning || stableMotionHold || calibrating || !relativeEntry();
    els.sendShank.disabled = blocked || motionRunning || stableMotionHold || !armed || calibrating || !shankEntry();
    els.shankNumber.disabled = blocked || motionRunning || stableMotionHold || calibrating || !shankEntry();
    els.rangeExtended.disabled = blocked || motionRunning || stableMotionHold || !ready || armed || !!state.extended_range || !els.rangeConfirm.checked;
    els.rangeNormal.disabled = blocked || motionRunning || stableMotionHold || !ready || armed || !state.extended_range;
    els.arm.disabled = blocked || !ready || armed;
    els.offSelected.disabled = blocked || motionRunning || stableMotionHold || !ready || calibrating || !chosen || state.active_channel !== selected();
    els.send.disabled = blocked || motionRunning || stableMotionHold || !armed || calibrating || !chosen;
    els.minus.disabled = blocked || motionRunning || stableMotionHold || !armed || !chosen;
    els.plus.disabled = blocked || motionRunning || stableMotionHold || !armed || !chosen;
    els.save.disabled = blocked || motionRunning || stableMotionHold || !state || !!state.fault || !chosen || state.armed;
    els.calStart.disabled = blocked || motionRunning || stableMotionHold || !ready || armed || !els.calConfirm.checked;
    els.calStop.disabled = false;
    els.calSaveAll.disabled = blocked || !state || !state.pca || state.fault ||
      !state.calibration_pending ||
      (state.armed && !state.calibrating);
    els.mApply.disabled = blocked || motionRunning || livePoseInFlight || livePoseQueued || balanceBusy || !ready || calibrating || state.active_channel >= 0;
    [els.mStep,els.mLift,els.mPeriod,els.mHeight,els.mHeightRange,els.mForward,els.mShift].forEach(input=>{input.disabled=blocked||motionRunning;});
    document.querySelectorAll('.mPreset').forEach(button=>{button.disabled=blocked||motionRunning;});
    els.mReset.disabled = blocked || !state;
    els.mPreview.disabled = blocked || motionRunning || balanceBusy || profileDraftDirty || trimDraftDirty || trotPitchDraftDirty || !ready || calibrating || state.active_channel >= 0;
    els.mPreviewMode.disabled = blocked || motionRunning;
    els.mSave.disabled = blocked || !state || state.armed || state.fault || motionRunning || stableMotionHold || motionDraftDirty || livePoseInFlight || livePoseQueued;
    els.mLoad.disabled = els.mSave.disabled;
    const draftBlocked = motionDraftDirty || bodyDraftDirty || profileDraftDirty || trimDraftDirty || trotPitchDraftDirty;
    const commonStartBlocked = blocked || livePoseInFlight || livePoseQueued || !ready || calibrating || state.active_channel >= 0 || draftBlocked;
    els.mStand.disabled = commonStartBlocked || (!armed && (motionRunning || stableMotionHold));
    const startBlocked = commonStartBlocked || !armed;
    els.mStart.disabled = startBlocked;
    document.querySelectorAll('.mLeg').forEach(button => { button.disabled = startBlocked; });
    els.mStop.disabled = false;
    els.mOff.disabled = false;
    const drive=motion&&motion.drive;
    const movingBusy=!!drive&&(drive.moving_cg_enabled||drive.moving_cg_pending);
    const stableLiveMode=!!motion&&((!armed&&motion.mode==='idle')||
      (armed&&(motion.mode==='stand'||motion.mode==='hold')));
    const livePoseDisabled=busy||!state||!ready||calibrating||state.active_channel>=0||state.fault||
      previewChecking||balanceBusy||movingBusy||!stableLiveMode||Date.now()<stopUntilMs;
    [els.mHeight,els.mHeightRange,els.bPitch,els.bPitchRange,els.bRoll,els.bRollRange,els.bX,els.bXRange]
      .forEach(input=>{input.disabled=livePoseDisabled;});
    els.dStraightTrim.disabled=blocked||motionRunning||balanceBusy||movingBusy;
    els.dStraightApply.disabled=blocked||motionRunning||balanceBusy||movingBusy||!ready;
    els.dTrotPitch.disabled=blocked||motionRunning||balanceBusy||movingBusy;
    els.dTrotPitchApply.disabled=blocked||motionRunning||balanceBusy||movingBusy||!ready;
    [els.bCg,els.bCgShift].forEach(input=>{input.disabled=blocked||motionRunning||balanceBusy;});
    els.bCgPreset.disabled=blocked||motionRunning||balanceBusy;
    els.bCgApply.disabled=blocked||motionRunning||livePoseInFlight||livePoseQueued||balanceBusy||!ready||calibrating||state.active_channel>=0;
    els.bReset.disabled=blocked||!state;
    els.bRollSign.disabled=blocked||balanceBusy||movingBusy;
    els.bSetRollSign.disabled=blocked||balanceBusy||movingBusy||!state||!motion||!((!state.armed&&motion.mode==='idle')||motion.mode==='hold');
    els.bCapture.disabled=blocked||balanceBusy||movingBusy||!ready||!motion||motion.mode!=='stand'||motion.pending||motion.transition;
    els.bOn.disabled=blocked||balanceBusy||movingBusy||!ready||!balance||!balance.reference_ready||
      !motion||motion.mode!=='stand'||motion.pending||motion.transition||motionDraftDirty||bodyDraftDirty||profileDraftDirty||rollSignDirty;
    els.bOff.disabled=false;
    [els.pPlannerSlew,els.pWait,els.pAngle,els.pRate,els.pTau,els.pMaxShift,
      els.pFeedbackSlew,els.pPitchKp,els.pPitchKd,els.pRollKp,els.pRollKd].forEach(input=>{input.disabled=blocked||motionRunning||balanceBusy||movingBusy;});
    [els.pTrotFf,els.pTrotForward,els.pTrotBackward,els.pTrotTurn,els.pTrotMax].forEach(input=>{input.disabled=blocked||motionRunning||balanceBusy||movingBusy;});
    els.pApply.disabled=blocked||motionRunning||balanceBusy||movingBusy||!ready;
    els.pTrotApply.disabled=els.pApply.disabled;
    els.pReset.disabled=blocked||!state;
    els.pSave.disabled=blocked||livePoseInFlight||livePoseQueued||!state||state.armed||state.fault||motionRunning||stableMotionHold||
      bodyDraftDirty||profileDraftDirty;
    els.pLoad.disabled=els.pSave.disabled;
    const localDriveActive=!!drive&&drive.active&&driveHeld&&!!heldDriveButton;
    const driveBlocked=!localDriveActive&&(blocked||!ready||!armed||calibrating||motionDraftDirty||bodyDraftDirty||
      profileDraftDirty||trimDraftDirty||trotPitchDraftDirty||livePoseInFlight||livePoseQueued||!drive||drive.active);
    document.querySelectorAll('.drivePad').forEach(button=>{
      button.disabled=driveBlocked;
      button.setAttribute?.('aria-pressed',String(localDriveActive&&button===heldDriveButton));
    });
    els.dMode.disabled=blocked||!!(drive&&drive.active);
    els.dMovingOn.disabled=blocked||profileDraftDirty||!ready||!balance||!balance.reference_ready||movingBusy||
      motion.mode!=='stand'||motion.pending||motion.transition;
    els.dMovingOff.disabled=false;
    const hiddenDirtySections=[];
    if(bodyDraftDirty)hiddenDirtySections.push('姿态/CG');
    if(profileDraftDirty)hiddenDirtySections.push('高级参数');
    if(trimDraftDirty)hiddenDirtySections.push('直行修正');
    if(trotPitchDraftDirty)hiddenDirtySections.push('Trot Pitch补偿');
    els.dirtyNotice.hidden=!hiddenDirtySections.length;
    els.dirtyNotice.textContent=hiddenDirtySections.length?
      '未应用：'+hiddenDirtySections.join('、')+'。启动已锁定，请展开对应区域处理。':'';
    els.dirtyNotice.className='draft-note bad';
  }

  function syncMotionDraft(s) {
    const p = s && s.motion && s.motion.parameters;
    if (!Array.isArray(p) || p.length !== 6 || !p.every(Number.isFinite)) return;
    if(!motionDraftDirty)
      [els.mStep.value,els.mLift.value,els.mPeriod.value,els.mForward.value,els.mShift.value]=
        [p[0],p[1],p[2],p[4],p[5]].map(String);
    if(!motionDraftDirty&&!livePoseQueued&&!livePoseInFlight){
      els.mHeight.value=String(p[3]);els.mHeightRange.value=els.mHeight.value;
    }
  }
  function syncTrimDraft(s) {
    const d=s&&s.motion&&s.motion.drive;
    if(trimDraftDirty||!d||!Number.isFinite(Number(d.straight_trim)))return;
    els.dStraightTrim.value=String(Number(d.straight_trim));
  }
  function syncTrotPitchDraft(s) {
    const d=s&&s.motion&&s.motion.drive;
    if(trotPitchDraftDirty||!d||!Number.isFinite(Number(d.trot_pitch_offset_deg)))return;
    els.dTrotPitch.value=String(Number(d.trot_pitch_offset_deg));
  }
  function syncBodyDraft(s) {
    const b=s&&s.motion&&s.motion.body;
    if(!b)return;
    if(!livePoseQueued&&!livePoseInFlight){
      els.bPitch.value=String(b.pitch_deg);els.bRoll.value=String(b.roll_deg);
      els.bPitchRange.value=els.bPitch.value;els.bRollRange.value=els.bRoll.value;
      els.bX.value=String(b.x_mm);els.bXRange.value=els.bX.value;
    }
    if(!bodyDraftDirty){
      els.bCg.checked=!!b.cg_enabled;
      els.bCgShift.value=String(b.cg_shift_mm);
    }
  }
  function syncProfileDraft(s) {
    const p=s&&s.motion&&s.motion.profile;
    if(profileDraftDirty||!p)return;
    els.pPlannerSlew.value=String(p.planner_slew_mm_s);
    els.pWait.value=String(Number(p.wait_timeout_ms)/1000);
    els.pAngle.value=String(p.moving_angle_gain);els.pRate.value=String(p.moving_rate_gain);
    els.pTau.value=String(p.moving_tau_s);els.pMaxShift.value=String(p.moving_max_shift_mm);
    els.pFeedbackSlew.value=String(p.moving_slew_mm_s);
    els.pPitchKp.value=String(p.pitch_kp);els.pPitchKd.value=String(p.pitch_kd_s);
    els.pRollKp.value=String(p.roll_kp);els.pRollKd.value=String(p.roll_kd_s);
    els.pTrotFf.checked=!!p.trot_ff_enabled;els.pTrotForward.value=String(p.trot_ff_forward);
    els.pTrotBackward.value=String(p.trot_ff_backward);els.pTrotTurn.value=String(p.trot_ff_turn);
    els.pTrotMax.value=String(p.trot_ff_max_mm);
  }
  function renderAdvanced(s) {
    const m=s&&s.motion,b=m&&m.body,balance=m&&m.balance,preview=m&&m.preview;
    if(!m||!b||!balance||!preview)return;
    els.bodyApplied.textContent='Pitch '+Number(b.pitch_deg).toFixed(1)+'° / Roll '+
      Number(b.roll_deg).toFixed(1)+'° / X '+Number(b.x_mm).toFixed(1)+'mm / CG '+
      (b.cg_enabled?'开，余量 '+Number(b.cg_shift_mm).toFixed(1)+'mm':'关')+' / rev '+Number(b.revision||0);
    els.bodyCgRange.textContent=Number(b.cg_actual_x_min_mm).toFixed(1)+' 到 '+
      Number(b.cg_actual_x_max_mm).toFixed(1)+' mm；髋轴 '+Number(b.length_mm).toFixed(1)+'×'+
      Number(b.width_mm).toFixed(1)+'mm（'+(b.dimensions_measured?'已实测':'原项目名义，未实测')+'）';
    const errorText={not_run:'尚未预检',checking:'正在检查',cancelled:'检查已取消',imu_stale:'IMU样本过期',
      output_unhealthy:'输出状态失效',state_changed:'配置已变化',lease_expired:'方向租约过期',
      reference_invalid:'姿态参考失效',initial_frame_invalid:'初始帧无效',
      none:'通过',invalid_parameters:'参数无效',body_transform:'机身变换失败',
      ik:'IK不可达',hip_window:'大腿窗口越界',shank_map:'小腿映射越界',servo_range:'通道角度越界',
      support_margin:'名义体中心超出支撑三角形'}[preview.error]||'未知错误';
    const jointNames8=['L1H 大腿','L1S 小腿','L2H 大腿','L2S 小腿','L3H 大腿','L3S 小腿','L4H 大腿','L4S 小腿'];
    const legText=Number(preview.leg)>=0?'腿'+(Number(preview.leg)+1):'—';
    const jointText=Number(preview.joint)>=0?(jointNames8[Number(preview.joint)]||'索引'+preview.joint):'不适用';
    const marginText=b.cg_enabled?Number(preview.min_support_margin_mm).toFixed(2)+'mm':'不适用';
    els.previewSummary.textContent=(preview.pending?'正在检查步态… '+preview.stage+' · '+Number(preview.frames_checked||0)+'帧':
      errorText+'（'+preview.error+' / '+(preview.result||'—')+'）')+' · '+legText+' · '+jointText+
      ' · phase '+Number(preview.phase).toFixed(3)+' · 名义支撑边距 '+marginText;
    const rollMode=Number(balance.roll_sensor_sign)===0?'Roll主动关闭':
      Number(balance.roll_sensor_sign)===-1?'Roll已确认 body=-sensor（-1）':'Roll手动覆盖 body=+sensor（+1）';
    els.balanceState.textContent=(balance.enabled?'运行中':balance.enable_pending?'待新样本开启':
      balance.capture_pending?'抓取 '+balance.capture_samples+'/'+balance.capture_required:
      balance.reference_ready?'参考已就绪':'关闭/无参考')+' · '+rollMode+' · 修正 P '+
      Number(balance.pitch_correction_deg).toFixed(2)+'° R '+Number(balance.roll_correction_deg).toFixed(2)+
      '° · '+balance.error;
    if(!rollSignDirty)els.bRollSign.value=String(balance.roll_sensor_sign);
    els.bodyDraftState.textContent=bodyDraftDirty?
      '未应用CG Walk草稿：运动与自稳启动已锁定；请应用或放弃。':
      'CG Walk草稿与板端RAM同步；Pitch/Roll/body_x由上方实时姿态链独立更新。';
    els.bodyDraftState.className='message '+(bodyDraftDirty?'bad':'ok');
    const liveParameters=Array.isArray(m.parameters)?m.parameters:[];
    els.livePoseState.textContent=livePoseInFlight?'实时姿态正在等待板端原子确认…':
      livePoseQueued?'实时姿态已合并，等待150ms节流发送…':
      liveParameters.length===6?'板端有效值：down '+Number(liveParameters[3]).toFixed(1)+'mm / Pitch '+
        Number(b.pitch_deg).toFixed(1)+'° / Roll '+Number(b.roll_deg).toFixed(1)+'° / X '+
        Number(b.x_mm).toFixed(1)+'mm。':'实时姿态等待板端读回。';
    els.livePoseState.className='draft-note '+((livePoseInFlight||livePoseQueued)?'':'ok');
    const profile=m.profile;
    if(profile){
      els.profileState.textContent=(profile.saved?'已保存NVS':'仅RAM/默认')+' · '+profile.state+
        ' · rev '+Number(profile.revision||0);
      els.profileDraftState.textContent=profileDraftDirty?
        '未应用控制Profile草稿：方向启动已锁定。':'控制Profile草稿与板端RAM同步；不包含enable/reference/方向意图。';
      els.profileDraftState.className='message '+(profileDraftDirty?'bad':'ok');
    }
    const drive=m.drive;
    if(drive){
      els.driveState.textContent=(drive.active?'ACTIVE':'inactive')+' · epoch '+drive.epoch+
        ' seq '+drive.seq+' · lease '+drive.lease_remaining_ms+'ms · '+drive.reason+
        ' · intent '+Number(drive.forward).toFixed(1)+'/'+Number(drive.turn).toFixed(1)+
        (drive.stepping?' step':'')+' · trim '+Number(drive.straight_trim).toFixed(2)+
        ' · effective turn '+Number(drive.effective_turn).toFixed(2);
      els.straightTrimState.textContent=trimDraftDirty?
        '直行修正尚未应用：启动已锁定。':'板端RAM '+Number(drive.straight_trim).toFixed(2)+
        '；当前有效turn '+Number(drive.effective_turn).toFixed(2)+'；重启恢复默认0.25。';
      els.straightTrimState.className='draft-note '+(trimDraftDirty?'bad':'ok');
      els.trotPitchState.textContent=trotPitchDraftDirty?
        'Trot Pitch补偿尚未应用：启动与预检已锁定。':'板端RAM '+
        Number(drive.trot_pitch_offset_deg).toFixed(1)+'°；Trot有效Pitch '+
        Number(drive.trot_effective_pitch_deg).toFixed(1)+'°；rev '+
        Number(drive.trot_pitch_revision||0)+'；仅Trot，重启归零。';
      els.trotPitchState.className='draft-note '+(trotPitchDraftDirty?'bad':'ok');
      els.movingCgState.textContent=(drive.moving_cg_enabled?'运行':drive.moving_cg_pending?'待新样本':'关闭')+
        ' · shift '+Number(drive.moving_cg_shift_mm).toFixed(3)+'mm · '+drive.moving_cg_error+
        (drive.phase_waiting?' · 等待移身到位(stage '+drive.cg_stage+')':'');
      els.driveTiming.textContent='名义 '+Number(drive.nominal_elapsed_s).toFixed(2)+'s / 实际 '+
        Number(drive.actual_elapsed_s).toFixed(2)+'s；预检 '+Number(m.preview&&m.preview.duration_ms||0)+
        'ms，预测实际圈 '+Number(m.preview&&m.preview.actual_cycle_s||0).toFixed(2)+'s';
    }
  }
  function renderMotion(s) {
    if (!s.motion) return;
    const m=s.motion;
    const modeNames={idle:'已关闭',stand:'站立',hold:'停止保持',hold_error:'错误保持',walk:'Walk',trot:'Trot',leg_test_1:'腿1测试',leg_test_2:'腿2测试',leg_test_3:'腿3测试',leg_test_4:'腿4测试'};
    const mainPreviewErrors={imu_stale:'IMU样本过期',output_unhealthy:'输出状态失效',
      state_changed:'参数已变化',lease_expired:'方向租约过期',reference_invalid:'姿态参考失效',
      initial_frame_invalid:'初始帧无效',invalid_parameters:'参数无效',
      body_transform:'机身变换失败',ik:'IK不可达',hip_window:'大腿窗口越界',
      shank_map:'小腿映射越界',servo_range:'通道角度越界',
      support_margin:'支撑余量不足',drive_intent:'方向意图无效'};
    const previewTerminal=m.preview&&m.preview.result&&/^cancelled/.test(m.preview.result)?' · 检查已取消':
      m.preview&&m.preview.result&&/^rejected/.test(m.preview.result)?
      ' · 检查未通过：'+(mainPreviewErrors[m.preview.error]||'请查看高级详情'):'';
    els.motionState.textContent=m.preview&&m.preview.pending?'正在检查步态… 原运动状态继续':
      ((m.error&&m.error!=='none'?'异常：'+m.error+' · ':'')+'当前 '+(modeNames[m.mode]||m.mode)+
      (m.transition?'（过渡中）':m.pending?'（等待新IMU启动）':previewTerminal));
    const feet=Array.isArray(m.feet)?m.feet:[];
    els.motionFeet.replaceChildren(...feet.slice(0,4).map((v,i)=>{const item=document.createElement('span');item.textContent='腿'+(i+1)+' F '+Number(v[0]).toFixed(1)+' / D '+Number(v[1]).toFixed(1)+' mm';return item;}));
    const names=['L1H','L1S','L2H','L2S','L3H','L3S','L4H','L4S'];
    const angles=Array.isArray(m.command_deg)?m.command_deg:[];
    els.motionAngles.replaceChildren(...angles.slice(0,8).map((v,i)=>{const item=document.createElement('span');item.textContent=names[i]+' '+Number(v).toFixed(1)+'°';return item;}));
    const p=Array.isArray(m.parameters)?m.parameters:[];
    els.motionApplied.textContent=p.length===6 ? p.map((v,i)=>Number(v).toFixed(i===2?2:1)).join(' / ') : '—';
    els.motionPersist.textContent=(m.parameters_saved?'已保存 NVS':'仅 RAM／默认')+' · '+(m.parameter_state||'未知')+' · rev '+Number(m.parameter_revision||0);
    const d=m.diagnostics||{};
    els.motionDiagnostics.textContent='帧 '+Number(d.frame_count||0)+' · gap '+Number(d.last_gap_ms||0)+'/'+Number(d.max_gap_ms||0)+' ms · 漏期 '+Number(d.missed_deadlines||0);
    els.motionDraftState.textContent=motionDraftDirty ?
      '参数尚未应用：行走已锁定。' : '步态参数已应用；预设不会自动保存或运动。';
    els.motionDraftState.className='draft-note '+(motionDraftDirty?'bad':'ok');
  }
  function formatVector(values) {
    return Array.isArray(values) && values.length === 3 && values.every(Number.isFinite)
      ? values.map(v => v.toFixed(2)).join(' / ') : '—';
  }
  function renderAttitude(s) {
    const a = s && s.attitude_ok && s.attitude;
    els.roll.textContent = a && Number.isFinite(a.roll_deg) ? a.roll_deg.toFixed(1) + '°' : '—';
    els.pitch.textContent = a && Number.isFinite(a.pitch_deg) ? a.pitch_deg.toFixed(1) + '°' : '—';
    els.yaw.textContent = a && Number.isFinite(a.yaw_deg) ? a.yaw_deg.toFixed(1) + '°' : '—';
    els.yawRef.textContent = a && Number.isFinite(a.yaw_reference_ms)
      ? '上电后 ' + (a.yaw_reference_ms / 1000).toFixed(1) + ' s' : '—';
  }
  function render(s) {
    state = s;
    if(els.message.textContent==='正在读取状态；页面不会自动使能或运动。')
      feedback('先点站立，再选择步态开始。','ok');
    els.firmwareVersion.textContent = s.firmware_version || '未提供（可能是旧固件）';
    badge(els.link, '已连接', 'ok');
    if (s.fault) badge(els.pca, 'PCA9685/输出状态未确认', 'bad');
    else badge(els.pca, 'PCA9685 ' + (s.pca ? '正常' : '未就绪'), s.pca ? 'ok' : 'bad');
    if (s.fault) badge(els.armed, '故障：输出可能仍在持续', 'bad');
    else if (s.calibrating) badge(els.armed, '8 路中位标定输出', 'bad');
    else if (s.motion && s.motion.pending) badge(els.armed, '运动待启动', 'bad');
    else if (s.motion && s.motion.transition) badge(els.armed, '八路过渡输出', 'bad');
    else if (s.motion && (s.motion.mode === 'walk' || s.motion.mode === 'trot' ||
      /^leg_test_[1-4]$/.test(s.motion.mode))) badge(els.armed, '八路运动输出', 'bad');
    else if (s.motion && ['stand','hold','hold_error'].includes(s.motion.mode))
      badge(els.armed, s.motion.mode === 'hold_error' ?
        '八路姿态保持（错误：'+s.motion.error+'）' : '八路姿态保持', 'bad');
    else badge(els.armed, s.armed ?
      (s.active_channel >= 0 ? 'PWM 正在输出' : '已使能，未发脉冲') : '输出已关闭',
      s.armed ? 'bad' : 'ok');
    badge(els.imuStatus, 'IMU ' + (s.imu_ok ? '正常' : '未就绪'), s.imu_ok ? 'ok' : 'bad');
    els.fault.hidden = !s.fault;
    const entries = Array.isArray(s.channels) ? s.channels : [];
    const old = els.channel.value;
    const signature = entries.map(x => x.channel + ':' + x.servo + ':' + x.joint).join(',');
    if (els.channel.dataset.signature !== signature) {
      els.channel.replaceChildren(new Option('— 先选择接口 —', ''));
      entries.forEach(x => els.channel.add(new Option('Servo' + x.servo + ' · PCA ch' + x.channel + ' · ' + (jointNames[x.joint] || '未配置'), String(x.channel))));
      els.channel.dataset.signature = signature;
      els.channel.value = entries.some(x => String(x.channel) === old) ? old : '';
    }
    const entry = currentEntry();
    els.center.textContent = entry && Number.isFinite(entry.center_us) ? entry.center_deg + '° · ' + entry.center_us + ' µs' : '—';
    els.centers.replaceChildren(...entries.map(x => {
      const item = document.createElement('span');
      item.textContent = 'Servo' + x.servo + ' · ' + x.center_deg + '°';
      return item;
    }));
    const active = entries.find(x => x.channel === s.active_channel);
    const coordinated=s.motion && (s.motion.pending || s.motion.transition ||
      ['stand','hold','hold_error','walk','trot','leg_test_1','leg_test_2','leg_test_3','leg_test_4'].includes(s.motion.mode));
    els.active.textContent = s.fault ? '未知（可能仍在输出）' : s.calibrating ?
      '8 路中位标定（mask ' + Number(s.calibration_mask).toString(16) + '）' :
      (coordinated ? (s.motion.pending?'八路请求待执行（原输出状态保持）':'八路协调输出') :
      (s.active_channel >= 0 ? (active ? 'Servo' + active.servo : 'PCA ch' + s.active_channel) : '无'));
    els.remaining.textContent = s.fault ? '故障：输出状态未知' :
      (s.manual_hold ? '手动保持，直到关闭' :
      (Number.isFinite(s.remaining_ms) ? Math.max(0, s.remaining_ms) + ' ms' : '—'));
    els.calProgress.textContent = s.imu_ok && Number.isInteger(s.calibration_samples)
      ? (s.attitude_ok ? '完成' : (s.gyro_calibrated ? '零偏完成，姿态待稳定' :
         s.calibration_samples + '/' + s.calibration_required)) : '—';
    els.aNorm.textContent = typeof s.accel_norm_mps2 === 'number' && Number.isFinite(s.accel_norm_mps2)
      ? s.accel_norm_mps2.toFixed(2) + ' m/s²' : '—';
    renderAttitude(s);
    syncMotionDraft(s);
    syncTrimDraft(s);
    syncTrotPitchDraft(s);
    syncBodyDraft(s);
    syncProfileDraft(s);
    renderMotion(s);
    renderAdvanced(s);
    els.accel.textContent = s.imu ? formatVector(s.imu.accel) : '—';
    els.gyro.textContent = s.imu ? formatVector(s.imu.gyro) : '—';
    syncRange();
    syncRelativeDraft();
    syncShankDraft();
    controls();
  }
  async function poll() {
    const controller = new AbortController();
    pollController = controller;
    try {
      const response = await fetch('/api/state', {cache:'no-store', signal:controller.signal});
      if (!response.ok) throw new Error('状态请求失败（HTTP ' + response.status + '）');
      const next = await response.json();
      if (!controller.signal.aborted) render(next);
    } catch (e) {
      if (controller.signal.aborted && Date.now() < stopUntilMs) return;
      state = null;
      badge(els.link, '连接中断', 'bad');
      badge(els.pca, 'PCA9685 状态未知', 'bad');
      badge(els.armed, '输出状态未知', 'bad');
      badge(els.imuStatus, 'IMU 状态未知', 'bad');
      els.fault.hidden = false;
      els.active.textContent = '—';
      els.remaining.textContent = '—';
      els.calProgress.textContent = '—';
      els.aNorm.textContent = '—';
      els.center.textContent = '—';
      els.centers.replaceChildren();
      els.accel.textContent = '—';
      els.gyro.textContent = '—';
      renderAttitude(null);
      controls();
      feedback('无法读取板子状态：' + e.message + '。请检查连接和舵机电源。', 'bad');
    } finally {
      if (pollController === controller) pollController = null;
      setTimeout(poll, 1000);
    }
  }
  async function sendStopAttempt(cmd, epoch) {
    if (stopInFlight) return;
    stopInFlight = true;
    const controller = new AbortController();
    stopController = controller;
    const timeout = setTimeout(() => controller.abort(), 700);
    try {
      const response = await fetch('/api/action', {
        method:'POST',
        headers:{'Content-Type':'application/x-www-form-urlencoded','X-Applepy-Token':token},
        body:new URLSearchParams({cmd}).toString(), signal:controller.signal
      });
      const result = await response.json();
      if (epoch !== requestEpoch) return;
      if (response.ok && result.ok) {
        stopConfirmed = true;
        feedback(cmd==='moving_cg_off' ?
          '固件已停止移动CG反馈和方向会话并保持最后帧；舵机仍有力。' : cmd==='balance_off' ?
          '固件已关闭静态自稳并保持最后帧；舵机仍有力，需要失力请点全关。' : cmd==='motion_stop' ?
          '固件已停止周期推进并保持最后帧；舵机仍有力，需要失力请点全关。' :
          '固件已接受软件全关；仍请核对真实输出，必要时物理断电。', 'ok');
      } else feedback(result.message || (cmd==='moving_cg_off' ?
        '停止移动CG/方向并保持未确认；租约仍会兜底失效。' : cmd==='balance_off' ?
        '关闭自稳并保持未确认；输出状态未知，必要时物理断电。' : cmd==='motion_stop' ?
        '停止并保持未确认；输出状态未知，必要时物理断电。' :
        '软件全关被拒绝，请物理断电。'), 'bad');
    } catch (e) {
      if (epoch === requestEpoch && !stopConfirmed)
        feedback(cmd==='moving_cg_off' ?
          '停止移动CG/方向尚未确认；等待租约兜底，必要时物理断电。' : cmd==='balance_off' ?
          '关闭自稳并保持尚未确认；输出状态未知，必要时物理断电。' : cmd==='motion_stop' ?
          '停止并保持尚未确认；输出状态未知，必要时物理断电。' :
          '软件全关尚未确认；正在重试，立即准备物理断电。', 'bad');
    } finally {
      clearTimeout(timeout);
      if (stopController === controller) { stopController = null; stopInFlight = false; }
    }
  }
  function priorityStop(cmd) {
    // Abort the old browser wait, but never assume that abort cancels a command
    // already received by the MCU. Repeat OFF for a short bounded window.
    const epoch = ++requestEpoch;
    cancelDriveGestureLocal();
    cancelLivePoseLocal();
    if (activeController) activeController.abort();
    if (pollController) pollController.abort();
    if (stopController) { stopController.abort(); stopController = null; stopInFlight = false; }
    if (stopTimer) clearInterval(stopTimer);
    stopUntilMs = Date.now() + 3000;
    stopConfirmed = false;
    state = null; // Do not enable new writes from status sampled before OFF.
    badge(els.link, '操作后状态待确认', 'bad');
    badge(els.armed, '输出状态未知', 'bad');
    feedback(cmd==='moving_cg_off' ?
      '正在优先停止移动CG和方向会话并保持；成功后舵机仍有力。' : cmd==='balance_off' ?
      '正在优先关闭静态自稳并保持；成功后舵机仍有力。' : cmd==='motion_stop' ?
      '正在优先请求停止并保持；成功后舵机仍有力。若状态未知，必要时物理断电。' :
      '正在优先发送软件全关；若无确认，请立即物理断电。', 'bad');
    controls();
    sendStopAttempt(cmd, epoch);
    const timer = setInterval(() => {
      if (epoch !== requestEpoch || Date.now() >= stopUntilMs) {
        clearInterval(timer);
        if (stopTimer === timer) stopTimer = null;
        if (epoch === requestEpoch && !stopConfirmed)
          feedback(cmd==='moving_cg_off' ?
            '停止移动CG/方向未得到确认；等待租约失效，输出状态未知。' : cmd==='balance_off' ?
            '关闭自稳并保持未得到确认；输出状态未知，必要时物理断电。' : cmd==='motion_stop' ?
            '停止并保持未得到确认；输出状态未知，必要时物理切断舵机电源。' :
            '软件全关未得到确认；请物理切断舵机电源。', 'bad');
        controls();
      } else sendStopAttempt(cmd, epoch);
    }, 400);
    stopTimer = timer;
  }
  async function action(cmd, fields = {}) {
    if (cmd === 'off' || cmd === 'motion_stop' || cmd === 'balance_off' || cmd === 'moving_cg_off') { priorityStop(cmd); return; }
    if (busy || livePoseInFlight || livePoseQueued || Date.now() < stopUntilMs || !state) return;
    const channel = selected();
    if (!['cal_start','cal_save_all','range_extended','range_normal','arm','arm_stand','stand','walk','trot','leg_test','motion_set','motion_preview','motion_save','motion_load','body_set','balance_capture','balance_on','balance_off','balance_roll_sign','control_set','trot_ff_set','straight_trim_set','trot_pitch_set','profile_save','profile_load','moving_cg_on','moving_cg_off','drive_begin','drive_renew','drive_end'].includes(cmd) && channel === null) {
      feedback('请先选择接口。', 'bad'); return;
    }
    const epoch = requestEpoch;
    const submittedDraftRevision = cmd === 'motion_set' ? motionDraftRevision : -1;
    const submittedBodyRevision = cmd === 'body_set' ? bodyDraftRevision : -1;
    const submittedRollRevision = cmd === 'balance_roll_sign' ? rollSignRevision : -1;
    const submittedProfileRevision = (cmd === 'control_set'||cmd==='trot_ff_set') ? profileDraftRevision : -1;
    const submittedTrimRevision = cmd === 'straight_trim_set' ? trimDraftRevision : -1;
    const submittedTrotPitchRevision = cmd === 'trot_pitch_set' ? trotPitchDraftRevision : -1;
    const controller = new AbortController();
    activeController = controller;
    busy = true; controls();
    try {
      const form = new URLSearchParams({cmd, ...fields});
      if (channel !== null) form.set('channel', String(channel));
      const response = await fetch('/api/action', {
        method:'POST',
        headers:{'Content-Type':'application/x-www-form-urlencoded','X-Applepy-Token':token},
        body:form.toString(), signal:controller.signal
      });
      const result = await response.json();
      if (epoch !== requestEpoch) return; // A later OFF supersedes this reply.
      feedback(result.message || (result.ok ? '操作成功。' : '操作失败。'), response.ok && result.ok ? 'ok' : 'bad');
      if (cmd === 'cal_start' && response.ok && result.ok) els.calConfirm.checked = false;
      if (cmd === 'range_extended' && response.ok && result.ok) els.rangeConfirm.checked = false;
      const status = await fetch('/api/state', {cache:'no-store', signal:controller.signal});
      if (!status.ok) throw new Error('状态未更新');
      const next = await status.json();
      if (epoch === requestEpoch) {
        if (cmd === 'motion_set' && response.ok && result.ok) {
          const expected=[fields.step,fields.lift,fields.period,fields.height,fields.forward,fields.shift].map(Number);
          const actual=next.motion && next.motion.parameters;
          const verified=Array.isArray(actual) && actual.length===6 &&
            actual.every((v,i)=>Number.isFinite(v) && Math.abs(v-expected[i])<0.011);
          if (verified && submittedDraftRevision===motionDraftRevision) motionDraftDirty=false;
          else if (submittedDraftRevision!==motionDraftRevision)
            feedback('应用期间草稿已改变：旧请求虽成功，但保留新草稿并继续禁止启动。','bad');
          else feedback('固件回复成功，但状态回读与草稿不一致；保留草稿并禁止启动。','bad');
        }
        if (cmd === 'motion_load' && response.ok && result.ok) motionDraftDirty=false;
        if (cmd === 'body_set' && response.ok && result.ok) {
          const expected=[fields.pitch,fields.roll,fields.x,fields.cg,fields.cg_shift].map(Number);
          const actual=next.motion&&next.motion.body;
          const values=actual?[actual.pitch_deg,actual.roll_deg,actual.x_mm,actual.cg_enabled?1:0,actual.cg_shift_mm]:[];
          const verified=values.length===5&&values.every((v,i)=>Number.isFinite(v)&&Math.abs(v-expected[i])<0.011);
          if(verified&&submittedBodyRevision===bodyDraftRevision)bodyDraftDirty=false;
          else if(submittedBodyRevision!==bodyDraftRevision)
            feedback('应用期间机身/CG草稿已改变：保留新草稿并继续禁止启动。','bad');
          else feedback('板端机身/CG读回不一致；保留草稿并禁止启动。','bad');
        }
        if(cmd==='balance_roll_sign'&&response.ok&&result.ok) {
          const actual=next.motion&&next.motion.balance&&Number(next.motion.balance.roll_sensor_sign);
          if(actual===Number(fields.sign)&&submittedRollRevision===rollSignRevision)rollSignDirty=false;
          else if(submittedRollRevision!==rollSignRevision)
            feedback('应用期间Roll符号草稿已改变：保留新选择并禁止开启自稳。','bad');
          else feedback('板端Roll符号读回不一致；保留选择并禁止开启自稳。','bad');
        }
        if((cmd==='control_set'||cmd==='trot_ff_set')&&response.ok&&result.ok){
          const p=next.motion&&next.motion.profile;
          const expected=[fields.planner_slew,Number(fields.wait_s)*1000,fields.angle_gain,
            fields.rate_gain,fields.tau,fields.max_shift,fields.feedback_slew,fields.pitch_kp,fields.pitch_kd,
            fields.roll_kp,fields.roll_kd,fields.ff_enable,fields.ff_forward,fields.ff_backward,
            fields.ff_turn,fields.ff_max].map(Number);
          const actual=p?[p.planner_slew_mm_s,p.wait_timeout_ms,p.moving_angle_gain,p.moving_rate_gain,
            p.moving_tau_s,p.moving_max_shift_mm,p.moving_slew_mm_s,p.pitch_kp,p.pitch_kd_s,
             p.roll_kp,p.roll_kd_s,p.trot_ff_enabled?1:0,p.trot_ff_forward,p.trot_ff_backward,
             p.trot_ff_turn,p.trot_ff_max_mm]:[];
          const verified=actual.length===16&&actual.every((v,i)=>Number.isFinite(v)&&Math.abs(v-expected[i])<0.011);
          if(verified&&submittedProfileRevision===profileDraftRevision)profileDraftDirty=false;
          else if(submittedProfileRevision!==profileDraftRevision)
            feedback('应用期间控制Profile草稿已改变：保留新草稿并禁止方向启动。','bad');
          else feedback('板端控制Profile读回不一致；保留草稿。','bad');
        }
        if(cmd==='straight_trim_set'&&response.ok&&result.ok){
          const actual=next.motion&&next.motion.drive&&Number(next.motion.drive.straight_trim);
          const expected=Number(fields.value);
          if(Number.isFinite(actual)&&Math.abs(actual-expected)<0.001&&
             submittedTrimRevision===trimDraftRevision)trimDraftDirty=false;
          else if(submittedTrimRevision!==trimDraftRevision)
            feedback('应用期间直行修正草稿已改变：保留新草稿并继续禁止启动。','bad');
          else feedback('板端直行修正读回不一致；保留草稿并禁止启动。','bad');
        }
        if(cmd==='trot_pitch_set'&&response.ok&&result.ok){
          const drive=next.motion&&next.motion.drive;
          const actual=drive&&Number(drive.trot_pitch_offset_deg);
          const expected=Number(fields.value);
          if(Number.isFinite(actual)&&Math.abs(actual-expected)<0.001&&
             submittedTrotPitchRevision===trotPitchDraftRevision)trotPitchDraftDirty=false;
          else if(submittedTrotPitchRevision!==trotPitchDraftRevision)
            feedback('应用期间Trot Pitch草稿已改变：保留新草稿并继续禁止启动。','bad');
          else feedback('板端Trot Pitch读回不一致；保留草稿并禁止启动。','bad');
        }
        if(cmd==='profile_load'&&response.ok&&result.ok){bodyDraftDirty=false;profileDraftDirty=false;}
        render(next);
        return {ok:response.ok&&result.ok,state:next,result};
      }
    } catch (e) {
      if (epoch === requestEpoch)
        feedback('请求失败：' + e.message + '。请检查输出并切断舵机电源。', 'bad');
      return null;
    } finally {
      if (activeController === controller) activeController = null;
      busy = false;
      controls();
    }
  }
  function livePoseFields(){return{height:els.mHeight.value,pitch:els.bPitch.value,roll:els.bRoll.value,x:els.bX.value};}
  function setLivePoseInputs(height,pitch,roll,x){
    els.mHeight.value=els.mHeightRange.value=String(height);
    els.bPitch.value=els.bPitchRange.value=String(pitch);
    els.bRoll.value=els.bRollRange.value=String(roll);
    els.bX.value=els.bXRange.value=String(x);
  }
  function cancelLivePoseLocal(){
    ++livePoseRevision;livePoseQueued=false;
    if(livePoseTimer){clearTimeout(livePoseTimer);livePoseTimer=null;}
    if(livePoseController){livePoseController.abort();livePoseController=null;}
    livePoseInFlight=false;
  }
  function queueLivePose(){
    ++livePoseRevision;
    const values=[els.mHeight.value,els.bPitch.value,els.bRoll.value,els.bX.value].map(Number);
    const valid=values.every(Number.isFinite)&&values[0]>=100&&values[0]<=115&&
      values[1]>=-7&&values[1]<=5&&values[2]>=-10&&values[2]<=10&&
      values[3]>=-8&&values[3]<=8;
    if(livePoseTimer){clearTimeout(livePoseTimer);livePoseTimer=null;}
    if(!valid){livePoseQueued=false;els.livePoseState.textContent='实时姿态输入越过硬范围；未发送。';
      els.livePoseState.className='draft-note bad';controls();return;}
    livePoseQueued=true;els.livePoseState.textContent='实时姿态已合并，等待150ms节流发送…';
    els.livePoseState.className='draft-note';controls();
    livePoseTimer=setTimeout(flushLivePose,150);
  }
  async function flushLivePose(){
    livePoseTimer=null;
    if(livePoseInFlight||!livePoseQueued||!state||Date.now()<stopUntilMs)return;
    if(busy){livePoseTimer=setTimeout(flushLivePose,50);return;}
    const revision=livePoseRevision,epoch=requestEpoch,fields=livePoseFields();
    livePoseQueued=false;livePoseInFlight=true;controls();
    const controller=new AbortController();livePoseController=controller;
    try{
      const response=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Applepy-Token':token},
        body:new URLSearchParams({cmd:'pose_live_set',...fields}).toString(),signal:controller.signal});
      const result=await response.json();
      if(epoch!==requestEpoch)return;
      const status=await fetch('/api/state',{cache:'no-store',signal:controller.signal});
      if(!status.ok)throw new Error('状态未更新');
      const next=await status.json(),p=next.motion&&next.motion.parameters,b=next.motion&&next.motion.body;
      const actual=Array.isArray(p)&&p.length===6&&b?[Number(p[3]),Number(b.pitch_deg),Number(b.roll_deg),Number(b.x_mm)]:[];
      const expected=[fields.height,fields.pitch,fields.roll,fields.x].map(Number);
      const verified=response.ok&&result.ok&&actual.length===4&&actual.every((v,i)=>Number.isFinite(v)&&Math.abs(v-expected[i])<0.011);
      state=next;
      if(revision===livePoseRevision&&!livePoseQueued){
        if(verified)setLivePoseInputs(actual[0],actual[1],actual[2],actual[3]);
        else if(actual.length===4)setLivePoseInputs(actual[0],actual[1],actual[2],actual[3]);
      }
      render(next);
      if(revision===livePoseRevision&&!livePoseQueued)
        feedback(verified?(result.message||'实时姿态已确认。'):
          (result.message||'实时姿态被拒绝；已恢复板端有效值。'),verified?'ok':'bad');
    }catch(e){
      if(epoch===requestEpoch&&revision===livePoseRevision&&!livePoseQueued)
        feedback('实时姿态请求失败：'+e.message+'；等待状态轮询确认，必要时停止或物理断电。','bad');
    }finally{
      if(livePoseController===controller)livePoseController=null;
      livePoseInFlight=false;controls();
      if(livePoseQueued&&!livePoseTimer)livePoseTimer=setTimeout(flushLivePose,0);
    }
  }
  els.viewControl.addEventListener('click',()=>showView('control'));
  els.viewCalibration.addEventListener('click',()=>showView('calibration'));
  els.channel.addEventListener('change', () => {
    const entry = currentEntry();
    const [lo, hi] = pulseLimits();
    if (entry && Number.isInteger(entry.center_us) && entry.center_us >= lo && entry.center_us <= hi)
      setDraftPulse(entry.center_us);
    else setDraftPulse(1500);
    syncRelativeDraft();
    syncShankDraft();
    controls();
  });
  els.offsetNumber.addEventListener('input', offsetPreview);
  els.sendOffset.addEventListener('click', () => {
    const offset = offsetPreview();
    if (offset === null) { feedback('偏移无效／越界；未发送。', 'bad'); return; }
    action('offset', {deg:String(offset)});
  });
  els.centerOutput.addEventListener('click', () => {
    els.offsetNumber.value = '0'; offsetPreview();
    action('offset', {deg:'0'});
  });
  els.shankNumber.addEventListener('input', shankPreview);
  els.sendShank.addEventListener('click', () => {
    const q = shankPreview();
    if (q === null) { feedback('小腿 q 无效／越界；未发送。', 'bad'); return; }
    action('shank', {q_deg:String(q)});
  });
  els.pulse.addEventListener('input', () => { setDraftPulse(Number(els.pulse.value)); });
  els.pulseNumber.addEventListener('input', () => {
    const n = Number(els.pulseNumber.value);
    const [lo, hi] = pulseLimits();
    if (els.pulseNumber.value.trim() && Number.isInteger(n) && n >= lo && n <= hi) setDraftPulse(n);
    else els.angleNumber.value = '';
  });
  els.angleNumber.addEventListener('input', () => {
    const raw = els.angleNumber.value.trim(), degrees = Number(raw);
    const us = Math.round(500 + degrees * 2000 / 180);
    const [lo, hi] = pulseLimits();
    if (raw && Number.isFinite(degrees) && degrees >= (lo-500)*180/2000 &&
        degrees <= (hi-500)*180/2000 && us >= lo && us <= hi) {
      els.pulse.value = els.pulseNumber.value = String(us);
    } else els.pulseNumber.value = ''; // Invalid angle must not send the previous draft.
  });
  els.rangeConfirm.addEventListener('change', controls);
  els.rangeExtended.addEventListener('click', () => {
    if (!els.rangeConfirm.checked) { feedback('先确认无负载、小步测试和物理断电手段。', 'bad'); return; }
    action('range_extended', {confirm:'EXTENDED_RANGE_RISK'});
  });
  els.rangeNormal.addEventListener('click', () => action('range_normal'));
  els.arm.addEventListener('click', () => action('arm'));
  els.offSelected.addEventListener('click', () => action('off_channel'));
  els.calConfirm.addEventListener('change', controls);
  els.calStart.addEventListener('click', () => {
    if (!els.calConfirm.checked) { feedback('先确认受力连接已松开并准备物理断电。', 'bad'); return; }
    action('cal_start', {confirm:'LOOSE_LINKAGES'});
  });
  els.calStop.addEventListener('click', () => { action('off'); showView('control'); });
  els.calSaveAll.addEventListener('click', async () => {
    const outcome=await action('cal_save_all');
    if(outcome&&outcome.ok)showView('control');
  });
  els.send.addEventListener('click', () => {
    const raw = els.pulseNumber.value.trim(), us = Number(raw);
    const [lo, hi] = pulseLimits();
    if (!raw || !Number.isInteger(us) || us < lo || us > hi) {
      feedback('脉宽必须是 ' + lo + '–' + hi + ' µs 之间的整数。', 'bad'); return;
    }
    action('pulse', {us:String(us)});
  });
  els.minus.addEventListener('click', () => action('nudge', {delta:'-1'}));
  els.plus.addEventListener('click', () => action('nudge', {delta:'+1'}));
  els.save.addEventListener('click', () => action('save'));
  const markMotionDraft=()=>{motionDraftDirty=true;++motionDraftRevision;renderMotion(state);controls();};
  const markBodyDraft=()=>{bodyDraftDirty=true;++bodyDraftRevision;renderAdvanced(state);controls();};
  function bindDraftPair(numberInput,rangeInput,markDirty){
    numberInput.addEventListener('input',()=>{if(numberInput.value!==''&&Number.isFinite(Number(numberInput.value)))rangeInput.value=numberInput.value;markDirty();});
    rangeInput.addEventListener('input',()=>{numberInput.value=rangeInput.value;markDirty();});
  }
  const motionFields=()=>({step:els.mStep.value,lift:els.mLift.value,period:els.mPeriod.value,height:els.mHeight.value,forward:els.mForward.value,shift:els.mShift.value});
  [els.mStep,els.mLift,els.mPeriod,els.mForward,els.mShift].forEach(input=>input.addEventListener('input',markMotionDraft));
  bindDraftPair(els.mHeight,els.mHeightRange,queueLivePose);
  document.querySelectorAll('.mPreset').forEach(button=>button.addEventListener('click',()=>{
    const values=button.dataset.values.split(',');
    [els.mStep.value,els.mLift.value,els.mPeriod.value,els.mHeight.value,els.mForward.value,els.mShift.value]=values;
    els.mHeightRange.value=els.mHeight.value;
    if(button.dataset.mode)els.dMode.value=button.dataset.mode;
    motionDraftDirty=true; ++motionDraftRevision; controls(); renderMotion(state);
    feedback('预设已写入草稿；尚未应用、保存或运动。','');
  }));
  els.mApply.addEventListener('click',()=>action('motion_set',motionFields()));
  els.mReset.addEventListener('click',()=>{motionDraftDirty=false;syncMotionDraft(state);renderMotion(state);controls();feedback('已放弃草稿并同步板上已应用值。','ok');});
  els.mSave.addEventListener('click',()=>action('motion_save'));
  els.mLoad.addEventListener('click',()=>action('motion_load'));
  els.mPreview.addEventListener('click',()=>action('motion_preview',{mode:els.mPreviewMode.value}));
  els.mStand.addEventListener('click',()=>state?action(state.armed?'stand':'arm_stand'):null);
  els.mStart.addEventListener('click',()=>state?action(els.dMode.value==='walk'?'walk':'trot'):null);
  els.dStraightTrim.addEventListener('input',()=>{trimDraftDirty=true;++trimDraftRevision;renderAdvanced(state);controls();});
  els.dStraightApply.addEventListener('click',()=>action('straight_trim_set',{value:els.dStraightTrim.value}));
  els.dTrotPitch.addEventListener('input',()=>{trotPitchDraftDirty=true;++trotPitchDraftRevision;renderAdvanced(state);controls();});
  els.dTrotPitchApply.addEventListener('click',()=>action('trot_pitch_set',{value:els.dTrotPitch.value}));
  document.querySelectorAll('.mLeg').forEach(b=>b.addEventListener('click',()=>action('leg_test',{leg:b.dataset.leg})));
  els.mStop.addEventListener('click',()=>action('motion_stop'));
  els.mOff.addEventListener('click',()=>action('off'));
  const bodyFields=()=>({pitch:els.bPitch.value,roll:els.bRoll.value,x:els.bX.value,
    cg:els.bCg.checked?'1':'0',cg_shift:els.bCgShift.value});
  [els.bCg,els.bCgShift].forEach(input=>input.addEventListener('input',markBodyDraft));
  bindDraftPair(els.bPitch,els.bPitchRange,queueLivePose);
  bindDraftPair(els.bRoll,els.bRollRange,queueLivePose);
  bindDraftPair(els.bX,els.bXRange,queueLivePose);
  els.bCgPreset.addEventListener('click',()=>{
    [els.mStep.value,els.mLift.value,els.mPeriod.value,els.mHeight.value,els.mForward.value,els.mShift.value]=['6','2','3.2','108','0','2'];
    els.mHeightRange.value=els.mHeight.value;
    els.bCg.checked=true;els.bCgShift.value='2.5';
    motionDraftDirty=true;bodyDraftDirty=true;++motionDraftRevision;++bodyDraftRevision;
    renderMotion(state);renderAdvanced(state);controls();
    feedback('CG Walk 3.2s已写入两组草稿；尚未应用、预检或运动。','');
  });
  els.bCgApply.addEventListener('click',()=>action('body_set',bodyFields()));
  els.bReset.addEventListener('click',()=>{bodyDraftDirty=false;rollSignDirty=false;syncBodyDraft(state);renderAdvanced(state);controls();feedback('已放弃CG Walk/Roll符号草稿并同步板端RAM。','ok');});
  els.bRollSign.addEventListener('change',()=>{rollSignDirty=true;++rollSignRevision;controls();});
  els.bSetRollSign.addEventListener('click',()=>action('balance_roll_sign',{sign:els.bRollSign.value}));
  els.bCapture.addEventListener('click',()=>action('balance_capture'));
  els.bOn.addEventListener('click',()=>action('balance_on'));
  els.bOff.addEventListener('click',()=>action('balance_off'));
  const profileFields=()=>({planner_slew:els.pPlannerSlew.value,wait_s:els.pWait.value,
    angle_gain:els.pAngle.value,rate_gain:els.pRate.value,tau:els.pTau.value,
    max_shift:els.pMaxShift.value,feedback_slew:els.pFeedbackSlew.value,
    pitch_kp:els.pPitchKp.value,pitch_kd:els.pPitchKd.value,
    roll_kp:els.pRollKp.value,roll_kd:els.pRollKd.value,
    ff_enable:els.pTrotFf.checked?'1':'0',ff_forward:els.pTrotForward.value,
    ff_backward:els.pTrotBackward.value,ff_turn:els.pTrotTurn.value,ff_max:els.pTrotMax.value});
  [els.pPlannerSlew,els.pWait,els.pAngle,els.pRate,els.pTau,els.pMaxShift,
    els.pFeedbackSlew,els.pPitchKp,els.pPitchKd,els.pRollKp,els.pRollKd].forEach(input=>input.addEventListener('input',()=>{
      profileDraftDirty=true;++profileDraftRevision;renderAdvanced(state);controls();}));
  [els.pTrotFf,els.pTrotForward,els.pTrotBackward,els.pTrotTurn,els.pTrotMax].forEach(input=>input.addEventListener('input',()=>{
      profileDraftDirty=true;++profileDraftRevision;renderAdvanced(state);controls();}));
  els.pApply.addEventListener('click',()=>action('control_set',profileFields()));
  els.pTrotApply.addEventListener('click',()=>{const p=profileFields();action('trot_ff_set',{
    enable:p.ff_enable,forward_gain:p.ff_forward,backward_gain:p.ff_backward,turn_gain:p.ff_turn,max_abs:p.ff_max,
    ...p});});
  els.pReset.addEventListener('click',()=>{profileDraftDirty=false;syncProfileDraft(state);renderAdvanced(state);controls();feedback('已放弃控制Profile草稿。','ok');});
  els.pSave.addEventListener('click',()=>action('profile_save'));
  els.pLoad.addEventListener('click',()=>action('profile_load'));
  els.dMovingOn.addEventListener('click',()=>action('moving_cg_on'));
  els.dMovingOff.addEventListener('click',()=>action('moving_cg_off'));

  function stopRenewTimer(){if(driveRenewTimer){clearInterval(driveRenewTimer);driveRenewTimer=null;}}
  function cancelDriveGestureLocal(){
    ++driveGestureGeneration;driveHeld=false;driveRenewInFlight=false;heldDriveButton=null;
    stopRenewTimer();driveEpoch=0;
  }
  async function renewDrive(){
    if(!driveEpoch||!driveHeld||driveRenewInFlight)return;
    const generation=driveGestureGeneration,epoch=driveEpoch,seq=++driveSeq;
    driveRenewInFlight=true;
    try{
      const response=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Applepy-Token':token},
        body:new URLSearchParams({cmd:'drive_renew',epoch:String(epoch),seq:String(seq)}).toString()});
      const result=await response.json();
      if(generation!==driveGestureGeneration||epoch!==driveEpoch)return;
      if(!response.ok||!result.ok){cancelDriveGestureLocal();feedback(result.message||'方向续租失败；固件将按租约保持最后帧。','bad');}
    }catch(e){if(generation===driveGestureGeneration&&epoch===driveEpoch){cancelDriveGestureLocal();
      feedback('方向续租网络失败；固件将在租约到期后保持最后帧。','bad');}}
    finally{if(generation===driveGestureGeneration)driveRenewInFlight=false;}
  }
  async function beginDrive(button){
    if(button.disabled||driveHeld||!state||!state.motion||!state.motion.drive)return;
    const generation=++driveGestureGeneration;driveHeld=true;heldDriveButton=button;driveEpoch=0;driveSeq=1;
    controls();
    const expectedEpoch=Number(state.motion.drive.epoch);
    const outcome=await action('drive_begin',{epoch:String(expectedEpoch),seq:'1',mode:els.dMode.value,
      forward:button.dataset.forward,turn:button.dataset.turn,step:button.dataset.step});
    if(generation!==driveGestureGeneration||!driveHeld)return;
    let acceptedEpoch=(expectedEpoch+1)>>>0;if(acceptedEpoch===0)acceptedEpoch=1;
    if(outcome&&outcome.ok&&state&&state.motion&&state.motion.drive&&state.motion.drive.active&&
       Number(state.motion.drive.epoch)===acceptedEpoch&&Number(state.motion.drive.seq)===1){
      driveEpoch=Number(state.motion.drive.epoch);driveSeq=Number(state.motion.drive.seq)||1;
      driveRenewTimer=setInterval(renewDrive,150);
    }else cancelDriveGestureLocal();
    controls();
  }
  async function releaseDrive(){
    if(!driveHeld&&!driveRenewTimer&&!driveEpoch)return true;
    const epoch=driveEpoch,seq=driveSeq+1;
    cancelDriveGestureLocal();controls();
    if(!epoch){priorityStop('motion_stop');return false;}
    try{
      const response=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Applepy-Token':token},
        body:new URLSearchParams({cmd:'drive_end',epoch:String(epoch),seq:String(seq)}).toString()});
      const result=await response.json();
      if(!response.ok||!result.ok){feedback(result.message||'方向释放被拒绝；固件租约将在500ms内冻结最后帧。','bad');return false;}
      const status=await fetch('/api/state',{cache:'no-store'});
      if(!status.ok)throw new Error('状态未更新');
      state=await status.json();render(state);
      feedback(result.message||'方向已停止并保持最后帧。','ok');return true;
    }catch(e){feedback('方向释放未确认；固件租约将在500ms内冻结最后帧。','bad');return false;}
  }
  let driveSelectionGeneration=0;
  async function selectDrive(button){
    if(button.disabled)return;
    const selection=++driveSelectionGeneration;
    if(driveHeld&&heldDriveButton===button){await releaseDrive();return;}
    if(driveHeld||driveEpoch||driveRenewTimer){
      const stopped=await releaseDrive();
      if(!stopped||selection!==driveSelectionGeneration)return;
    }
    if(selection===driveSelectionGeneration)await beginDrive(button);
  }
  document.querySelectorAll('.drivePad').forEach(button=>{
    button.addEventListener('click',event=>{event.preventDefault();selectDrive(button);});
    button.addEventListener('contextmenu',event=>event.preventDefault());
  });
  if(document.addEventListener)document.addEventListener('visibilitychange',()=>{if(document.hidden&&driveHeld){
    cancelDriveGestureLocal();controls();feedback('页面已隐藏：停止方向续租，固件将在500ms租约到期后冻结最后帧。','bad');}});
  showView('control');
  controls(); poll();
})();
</script>
</body>
</html>)APPLEPY";

}  // namespace dog
