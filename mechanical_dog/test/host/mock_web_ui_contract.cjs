const assert = require('node:assert/strict');
const {action,state,motion} = require('./mock_web_ui_server.cjs');

const call = fields => action(new URLSearchParams(fields));
const setMotion = values => call({cmd:'motion_set',step:String(values[0]),lift:String(values[1]),
  period:String(values[2]),height:String(values[3]),forward:String(values[4]),shift:String(values[5])});

assert.equal(motion.drive.straight_trim,0.25);
assert.equal(setMotion([25.3,7.1,0.5,108,0,2])[0],200);
let [status,result] = call({cmd:'motion_preview',mode:'walk'});
assert.equal(status,200);
assert.equal(result.ok,true);
assert.equal(motion.preview.error,'none');

assert.equal(setMotion([17.9,12.8,0.5,114,0,2])[0],200);
[status,result] = call({cmd:'motion_preview',mode:'trot'});
assert.equal(status,200);
assert.equal(result.ok,true);
assert.equal(motion.preview.error,'none');

assert.equal(call({cmd:'straight_trim_set',value:'0.05'})[0],200);
assert.equal(motion.drive.straight_trim,0.05);
assert.equal(call({cmd:'straight_trim_set',value:'0.31'})[0],400);
assert.equal(call({cmd:'trot_pitch_set',value:'-0.5'})[0],200);
assert.equal(motion.drive.trot_pitch_offset_deg,-0.5);
assert.equal(motion.drive.trot_effective_pitch_deg,-0.5);
assert.equal(motion.drive.trot_pitch_revision,1);
assert.equal(call({cmd:'trot_pitch_set',value:'-0.6'})[0],400);
assert.equal(call({cmd:'trot_pitch_set',value:'0.1'})[0],400);
assert.equal(call({cmd:'trot_pitch_set',value:'-0.25'})[0],400);

assert.equal(setMotion([6,2,1.2,108,0,2])[0],200);
[status,result] = call({cmd:'motion_preview',mode:'walk'});
assert.equal(status,200);
assert.equal(result.ok,true);
assert.equal(motion.preview.error,'none');
state.armed=true;
assert.equal(call({cmd:'walk'})[0],200);
call({cmd:'motion_stop'});

assert.equal(setMotion([6,2,1.0,108,0,2])[0],200);
[status,result] = call({cmd:'motion_preview',mode:'walk'});
assert.equal(status,200);
assert.equal(result.ok,true);
assert.equal(motion.preview.error,'none');
assert.equal(call({cmd:'walk'})[0],200,'former speed-only rejection now starts');
call({cmd:'motion_stop'});

assert.equal(setMotion([20,6,0.7,115,0,2])[0],200);
[status,result] = call({cmd:'motion_preview',mode:'trot'});
assert.equal(status,200);
assert.equal(result.ok,true);
assert.equal(motion.preview.error,'none');

assert.equal(setMotion([7,2,2,108,0,2])[0],200);
[status,result] = call({cmd:'motion_preview',mode:'walk'});
assert.equal(status,409);
assert.equal(motion.preview.error,'mock_unsupported');
assert.match(result.message,/模拟器未覆盖.*真实固件预检/);

process.stdout.write('Mock Web UI fixed preflight fixtures OK\n');

