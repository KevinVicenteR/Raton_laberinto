const assert=require('node:assert/strict');const p=require('../tools/telemetry_protocol.js');
assert.deepEqual(p.parse('1.20\tOBS raw=-1,50,60 age=100,0,20 phase=GIRO drops=4').raw,[-1,50,60]);
assert.equal(p.parse('MOTION completed=0 reason=2 actual=14').completed,0);
assert.equal(p.parse('RISK per_mille=-1 enabled=0').per_mille,-1);
assert.equal(p.parse('SWEEP observed=1 feasible=0 entry=-5').entry,-5);
assert.equal(p.parse('garbage'),null);assert.equal(p.parse('OBS raw=1,NaN,3'),null);
const frame={t:0,filtered:{L:-1,F:20,R:50},truth:[1,2,0]};assert.equal(p.validateReplay({trace:[frame]}).trace.length,1);
assert.throws(()=>p.validateReplay({trace:[]}));assert.throws(()=>p.validateReplay({trace:[{...frame,t:1},frame]}));assert.throws(()=>p.validateReplay({trace:[{...frame,truth:[NaN,0,0]}]}));console.log('Protocol and replay validation passed');

assert.equal(p.parse('OBS age=NaN'),null);assert.equal(p.parse('OBS raw=1,2'),null);assert.throws(()=>p.validateReplay({trace:[frame],maze:['+---+','|   |','+---+'],goal:[4,2]}));

assert.throws(()=>p.validateReplay({trace:[frame],clean_arrival:true,reached:null,events:[]}));
assert.throws(()=>p.validateReplay({trace:[frame],clean_arrival:true,reached:1,events:[[.5,'choque',1,2]]}));
// Redes neuronales
assert.deepEqual(p.parse('CTRL dyaw=-12 vscale=85'),{kind:'CTRL',dyaw:-12,vscale:85});
assert.equal(p.parse('LOGIC f=120 r=-30 b=-200 l=10 pick=f').pick,'f');
assert.equal(p.parse('NET control=1 control_trained=1 logic=0 logic_trained=0').logic,0);
assert.equal(p.parse('CTRL dyaw=abc vscale=85'),null);
