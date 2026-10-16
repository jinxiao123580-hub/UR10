"""Hardware-free metric mapping and safety-boundary tests."""
import numpy as np
import itertools
import ast
import inspect
from unittest.mock import patch, Mock
import write_ballpoint as w

# Check shared helper signatures without touching hardware. This catches
# required-argument changes even though geometry tests do not execute motion.
tree = ast.parse(inspect.getsource(w))
phase_calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='writing_state']
assert len(phase_calls)==1
phase=phase_calls[0].args[-1]
assert isinstance(phase,ast.UnaryOp) and isinstance(phase.op,ast.Not) and isinstance(phase.operand,ast.Name) and phase.operand.id=='on_paper'
for node in ast.walk(tree):
    if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and isinstance(node.func.value,ast.Name) and node.func.value.id == 'probe':
        helper = getattr(w.probe,node.func.attr)
        inspect.signature(helper).bind(*([None]*len(node.args)),**{kw.arg:None for kw in node.keywords})
script = w.probe.finite_movel_program('pen_write_step',np.zeros(6),.001,.002)
assert w.motion_parameters(False)==(.010,.020,.002)
assert w.motion_parameters(True)==(.003,.005,.001)
assert w.motion_parameters(True,True)==(.030,.100,.002)
assert w.motion_parameters(False,True)[0]==.030
assert w.motion_parameters(False)[2]<.0025
reader=Mock()
with patch.object(w.probe,'stationary_state',side_effect=[TimeoutError('silent'),('q','tcp','v')]), patch.object(w.probe,'stop') as stopped:
    assert w.read_stationary(reader,'test')==('q','tcp','v')
    assert reader.connect.call_count==1 and stopped.call_count==1
reader=Mock()
with patch.object(w.probe,'stationary_state',side_effect=TimeoutError('silent')) as reads, patch.object(w.probe,'stop'):
    try:
        w.read_stationary(reader,'test')
    except TimeoutError:
        pass
    else:
        raise AssertionError('persistent silent stream must fail closed')
    assert reads.call_count==3 and reader.connect.call_count==2
reader=Mock()
with patch.object(w.probe,'stationary_state',side_effect=TimeoutError('silent')), patch.object(w.probe,'stop'):
    try:
        w.read_stationary(reader,'test',False)
    except TimeoutError:
        pass
    else:
        raise AssertionError('contact phase must not retry a lost stationary state')
    reader.connect.assert_not_called()
assert script.endswith('end\n') and script.count('pen_write_step()') == 1
class FakeForce:
    def __init__(self,delta):
        self.delta=delta
    def recent(self):
        return [(0,0,0,[0,0,self.delta,0,0,0])]*10
assert w.probe.force_level(FakeForce(1.2),np.zeros(6))==1.2
try:
    w.probe.force_level(FakeForce(5.1),np.zeros(6))
except w.probe.ForceGuardError:
    pass
else:
    raise AssertionError('5 N overload must remain active')
assert '空走或抬笔时力变化达到 1 N' not in inspect.getsource(w.execute)
with patch.object(w.probe,'motion_state',return_value='recovered') as recover, patch.object(w.probe,'fresh',side_effect=TimeoutError('lost')) as fresh:
    assert w.writing_state(None,None,None,'test',None,script,[],True)=='recovered'
    try:
        w.writing_state(None,None,None,'test',None,script,[],False)
    except TimeoutError:
        pass
    else:
        raise AssertionError('writing must not auto-resume after lost state')
    assert recover.call_count==1 and fresh.call_count==1
with patch.object(w.probe,'stop') as stopped, patch.object(w.probe,'force_level',side_effect=[2.2,1.2,1.2]), patch.object(w.probe,'stationary_state',return_value=(np.zeros(6),np.zeros(6),np.zeros(6))), patch.object(w.probe,'model_gate') as gate, patch.object(w.probe,'move_step') as move:
    assert abs(w.unload_pen(None,None,None,'test',np.zeros(6),{},.001)-.0001)<1e-12
    assert stopped.call_count==1 and gate.call_count==1
    assert move.call_args.args[4][2]==.0001
with patch.object(w.probe,'stop'), patch.object(w.probe,'force_level',return_value=2.2), patch.object(w.probe,'move_step') as move:
    try:
        w.unload_pen(None,None,None,'test',np.zeros(6),{},0.)
    except RuntimeError:
        pass
    else:
        raise AssertionError('exhausted upward correction budget must stop')
    move.assert_not_called()

reg = {'corners_pen_tip_base_m':[[.4,-.3,.026968],[.697,-.3,.026968],[.4,-.09,.026968]],'tcp_sha256':'test'}
p = w.make_plan(reg,26.968,False)
import copy
probed=copy.deepcopy(p)
w.apply_contact_height(probed,{'status':'contact_recorded_retracted_2mm','contact_pen_tip_base_m':[.507348,-.155897,.025]})
assert all(pt[2]==.025 for stroke in probed['strokes_pen_tip_base_m'] for pt in stroke)
assert w.stroke_route(probed,.037,False)[0][0][2]==.037
for bad in [{'status':'stopped_on_error'}, {'status':'contact_recorded_retracted_2mm','contact_pen_tip_base_m':[.6,-.155897,.025]}]:
    try:
        w.apply_contact_height(copy.deepcopy(p),bad)
    except RuntimeError:
        pass
    else:
        raise AssertionError('failed or displaced contact must prevent writing')
assert abs(p['bounding_area_ratio']-.22664742664742665)<1e-6
assert np.allclose(p['strokes_pen_tip_base_m'][0][0],[.507348,-.155897,.026968])
air = w.make_plan(reg,26.968,True)
assert np.allclose(np.asarray(air['strokes_pen_tip_base_m'][0])-np.asarray(p['strokes_pen_tip_base_m'][0]),[0,0,.005])
# Rotating the paper rotates the glyph, without scaling millimetres.
rot = {'corners_pen_tip_base_m':[[.4,-.3,.026968],[.4,-.003,.026968],[.19,-.3,.026968]],'tcp_sha256':'test'}
r = w.make_plan(rot,26.968,False)
assert np.allclose(r['strokes_pen_tip_base_m'][0][0],[.255897,-.192652,.026968])
vertices = np.array([[.4,-.3,.026968],[.697,-.3,.026968],[.697,-.09,.026968],[.4,-.09,.026968]])
for triple in itertools.combinations(vertices.tolist(),3):
    for order in itertools.permutations(triple):
        ordered = w.unordered_corners(order)
        assert np.allclose(ordered,[vertices[0],vertices[1],vertices[3]])
theta = .7
R = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
rotated = vertices.copy(); rotated[:,:2] = vertices[:,:2] @ R.T
reference = w.unordered_corners(rotated[:3])
for triple in itertools.combinations(rotated.tolist(),3):
    for order in itertools.permutations(triple):
        assert np.allclose(w.unordered_corners(order),reference)
for corners in [
    [[.4,-.3,.027],[.697,-.3,.027],[.4,-.51,.027]],
    [[.4,-.3,.027],[.582,-.3,.027],[.4,-.019,.027]],
]:
    try:
        w.paper_axes(corners)
    except ValueError:
        pass
    else:
        raise AssertionError('invalid paper registration accepted')
uneven = {'corners_pen_tip_base_m':[[.4,-.3,.027],[.697,-.3,.03],[.4,-.09,.023]],'tcp_sha256':'test'}
assert np.allclose(w.make_plan(uneven,26.968,False)['strokes_pen_tip_base_m'][0],p['strokes_pen_tip_base_m'][0])
actual = [[.613283,-.023248,.026002],[.613267,-.310487,.025060],[.407285,-.311422,.024392]]
ordered = w.unordered_corners(actual)
actual_plan = w.make_plan({'corners_pen_tip_base_m':ordered,'tcp_sha256':'test'},26.968,False)
assert all(abs(pt[2]-.026968)<1e-12 for stroke in actual_plan['strokes_pen_tip_base_m'] for pt in stroke)
print('PASS: unordered corners, metric mapping, Z ignored for XY registration, user corner data, fixed writing height and safety geometry gates; no hardware accessed')
