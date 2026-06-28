#!/usr/bin/env python3
"""
Simulation visualizer - generates a standalone HTML visualization file.

Usage:
    cd src/robocup_planner
    python -m test.visualize
    # Then open test/sim_viz.html in any browser.
"""
import sys
import json
import pathlib
from typing import Any, Dict, List, Optional

pkg_root = pathlib.Path(__file__).resolve().parents[1]
if str(pkg_root) not in sys.path:
    sys.path.insert(0, str(pkg_root))

from robocup_planner.planning.aidlist_builder import compute_net_aidlist
from robocup_planner.planning.cargo_allocator import CargoAllocator, INTRANSIT_CARGO_IDS
from robocup_planner.planning.midlist_builder import build_full_midlist, build_mid
from robocup_planner.execution.executor import Executor, Plan
from robocup_planner.execution.cargo_state import CargoManager
from robocup_planner.product_catalog import get_material_count, product_name


STATION_LABELS: Dict[int, str] = {
    0: 'Home', 10: 'Workbench', 20: 'Customer',
    1: 'Storage 1', 2: 'Storage 2', 3: 'Storage 3', 4: 'Storage 4',
}

# Arena X positions (used for distance calc mock)
_POSITIONS: Dict[int, tuple] = {
    0:  (0.0, 0.0),
    10: (0.5, 0.0),
    20: (5.0, 0.0),
    1:  (1.0, 0.0),
    2:  (2.0, 0.0),
    3:  (3.0, 0.0),
    4:  (4.0, 0.0),
}


# ---------------------------------------------------------------------------
# Recording infrastructure
# ---------------------------------------------------------------------------

class RecordingNode:
    """Silent mock node that records all actions into a list."""

    def __init__(self, executor_holder: list, wb_auto_signal_on_nav: Optional[int] = None):
        self._log: List[Dict[str, Any]] = []
        self._executor_holder = executor_holder
        self._wb_auto_signal_on_nav = wb_auto_signal_on_nav
        self._nav_count = 0

    def get_logger(self):
        class _L:
            def info(self, m): pass
            def warning(self, m): pass
            def error(self, m): pass
        return _L()

    def navigate(self, station_id: int) -> bool:
        self._nav_count += 1
        label = f'navigate → {STATION_LABELS.get(station_id, station_id)}'
        self._log.append({'type': 'navigate', 'station_id': station_id, 'label': label})
        if self._wb_auto_signal_on_nav and self._nav_count == self._wb_auto_signal_on_nav:
            ex = self._executor_holder[0]
            if ex is not None:
                ex.wb_signal.set()
        return True

    def arm_pick_material(self, station_id: int, material_id: int, manipulator_slot: int) -> bool:
        cargo_id = manipulator_slot // 10
        placement = manipulator_slot % 10
        label = f'arm_pick_material: mat={material_id} → cargo{cargo_id}[{placement}]'
        self._log.append({
            'type': 'arm_pick_material',
            'material_id': material_id,
            'slot': manipulator_slot,
            'station_id': station_id,
            'label': label,
        })
        return True

    def arm_pick_product(self, station_id: int, product_id: int) -> bool:
        label = f'arm_pick_product: pid={product_id} from station {station_id}'
        self._log.append({'type': 'arm_pick_product', 'product_id': product_id,
                          'station_id': station_id, 'label': label})
        return True

    def arm_unload_material(self, cargo_id: int, placement_idx: int) -> bool:
        label = f'arm_unload_material: cargo{cargo_id}[{placement_idx}] → workbench'
        self._log.append({'type': 'arm_unload_material', 'cargo_id': cargo_id,
                          'placement_idx': placement_idx, 'label': label})
        return True

    def arm_deliver(self, from_cargo_id: int) -> bool:
        label = f'arm_deliver: from cargo {from_cargo_id} → customer'
        self._log.append({'type': 'arm_deliver', 'from_cargo_id': from_cargo_id, 'label': label})
        return True

    def wb_task(self, work_type: str, product_id: int) -> bool:
        label = f'wb_task: {work_type} product {product_id}'
        self._log.append({'type': 'wb_task', 'work_type': work_type,
                          'product_id': product_id, 'label': label})
        return True

    @property
    def action_log(self) -> List[Dict[str, Any]]:
        return list(self._log)


# ---------------------------------------------------------------------------
# State replay tracker
# ---------------------------------------------------------------------------

class StateTracker:
    def __init__(self, intransit_products: List[int]):
        self._cargo = CargoManager()
        self._alloc = CargoAllocator()
        self._alloc.allocate(intransit_products)
        self._wb_materials: List[int] = []
        self._wb_status: str = 'idle'
        self._wb_product: Optional[int] = None
        self._robot_pos: Optional[int] = None
        self._robot_trail: List[int] = []

    def _wb_clear_event(self) -> None:
        if self._wb_status in ('assembled', 'recycled'):
            self._wb_status = 'loaded' if self._wb_materials else 'idle'
            self._wb_product = None

    def apply(self, action: Dict[str, Any]) -> None:
        t = action['type']
        if t == 'navigate':
            sid = action['station_id']
            self._robot_trail.append(sid)
            self._robot_pos = sid
            self._wb_clear_event()
        elif t == 'arm_pick_product':
            self._wb_clear_event()
        elif t == 'arm_pick_material':
            mat, slot = action['material_id'], action['slot']
            cid = slot // 10
            if cid in INTRANSIT_CARGO_IDS:
                self._alloc.confirm_placed(cid, mat)
            else:
                self._cargo._slots[cid].place(mat, slot % 10)
            self._wb_clear_event()
        elif t == 'arm_unload_material':
            cid, idx = action['cargo_id'], action['placement_idx']
            mat = self._cargo._slots[cid]._contents.get(idx)
            if mat is not None:
                self._wb_materials.append(mat)
            self._cargo.remove_material(cid, idx)
            self._wb_status = 'loaded'
        elif t == 'wb_task':
            pid, wt = action['product_id'], action['work_type']
            if wt == 'RECYCLE':
                for mat_id, cnt in get_material_count(pid).items():
                    for _ in range(cnt):
                        self._cargo.place_material(mat_id)
                self._wb_materials = []
                self._wb_product = pid
                self._wb_status = 'recycled'
            elif wt == 'PRODUCE':
                self._cargo.add_finished_product()
                self._wb_materials = []
                self._wb_product = pid
                self._wb_status = 'assembled'
        elif t == 'arm_deliver':
            cid = action['from_cargo_id']
            if cid == 1:
                self._cargo.consume_finished_product()
            elif cid in INTRANSIT_CARGO_IDS:
                self._alloc.free_slot(cid)
            self._wb_clear_event()

    def snapshot(self) -> Dict[str, Any]:
        cargo = {
            str(cid): {str(i): mat for i, mat in slot._contents.items()}
            for cid, slot in self._cargo._slots.items()
        }
        intransit = {}
        for cid in INTRANSIT_CARGO_IDS:
            s = self._alloc._slots.get(cid)
            intransit[str(cid)] = {
                'product_id': s.product_id,
                'product_name': product_name(s.product_id),
                'build_order': list(s.build_order),
                'placed': list(s.placed),
                'complete': s.is_complete,
            } if s else None
        return {
            'cargo': cargo,
            'cargo1': self._cargo.finished_on_cargo1,
            'intransit': intransit,
            'workbench': {
                'materials': list(self._wb_materials),
                'status': self._wb_status,
                'product_id': self._wb_product,
                'product_name': product_name(self._wb_product) if self._wb_product else None,
            },
            'robot': {
                'station_id': self._robot_pos,
                'trail': list(self._robot_trail),
            },
        }


# ---------------------------------------------------------------------------
# Scenario runner
# ---------------------------------------------------------------------------

def _make_calc_mock():
    import math
    from unittest.mock import MagicMock
    calc = MagicMock()
    def get_pos(sid): return _POSITIONS.get(sid)
    def point_to_station(x, y, sid):
        pos = _POSITIONS.get(sid)
        return math.sqrt((x - pos[0])**2 + (y - pos[1])**2) if pos else float('inf')
    calc.get_position.side_effect = get_pos
    calc.point_to_station.side_effect = point_to_station
    return calc


def _build_plan(produce_ids, recycle_ids, storage_spec,
                workbench_id=10, customer_id=20, home_id=0):
    calc = _make_calc_mock()
    storage = [{'station_id': sid, 'material_ids': mats} for sid, mats in storage_spec.items()]
    _, net_aidlist, _ = compute_net_aidlist(produce_ids, recycle_ids)
    needs_recycling = bool(recycle_ids)
    recycle_orders = [{'station_id': customer_id, 'product_id': pid} for pid in recycle_ids]
    full_midlist = build_full_midlist(
        storage_stations=storage, customer_stations=[],
        recycle_orders=recycle_orders, calc=calc,
        home_station_id=home_id, workbench_station_id=workbench_id,
        needs_recycling=needs_recycling,
    )
    mid = build_mid(full_midlist, net_aidlist)
    temp_alloc = CargoAllocator()
    intransit_allocated = temp_alloc.allocate(produce_ids)
    intransit_ids = list(intransit_allocated.keys())
    workbench_ids = [pid for pid in produce_ids if pid not in intransit_ids]
    return Plan(
        mid=mid, workbench_products=workbench_ids, intransit_products=intransit_ids,
        workbench_station_id=workbench_id, customer_station_id=customer_id,
        home_station_id=home_id,
    )


def run_scenario_record(title: str, produce_ids, recycle_ids, storage_spec,
                        wb_auto_signal_nav=None) -> Dict[str, Any]:
    plan = _build_plan(produce_ids, recycle_ids, storage_spec)
    tracker = StateTracker(plan.intransit_products)

    executor_holder = [None]
    node = RecordingNode(executor_holder, wb_auto_signal_on_nav=wb_auto_signal_nav)
    executor = Executor(plan, node)
    executor_holder[0] = executor

    try:
        executor.run()
        raw_log = node.action_log
        delivers = sum(1 for a in raw_log if a['type'] == 'arm_deliver')
        expected = len(plan.workbench_products) + len(plan.intransit_products)
        status = 'PASS' if delivers == expected else f'FAIL (got {delivers}, expected {expected})'
    except Exception as e:
        raw_log = node.action_log
        status = f'ERROR: {e}'

    # Build steps list: step 0 = initial state, then one step per action
    steps = [{'action': None, 'state': tracker.snapshot()}]
    for action in raw_log:
        tracker.apply(action)
        steps.append({'action': action, 'state': tracker.snapshot()})

    wb_prods = [{'id': pid, 'name': product_name(pid)} for pid in plan.workbench_products]
    it_prods = [{'id': pid, 'name': product_name(pid)} for pid in plan.intransit_products]

    return {
        'title': title,
        'status': status,
        'workbench': wb_prods,
        'intransit': it_prods,
        'steps': steps,
    }


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

SCENARIOS = [
    dict(title='1 - Single in-transit (E-Stop)',
         produce_ids=[81], recycle_ids=[], storage_spec={1: [8, 1]}),
    dict(title='2 - Two in-transit (E-Stop + Battery)',
         produce_ids=[81, 34], recycle_ids=[], storage_spec={1: [8, 3], 2: [1, 4]}),
    dict(title='3 - Workbench-only (Big Tree)',
         produce_ids=[8518], recycle_ids=[], storage_spec={1: [8, 8], 2: [5, 1]},
         wb_auto_signal_nav=3),
    dict(title='4 - Mixed (E-Stop in-transit + Burger workbench)',
         produce_ids=[81, 48132], recycle_ids=[],
         storage_spec={1: [8, 8, 4], 2: [1, 1, 2], 3: [3]}),
    dict(title='5 - Recycle covers all (Big Tree → Big Tree)',
         produce_ids=[8518], recycle_ids=[8518], storage_spec={}),
    dict(title='6 - Partial recycle (Big Tree + recycle E-Stop)',
         produce_ids=[8518], recycle_ids=[81], storage_spec={1: [8, 5]}),
    dict(title='7 - Overflow to workbench (3 products)',
         produce_ids=[81, 34, 13], recycle_ids=[],
         storage_spec={1: [8, 3], 2: [1, 4], 3: [1, 3]},
         wb_auto_signal_nav=4),
]


# ---------------------------------------------------------------------------
# HTML Template — simple, table-based, information-dense
# ---------------------------------------------------------------------------

HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Sim Viz</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font-family: monospace; font-size: 12px; background: #f0f0f0; color: #111; }

/* ── Top bar ── */
#topbar {
  display: flex; align-items: center; gap: 8px; flex-wrap: wrap;
  padding: 5px 8px; background: #fff; border-bottom: 2px solid #888;
  position: sticky; top: 0; z-index: 10;
}
select, button { font-family: monospace; font-size: 12px; padding: 2px 6px; cursor: pointer; }
.badge { font-weight: bold; padding: 1px 8px; border-radius: 2px; }
.badge.pass  { background: #c8e6c9; color: #1b5e20; }
.badge.fail  { background: #ffcdd2; color: #b71c1c; }
.badge.error { background: #ffe0b2; color: #bf360c; }
#step-info { font-weight: bold; min-width: 80px; }
#plan-info { color: #555; font-size: 11px; margin-left: auto; }

/* ── Layout: sidebar + main ── */
#layout { display: flex; height: calc(100vh - 34px); overflow: hidden; }

/* ── Sidebar: action log ── */
#sidebar {
  width: 310px; flex-shrink: 0;
  border-right: 2px solid #888;
  display: flex; flex-direction: column; overflow: hidden;
  background: #fff;
}
#sidebar-hdr {
  padding: 3px 8px; background: #ddd; border-bottom: 1px solid #999;
  font-size: 10px; font-weight: bold; letter-spacing: 1px; flex-shrink: 0;
}
#action-log { flex: 1; overflow-y: auto; }
#action-log table { width: 100%; border-collapse: collapse; }
#action-log td { border-bottom: 1px solid #eee; padding: 3px 6px; vertical-align: top; }
#action-log tr.active td { background: #bbdefb; }
#action-log tr.done td  { color: #aaa; }
#action-log tr.future td{ color: #ccc; }
#action-log tr:hover td { background: #e3f2fd; cursor: pointer; }
.log-num  { color: #999; text-align: right; width: 22px; font-size: 10px; white-space: nowrap; }
.log-type { font-size: 10px; color: #666; }
.log-lbl  { font-size: 11px; }
#action-log tr.active .log-lbl { font-weight: bold; color: #0d47a1; }

/* ── Main area ── */
#main { flex: 1; overflow: auto; padding: 8px; display: flex; flex-direction: column; gap: 8px; }

/* ── Sections ── */
.sec { background: #fff; border: 1px solid #ccc; }
.sec-hdr {
  padding: 3px 8px; background: #e0e0e0; border-bottom: 1px solid #ccc;
  font-size: 10px; font-weight: bold; letter-spacing: 1px;
}
.sec-body { padding: 6px 8px; }

/* ── Tables ── */
table.t { border-collapse: collapse; width: 100%; font-size: 12px; }
table.t th { background: #f5f5f5; text-align: left; padding: 2px 6px; border: 1px solid #ccc; font-size: 10px; }
table.t td { padding: 2px 6px; border: 1px solid #ddd; vertical-align: middle; }
table.t tr:hover { background: #fafafa; }

/* ── Material chips ── */
.chip {
  display: inline-flex; align-items: center; justify-content: center;
  width: 22px; height: 22px; border-radius: 2px;
  font-size: 10px; font-weight: bold; color: #fff; margin: 1px;
}
.empty-cell { color: #ccc; font-size: 10px; }

/* ── Row groupings ── */
.row2 { display: flex; gap: 8px; }
.row2 > .sec { flex: 1; min-width: 0; }
</style>
</head>
<body>

<div id="topbar">
  <select id="scenario-sel" onchange="selectScenario(this.value)"></select>
  <span id="status-badge" class="badge"></span>
  <span id="step-info"></span>
  <button onclick="setStep(0)" title="First">⏮</button>
  <button onclick="stepBy(-1)" title="Prev (←)">◀</button>
  <button onclick="stepBy(1)"  title="Next (→)">▶</button>
  <button onclick="setStep(9999)" title="Last">⏭</button>
  <button id="btn-play" onclick="togglePlay()">▶ Play</button>
  <span id="plan-info"></span>
</div>

<div id="layout">

  <!-- Action log -->
  <div id="sidebar">
    <div id="sidebar-hdr">ACTION LOG</div>
    <div id="action-log"><table id="log-tbl"></table></div>
  </div>

  <!-- Main panels -->
  <div id="main">

    <!-- Map -->
    <div class="sec">
      <div class="sec-hdr">ROBOT NAVIGATION MAP — X = arena position (m), Y = visit order</div>
      <div class="sec-body" id="map-area"></div>
    </div>

    <!-- Cargo storage + intransit -->
    <div class="row2">
      <div class="sec">
        <div class="sec-hdr">CARGO 2–6 (storage trays) — placement cols: P0[0-1] P1[0-3] P2[2-3] P3[2-5] P4[4-5]</div>
        <div class="sec-body">
          <table class="t" id="cargo-tbl"></table>
        </div>
      </div>
      <div class="sec" style="min-width:340px">
        <div class="sec-hdr">CARGO 7/8 (in-transit assembly)</div>
        <div class="sec-body">
          <table class="t" id="intransit-tbl"></table>
        </div>
      </div>
    </div>

    <!-- Workbench + Cargo 1 -->
    <div class="row2">
      <div class="sec">
        <div class="sec-hdr">WORKBENCH</div>
        <div class="sec-body">
          <table class="t" id="wb-tbl"></table>
        </div>
      </div>
      <div class="sec" style="min-width:120px;max-width:160px">
        <div class="sec-hdr">CARGO 1 (finished)</div>
        <div class="sec-body" id="c1-area" style="text-align:center;padding:10px 0"></div>
      </div>
    </div>

  </div><!-- #main -->
</div><!-- #layout -->

<script>
const SIM_DATA = /*SIMDATA*/;

const MAT_COLOR = {
  1:'#e53935', 2:'#43a047', 3:'#1e88e5', 4:'#fdd835',
  5:'#b71c1c', 6:'#1b5e20', 7:'#0d47a1', 8:'#e64a19',
};
const MAT_NAME = {
  1:'R-2x2', 2:'G-2x2', 3:'B-2x2', 4:'Y-2x2',
  5:'R-2x4', 6:'G-2x4', 7:'B-2x4', 8:'Y-2x4',
};
const ACT_COLOR = {
  navigate:'#1565c0', arm_pick_material:'#e65100',
  arm_pick_product:'#880e4f', arm_unload_material:'#6a1b9a',
  arm_deliver:'#2e7d32', wb_task:'#f57f17',
};
// Arena station data
const STATIONS = [
  {id:0,  name:'Home',     short:'H',  x:0.0},
  {id:10, name:'WB',       short:'WB', x:0.5},
  {id:1,  name:'S1',       short:'S1', x:1.0},
  {id:2,  name:'S2',       short:'S2', x:2.0},
  {id:3,  name:'S3',       short:'S3', x:3.0},
  {id:4,  name:'S4',       short:'S4', x:4.0},
  {id:20, name:'Customer', short:'C',  x:5.0},
];
const SID2X   = {}; STATIONS.forEach(s=>SID2X[s.id]=s.x);
const SID2NAME= {}; STATIONS.forEach(s=>SID2NAME[s.id]=s.name);

let scenarioIdx=0, stepIdx=0, playTimer=null;
function scenario(){return SIM_DATA[scenarioIdx];}
function totalSteps(){return scenario().steps.length-1;}

// ── Init ─────────────────────────────────────────────────────────────────
function init(){
  const sel=document.getElementById('scenario-sel');
  SIM_DATA.forEach((s,i)=>{
    const o=document.createElement('option');
    o.value=i; o.textContent=s.title; sel.appendChild(o);
  });
  selectScenario(0);
  document.addEventListener('keydown',e=>{
    if(e.key==='ArrowLeft') {stopPlay();stepBy(-1);}
    if(e.key==='ArrowRight'){stopPlay();stepBy(1);}
    if(e.key===' '){togglePlay();e.preventDefault();}
    if(e.key==='Home'){stopPlay();setStep(0);}
    if(e.key==='End') {stopPlay();setStep(9999);}
  });
}

function selectScenario(idx){
  stopPlay(); scenarioIdx=parseInt(idx);
  document.getElementById('scenario-sel').value=idx;
  const s=scenario();
  const b=document.getElementById('status-badge');
  b.textContent=s.status;
  b.className='badge '+(s.status==='PASS'?'pass':s.status.startsWith('ERROR')?'error':'fail');
  const wb=s.workbench.map(p=>p.name).join(', ')||'—';
  const it=s.intransit.map(p=>p.name).join(', ')||'—';
  document.getElementById('plan-info').textContent=
    `Workbench: ${wb}  |  In-transit: ${it}`;
  setStep(0);
}

function setStep(n){stepIdx=Math.max(0,Math.min(n,totalSteps()));renderAll();}
function stepBy(d){setStep(stepIdx+d);}

function togglePlay(){
  if(playTimer){stopPlay();return;}
  document.getElementById('btn-play').textContent='⏹ Stop';
  playTimer=setInterval(()=>{
    if(stepIdx>=totalSteps()){stopPlay();return;}stepBy(1);
  },500);
}
function stopPlay(){
  if(!playTimer)return;
  clearInterval(playTimer);playTimer=null;
  document.getElementById('btn-play').textContent='▶ Play';
}

// ── Main render ───────────────────────────────────────────────────────────
function renderAll(){
  const steps=scenario().steps;
  const step=steps[stepIdx];
  document.getElementById('step-info').textContent=
    `Step ${stepIdx} / ${totalSteps()}`;
  renderLog(steps);
  renderMap(steps);
  renderCargo(step.state);
  renderIntransit(step.state);
  renderWorkbench(step.state);
  renderCargo1(step.state);
}

// ── Action log ────────────────────────────────────────────────────────────
function renderLog(steps){
  let rows='';
  steps.forEach((s,i)=>{
    const a=s.action;
    const cls=i===stepIdx?'active':i<stepIdx?'done':'future';
    const type=a?a.type:'—';
    const col=ACT_COLOR[type]||'#888';
    const label=a?a.label:'(initial state)';
    rows+=`<tr class="${cls}" onclick="setStep(${i})">
      <td class="log-num">${i}</td>
      <td>
        <span class="log-type" style="color:${col}">${type}</span><br>
        <span class="log-lbl">${label}</span>
      </td>
    </tr>`;
  });
  document.getElementById('log-tbl').innerHTML=rows;
  const active=document.querySelector('#log-tbl tr.active');
  if(active) active.scrollIntoView({block:'nearest'});
}

// ── Navigation map — 2D plot (X=arena_x, Y=visit_index) ──────────────────
function renderMap(steps){
  // Collect all navigate actions with their step index
  const visited=[], future=[];
  steps.forEach((s,i)=>{
    if(s.action&&s.action.type==='navigate'){
      (i<=stepIdx?visited:future).push({step:i, sid:s.action.station_id});
    }
  });
  const allNav=[...visited,...future];
  const totalNav=allNav.length;

  const W=700, H=180, pL=70, pR=20, pT=18, pB=35;
  const plotW=W-pL-pR, plotH=H-pT-pB;
  const maxY=Math.max(totalNav-1,1);
  const px=ax=>pL+ax/5.0*plotW;
  const py=vi=>pT+vi/maxY*plotH;

  let s=`<svg width="100%" viewBox="0 0 ${W} ${H}" style="max-height:180px;display:block"
    xmlns="http://www.w3.org/2000/svg">`;

  // Grid verticals + X labels
  STATIONS.forEach(st=>{
    const x=px(st.x);
    s+=`<line x1="${x}" y1="${pT}" x2="${x}" y2="${pT+plotH}" stroke="#e0e0e0" stroke-width="1"/>`;
    s+=`<text x="${x}" y="${H-18}" text-anchor="middle" font-size="10" fill="#555">${st.name}</text>`;
    s+=`<text x="${x}" y="${H-7}" text-anchor="middle" font-size="9" fill="#aaa">(${st.x}m)</text>`;
  });
  // Axes
  s+=`<line x1="${pL}" y1="${pT}" x2="${pL}" y2="${pT+plotH}" stroke="#666" stroke-width="1.5"/>`;
  s+=`<line x1="${pL}" y1="${pT+plotH}" x2="${pL+plotW}" y2="${pT+plotH}" stroke="#666" stroke-width="1.5"/>`;
  // Y-axis label
  s+=`<text x="12" y="${pT+plotH/2+4}" text-anchor="middle" font-size="10" fill="#888"
      transform="rotate(-90,12,${pT+plotH/2})">visit #</text>`;

  // Path lines
  for(let i=1;i<allNav.length;i++){
    const a=allNav[i-1], b=allNav[i];
    const x0=px(SID2X[a.sid]??0), y0=py(i-1);
    const x1=px(SID2X[b.sid]??0), y1=py(i);
    const isPast=i<=visited.length;
    s+=`<line x1="${x0}" y1="${y0}" x2="${x1}" y2="${y1}"
        stroke="${isPast?'#1565c0':'#bdbdbd'}" stroke-width="${isPast?2:1}"
        stroke-dasharray="${isPast?'none':'4,3'}"/>`;
  }

  // Points
  allNav.forEach((e,i)=>{
    const x=px(SID2X[e.sid]??0), y=py(i);
    const isCur=(i===visited.length-1);
    const isPast=(i<visited.length);
    // Y-axis tick
    if(isPast||isCur)
      s+=`<text x="${pL-4}" y="${y+4}" text-anchor="end" font-size="9" fill="#aaa">${i}</text>`;

    if(isCur){
      s+=`<circle cx="${x}" cy="${y}" r="8" fill="#1565c0"/>`;
      s+=`<text x="${x}" y="${y+4}" text-anchor="middle" font-size="9" fill="#fff" font-weight="bold">${SID2NAME[e.sid]||e.sid}</text>`;
    } else if(isPast){
      s+=`<circle cx="${x}" cy="${y}" r="4" fill="#90caf9" stroke="#1565c0" stroke-width="1"/>`;
      s+=`<text x="${x+7}" y="${y+4}" font-size="9" fill="#555">${SID2NAME[e.sid]||e.sid}</text>`;
    } else {
      s+=`<circle cx="${x}" cy="${y}" r="3" fill="#e0e0e0" stroke="#bbb" stroke-width="1"/>`;
    }
  });

  // Legend
  s+=`<circle cx="${pL+plotW-100}" cy="${pT+8}" r="4" fill="#90caf9" stroke="#1565c0" stroke-width="1"/>`;
  s+=`<text x="${pL+plotW-92}" y="${pT+12}" font-size="9" fill="#555">visited</text>`;
  s+=`<circle cx="${pL+plotW-50}" cy="${pT+8}" r="4" fill="#1565c0"/>`;
  s+=`<text x="${pL+plotW-42}" y="${pT+12}" font-size="9" fill="#555">current</text>`;

  s+='</svg>';
  document.getElementById('map-area').innerHTML=s;
}

// ── Cargo 2-6 table ───────────────────────────────────────────────────────
function chip(mat){
  if(mat===null||mat===undefined) return '<span class="empty-cell">—</span>';
  const c=MAT_COLOR[mat]||'#999';
  return `<span class="chip" style="background:${c}" title="${MAT_NAME[mat]||mat}">${mat}</span>`;
}

function renderCargo(state){
  let h=`<tr><th>Cargo</th><th>P0 (2×2)</th><th>P1 (2×4)</th><th>P2 (2×2)</th><th>P3 (2×4)</th><th>P4 (2×2)</th></tr>`;
  for(const cid of [2,3,4,5,6]){
    const c=state.cargo[String(cid)]||{};
    h+=`<tr><td><b>${cid}</b></td>`;
    for(let i=0;i<5;i++){
      const mat=c[String(i)];
      const hasVal=(mat!==null&&mat!==undefined);
      h+=`<td>${chip(hasVal?mat:null)}${hasVal?` <span style="font-size:10px;color:#666">${MAT_NAME[mat]||''}</span>`:''}`;
      h+=`</td>`;
    }
    h+='</tr>';
  }
  document.getElementById('cargo-tbl').innerHTML=h;
}

// ── Intransit 7/8 table ───────────────────────────────────────────────────
function renderIntransit(state){
  let h=`<tr><th>Cargo</th><th>Product</th><th>Build order (✓=placed, →=next, ·=pending)</th><th>Progress</th><th>Complete</th></tr>`;
  for(const cid of [7,8]){
    const sl=state.intransit[String(cid)];
    if(!sl){
      h+=`<tr><td><b>${cid}</b></td><td colspan="4" class="empty-cell">— not allocated —</td></tr>`;
      continue;
    }
    const placed=sl.placed.length, total=sl.build_order.length;
    const orderHtml=sl.build_order.map((m,i)=>{
      const c=MAT_COLOR[m]||'#999';
      const marker=i<placed?'✓':i===placed?'→':'·';
      const opacity=i<placed?1:i===placed?1:0.3;
      return `<span style="display:inline-flex;align-items:center;gap:1px;margin-right:3px">
        <span class="chip" style="background:${c};opacity:${opacity}">${m}</span><span style="font-size:10px">${marker}</span></span>`;
    }).join('');
    const done=sl.complete;
    h+=`<tr>
      <td><b>${cid}</b></td>
      <td style="font-size:11px">${sl.product_name}</td>
      <td>${orderHtml}</td>
      <td>${placed}/${total}</td>
      <td style="color:${done?'#1b5e20':'#888'};font-weight:${done?'bold':'normal'}">${done?'YES':'No'}</td>
    </tr>`;
  }
  document.getElementById('intransit-tbl').innerHTML=h;
}

// ── Workbench table ───────────────────────────────────────────────────────
function renderWorkbench(state){
  const wb=state.workbench;
  const STATUS_BG={idle:'#f5f5f5',loaded:'#e3f2fd',assembled:'#e8f5e9',recycled:'#f3e5f5'};
  const STATUS_FG={idle:'#888',loaded:'#0d47a1',assembled:'#1b5e20',recycled:'#4a148c'};
  const mats=wb.materials.map(m=>chip(m)).join(' ')||'<span class="empty-cell">—</span>';
  const bg=STATUS_BG[wb.status]||'#f5f5f5';
  const fg=STATUS_FG[wb.status]||'#333';
  let h=`<tr><th>Status</th><th>Product</th><th>Materials on WB</th></tr>
  <tr>
    <td><span style="background:${bg};color:${fg};padding:2px 8px;border-radius:2px;font-weight:bold">${wb.status}</span></td>
    <td style="font-size:11px">${wb.product_name||'—'}</td>
    <td>${mats}</td>
  </tr>`;
  document.getElementById('wb-tbl').innerHTML=h;
}

// ── Cargo 1 ───────────────────────────────────────────────────────────────
function renderCargo1(state){
  const n=state.cargo1;
  document.getElementById('c1-area').innerHTML=
    `<div style="font-size:32px;font-weight:bold;color:${n>0?'#1b5e20':'#ccc'}">${n}</div>
     <div style="font-size:11px;color:#888;margin-top:4px">finished product${n!==1?'s':''}</div>`;
}

init();
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print('Running all 7 scenarios...')
    all_data = []
    for s in SCENARIOS:
        print(f'  {s["title"]}...', end=' ', flush=True)
        result = run_scenario_record(**s)
        print(result['status'])
        all_data.append(result)

    data_json = json.dumps(all_data, ensure_ascii=False)
    html = HTML_TEMPLATE.replace('/*SIMDATA*/', data_json)

    out = pathlib.Path(__file__).parent / 'sim_viz.html'
    out.write_text(html, encoding='utf-8')

    passed = sum(1 for d in all_data if d['status'] == 'PASS')
    print(f'\n{passed}/{len(all_data)} scenarios PASS')
    print(f'Generated: {out.resolve()}')
    print(f'Open in browser:  file:///{out.resolve().as_posix()}')


if __name__ == '__main__':
    main()
