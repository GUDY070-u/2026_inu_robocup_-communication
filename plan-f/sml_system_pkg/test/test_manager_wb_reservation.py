import threading

from sml_msgs.msg import Step

from sml_system_pkg.sml_manager_node import SmlManagerNode


class _Logger:
    def info(self, _message):
        pass


def _step(step_id, type_, action, station_id):
    msg = Step()
    msg.step_id = int(step_id)
    msg.type = int(type_)
    msg.action = int(action)
    msg.station_id = int(station_id)
    return msg


def _manager():
    manager = object.__new__(SmlManagerNode)
    manager._lock = threading.Lock()
    manager.pending_steps = []
    manager.completed_steps = set()
    manager.amr_busy = False
    manager.wb_busy = False
    manager.wb_reserved_by_amr = None
    manager.fixed_workbench_station = 6
    manager._duration_summary_done = False
    manager.plan_end_time = None
    manager._mark_step_started = lambda _step: None
    manager._publish_status = lambda _message: None
    manager._log_step_duration_summary = lambda: None
    manager.get_logger = lambda: _Logger()
    manager.started_amr = []
    manager.started_wb = []
    manager._execute_amr = lambda step: manager.started_amr.append(step.step_id)
    manager._execute_wb = lambda step: manager.started_wb.append(step.step_id)
    return manager


def test_wb_interaction_reserves_wb_and_blocks_wb_action():
    manager = _manager()
    amr_wb = _step(1, Step.AMR, Step.UNLOAD, 6)
    wb_action = _step(2, Step.WB, Step.PRODUCE, 6)
    manager.pending_steps = [amr_wb, wb_action]

    manager._dispatch()

    assert manager.started_amr == [1]
    assert manager.started_wb == []
    assert manager.wb_reserved_by_amr == 1

    manager._set_amr_idle(amr_wb)
    assert manager.wb_reserved_by_amr is None


def test_amr_does_other_work_while_wb_is_busy():
    manager = _manager()
    manager.wb_busy = True
    blocked_wb_unload = _step(1, Step.AMR, Step.UNLOAD, 6)
    customer_load = _step(2, Step.AMR, Step.LOAD, 8)
    manager.pending_steps = [blocked_wb_unload, customer_load]

    manager._dispatch()

    assert manager.started_amr == [2]
    assert blocked_wb_unload in manager.pending_steps
    assert manager.wb_reserved_by_amr is None
