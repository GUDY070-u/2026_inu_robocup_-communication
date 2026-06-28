from collections import deque
from types import SimpleNamespace

from sml_system_pkg.sml_manager_node import SmlManagerNode


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warn(self, message):
        self.messages.append(('warn', message))


def _manager_for_slot_test():
    manager = object.__new__(SmlManagerNode)
    manager._arm_raw_slot_indices = [2, 3, 4, 5, 6]
    manager._arm_raw_slots = {
        slot: {'units': [None, None, None], 'items': {}}
        for slot in manager._arm_raw_slot_indices
    }
    manager._arm_item_keys = {}
    manager._arm_next_uid = 1
    manager._arm_cmd_slide_cache = {}
    manager._arm_pending_removals = {}
    manager._test_logger = _Logger()
    manager.get_logger = lambda: manager._test_logger
    return manager


def test_assembled_product_inherits_base_physical_position():
    manager = _manager_for_slot_test()

    # raw 8(size 2)은 logical slide 6에서 physical arm slide 51을 차지한다.
    assert manager._assign_raw_arm_position(6, 8) == 51

    assemble_step = SimpleNamespace(
        step_id=3,
        slide_ids=[6, 1],
        object_ids=[81],
    )
    assert manager._convert_step_slide_ids_for_arm(
        assemble_step, 'ASSEMBLE'
    ) == [51, 1]

    manager._commit_amr_assemble_slot_changes(assemble_step)

    # 완성품 81이 raw 8의 정확한 물리 위치 51을 이어받아야 한다.
    unload_step = SimpleNamespace(
        step_id=4,
        slide_ids=[6],
        object_ids=[81],
    )
    assert manager._convert_step_slide_ids_for_arm(
        unload_step, 'UNLOAD'
    ) == [51]
    assert not any(
        level == 'warn' and 'fallback' in message
        for level, message in manager._test_logger.messages
    )

    manager._commit_arm_slot_removals(unload_step)
    assert manager._arm_raw_slots[6]['units'] == [None, None, None]
    assert (6, 81) not in manager._arm_item_keys
