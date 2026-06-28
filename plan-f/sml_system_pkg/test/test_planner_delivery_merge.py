from sml_msgs.msg import Order, Station, Step, Task

from sml_system_pkg.planning.planner_core import PlannerCore


def _order(product_id):
    msg = Order()
    msg.order_type = Order.OT_PRODUCE
    msg.product_id = int(product_id)
    return msg


def _station(station_id, station_type, materials=None):
    msg = Station()
    msg.station_id = int(station_id)
    msg.station_type = int(station_type)
    msg.material_ids = [int(x) for x in (materials or [])]
    return msg


def _task(product_ids, materials):
    task = Task()
    task.order_list = [_order(pid) for pid in product_ids]
    task.arena_layout = [
        _station(1, Station.ST_STORAGE, materials),
        _station(2, Station.ST_STORAGE),
        _station(3, Station.ST_WORKBENCH),
        _station(4, Station.ST_STORAGE),
        _station(5, Station.ST_STORAGE),
        _station(6, Station.ST_WORKBENCH),
        _station(7, Station.ST_WORKBENCH),
        _station(8, Station.ST_CUSTOMER),
    ]
    return task


def _steps(plan, action, type_=Step.AMR, station_id=None):
    return [
        step for step in plan
        if int(step.type) == int(type_)
        and int(step.action) == int(action)
        and (station_id is None or int(step.station_id) == int(station_id))
    ]


def test_two_amr_products_are_produced_and_unloaded_as_one_batch():
    plan = PlannerCore().build_plan(
        _task([34, 13], [3, 4, 1, 3])
    )

    produce_steps = _steps(plan, Step.PRODUCE)
    assert len(produce_steps) == 1
    assert list(produce_steps[0].object_ids) == [34, 13]

    customer_unloads = _steps(plan, Step.UNLOAD, station_id=8)
    assert len(customer_unloads) == 1
    assert list(customer_unloads[0].object_ids) == [34, 13]
    assert len(customer_unloads[0].slide_ids) == 2


def test_last_wb_product_joins_amr_product_customer_unload():
    plan = PlannerCore().build_plan(
        _task([8518, 34], [8, 5, 1, 8, 3, 4])
    )

    customer_unloads = _steps(plan, Step.UNLOAD, station_id=8)
    assert len(customer_unloads) == 1
    assert list(customer_unloads[0].object_ids) == [8518, 34]
    assert [abs(int(sid)) % 10 for sid in customer_unloads[0].slide_ids] == [1, 7]

    produce_step = _steps(plan, Step.PRODUCE)[0]
    wb_product_load = next(
        step for step in plan
        if int(step.type) == int(Step.AMR)
        and int(step.action) == int(Step.LOAD)
        and list(step.object_ids) == [8518]
    )
    assert int(wb_product_load.step_id) in set(produce_step.depends_on)
