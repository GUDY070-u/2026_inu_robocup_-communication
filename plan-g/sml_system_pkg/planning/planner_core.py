"""Plan D planner core.

This module is intentionally separate from the old WB-centered planner.
It generates `sml_msgs/Step` with AMR internal `slide_ids`.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from typing import Dict, Iterable, List, Optional

from sml_msgs.msg import Order, Station, Step

from .planner_config import (
    AMR_CAPABLE_PRODUCTS,
    ASSEMBLY_SLOT_INDICES,
    BATCH_SIZE,
    BATCH_TO_RAW,
    PRODUCT_MATERIALS,
    PRODUCT_NAMES,
    PRODUCT_SLOT_INDEX,
    RAW_SLIDE_CAPACITY_UNITS,
    RAW_SLOT_INDICES,
    STATION_START_GOAL,
    WB_ONLY_PRODUCTS,
    PlannerConfig,
)

from .amr_inventory import (
    AmrInventory,
    InventoryItem,
    command_from_items,
    describe_items,
    ROLE_PRODUCE_WB_RAW,
    ROLE_AMR_ASSEMBLE_BASE,
    ROLE_AMR_ASSEMBLE_UPPER,
    ROLE_AMR_ASSEMBLE_PRODUCT,
    ROLE_WB_PRODUCT,
    ROLE_RECYCLE_PRODUCT,
    ROLE_RETURN_RAW,
    ROLE_REUSE_RAW,
)


class _NullLogger:
    def info(self, msg):
        pass

    def warn(self, msg):
        pass

    def error(self, msg):
        pass


class PlannerCore:
    """Build a Plan D step sequence from a `sml_msgs/Task`."""

    def __init__(
        self,
        config=None,
        station_coords=None,
        waypoint_cost_map=None,
        logger=None,
    ):
        self.config = config or PlannerConfig()
        self.station_coords = station_coords or {}
        self.waypoint_cost_map = waypoint_cost_map
        self._logger = logger or _NullLogger()
        self.use_time_cost = bool(self.config.use_time_cost)
        self.amr_speed_mps = float(self.config.amr_speed_mps)
        self._reset_runtime_state()

    def get_logger(self):
        return self._logger

    def _reset_runtime_state(self):
        self.steps: List[Step] = []
        self.step_id = 0
        self.current_station = STATION_START_GOAL
        self.recycle_step_by_order_index: Dict[int, int] = {}
        self.last_wb_clear_step_id: Optional[int] = None
        self.deferred_waste_jobs: List[dict] = []
        self.deferred_wb_product_delivery = None

        # Rich planner-side inventory metadata.  Step.msg stays compact
        # (object_ids/slide_ids), while this map lets the planner/debug log keep
        # role/order/slot/source/target information for each step.
        self.amr_inventory = AmrInventory()
        self.step_items: Dict[int, List[InventoryItem]] = {}

        # Recycle product pre-load pipeline state.
        #
        # `slide_id` includes order_index, but every recycle product occupies the
        # same physical product slot 1. It is considered free only after the AMR
        # UNLOAD-to-WB step completes.
        self.recycle_product_slot_free_after: Dict[int, Optional[int]] = {
            int(PRODUCT_SLOT_INDEX): None
        }
        self.last_recycle_unload_to_wb_step_id: Optional[int] = None

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def build_plan(self, task):
        self._reset_runtime_state()

        orders = self._parse_orders(task.order_list)
        station_info = self._parse_arena(task.arena_layout, orders)
        self._assign_sources(orders, station_info)

        produce_orders = [o for o in orders if o['order_type'] == Order.OT_PRODUCE]
        recycle_orders = [o for o in orders if o['order_type'] == Order.OT_RECYCLE]

        linked_recycles = self._linked_recycles(produce_orders)
        linked_recycle_indexes = {ro['order_index'] for ro in linked_recycles}
        standalone_recycles = [
            ro for ro in recycle_orders
            if ro['order_index'] not in linked_recycle_indexes
        ]

        wb_produces = [po for po in produce_orders if self._is_wb_only(po)]
        amr_assemble_orders = [po for po in produce_orders if not self._is_wb_only(po)]

        # 1. Lifecycle reuse first: recycle products that directly feed produce orders.
        for recycle_order in linked_recycles:
            self._append_recycle(recycle_order, station_info)

        # 2. WB-only production. WB can run while AMR later handles independent work,
        #    but AMR-WB interaction steps are protected by dependencies.
        wb_produces.sort(key=lambda po: len(po['materials']), reverse=True)
        for index, produce_order in enumerate(wb_produces):
            defer_customer_delivery = (
                bool(amr_assemble_orders)
                and index == len(wb_produces) - 1
            )
            self._append_wb_produce(
                produce_order,
                station_info,
                defer_customer_delivery=defer_customer_delivery,
            )

        # 3. AMR-capable production, using assembly slots 7/8 and raw slides 2~6.
        #    At most two AMR products are prepared together because there are two
        #    assembly slots.
        self._append_amr_assemble_batches(amr_assemble_orders, station_info)

        # 4. Recycle orders that do not help production are performed after useful work.
        #    In Recycling-only / standalone recycle flows, do not wait until the end
        #    to return all decomposed raw materials.  Instead, keep the WB busy:
        #      - transport one recycle product from CUSTOMER using slot 1;
        #      - when a WB recycle finishes, load its raw results and immediately
        #        exchange the next product into the WB;
        #      - while the WB processes that next product, return the previous raw.
        self._append_recycle_interleaved_group(standalone_recycles, station_info)

        # 5. Linked lifecycle recycle leftovers, if any, are still returned here.
        #    Standalone recycle jobs handled by _append_recycle_interleaved_group()
        #    do not add entries to deferred_waste_jobs.
        self._append_deferred_waste_returns(station_info)

        if self.steps:
            self._add_step(
                Step.AMR,
                Step.GOAL,
                [],
                STATION_START_GOAL,
                [s.step_id for s in self.steps],
                [],
            )

        self.get_logger().info(f'Plan D 계획 생성 완료: {len(self.steps)}개 스텝')
        self._log_plan_summary(orders)
        self._log_steps(self.steps)
        return self.steps

    # ------------------------------------------------------------------
    # Parsing and allocation
    # ------------------------------------------------------------------

    def _parse_orders(self, order_list):
        parsed = []
        for order_index, msg in enumerate(order_list):
            product_id = int(msg.product_id)
            materials = list(PRODUCT_MATERIALS.get(product_id, self._digits(product_id)))
            parsed.append({
                'order_index': int(order_index),
                'order_type': int(msg.order_type),
                'product_id': product_id,
                'materials': materials,
                'sources': [],
                'reuse_materials': [],
                'waste_materials': [],
                'used_by': [],
            })
        return parsed

    def _parse_arena(self, arena_layout, orders):
        station_items = {}
        storage_ids = []
        workbench_ids = []
        customer_id = None

        material_model = self._build_material_model(orders)
        initial_counts = Counter(material_model['produce_initial_counts'])
        waste_counts = Counter(material_model['recycle_leftover_counts'])

        stock_by_raw = defaultdict(list)
        waste_targets_by_raw = defaultdict(list)

        for station in arena_layout:
            sid = int(station.station_id)
            stype = int(station.station_type)
            material_ids = [int(x) for x in station.material_ids]
            station_items[sid] = list(material_ids)

            if stype == Station.ST_STORAGE:
                storage_ids.append(sid)
            elif stype == Station.ST_WORKBENCH:
                workbench_ids.append(sid)
            elif stype == Station.ST_CUSTOMER:
                customer_id = sid

            if stype not in (Station.ST_STORAGE, Station.ST_HYBRID):
                continue

            for object_id in material_ids:
                for raw in self._expand_station_object(object_id):
                    token = {
                        'station_id': sid,
                        'raw': int(raw),
                        # AMR receives actual raw IDs, not batch IDs.
                        'object_id': int(raw),
                    }
                    if initial_counts[raw] > 0:
                        stock_by_raw[raw].append(token)
                        initial_counts[raw] -= 1
                    elif waste_counts[raw] > 0:
                        waste_targets_by_raw[raw].append(token)
                        waste_counts[raw] -= 1
                    else:
                        # If an externally provided task has extra materials, keep them
                        # usable as stock instead of failing the whole task.
                        stock_by_raw[raw].append(token)

        if not workbench_ids:
            raise RuntimeError('arena_layout에 WORKBENCH station이 없습니다')
        if customer_id is None:
            raise RuntimeError('arena_layout에 CUSTOMER station이 없습니다')
        if not storage_ids:
            raise RuntimeError('arena_layout에 STORAGE station이 없습니다')

        fixed_wb = int(self.config.fixed_workbench_station_id)
        if fixed_wb not in workbench_ids:
            self.get_logger().warn(
                f'[WB] fixed workbench={fixed_wb}가 WORKBENCH 목록 {workbench_ids}에 없습니다. '
                f'fallback으로 {workbench_ids[0]} 사용'
            )
            fixed_wb = int(workbench_ids[0])
        self.get_logger().info(f'[WB] fixed workbench={fixed_wb} 사용')

        self.get_logger().info('===== Plan D lifecycle material model =====')
        self.get_logger().info(f'P produce_materials : {material_model["produce_materials"]}')
        self.get_logger().info(f'R recycle_materials : {material_model["recycle_materials"]}')
        self.get_logger().info(f'C common_reuse      : {material_model["common_reuse"]}')
        self.get_logger().info(f'P-C initial         : {material_model["produce_initial"]}')
        self.get_logger().info(f'R-C leftover        : {material_model["recycle_leftover"]}')
        self.get_logger().info('=========================================')

        return {
            'station_items': station_items,
            'storage_ids': sorted(storage_ids),
            'workbench_ids': sorted(workbench_ids),
            'wb_id': fixed_wb,
            'customer_id': int(customer_id),
            'stock_by_raw': stock_by_raw,
            'waste_targets_by_raw': waste_targets_by_raw,
            'material_model': material_model,
        }

    def _assign_sources(self, orders, station_info):
        produce_orders = [o for o in orders if o['order_type'] == Order.OT_PRODUCE]
        recycle_orders = [o for o in orders if o['order_type'] == Order.OT_RECYCLE]

        produce_need = []
        for po in produce_orders:
            produce_need.extend(po['materials'])

        recycle_orders_for_reuse = sorted(
            recycle_orders,
            key=lambda ro: self._multiset_overlap_count(ro['materials'], produce_need),
            reverse=True,
        )

        recycle_available = defaultdict(list)
        for ro in recycle_orders_for_reuse:
            for raw in ro['materials']:
                recycle_available[int(raw)].append(ro)

        stock_by_raw = station_info['stock_by_raw']

        for po in produce_orders:
            po['sources'] = []
            for material_index, raw in enumerate(po['materials']):
                raw = int(raw)
                source = None

                # Lifecycle reuse has priority over stock.
                while recycle_available[raw]:
                    candidate = recycle_available[raw].pop(0)
                    if candidate['order_index'] == po['order_index']:
                        continue
                    source = {
                        'kind': 'recycle',
                        'raw': raw,
                        'station_id': station_info['wb_id'],
                        'recycle_order': candidate,
                    }
                    candidate['used_by'].append({
                        'produce_order_index': po['order_index'],
                        'produce_product_id': po['product_id'],
                        'raw': raw,
                    })
                    candidate['reuse_materials'].append(raw)
                    break

                if source is None:
                    token = self._take_stock(raw, stock_by_raw)
                    if token is None:
                        raise RuntimeError(
                            f'PRODUCE {po["product_id"]}에 필요한 raw {raw}를 구할 수 없습니다'
                        )
                    source = {
                        'kind': 'stock',
                        'raw': raw,
                        'station_id': int(token['station_id']),
                        'object_id': int(token['object_id']),
                    }

                source['material_index'] = int(material_index)
                po['sources'].append(source)

        for ro in recycle_orders:
            remaining = list(ro['materials'])
            for raw in ro['reuse_materials']:
                if raw in remaining:
                    remaining.remove(raw)
            ro['waste_materials'] = remaining

    # ------------------------------------------------------------------
    # Step generation
    # ------------------------------------------------------------------


    # ------------------------------------------------------------------
    # Inventory item helpers
    # ------------------------------------------------------------------

    def _make_item(self, object_id, slot_index, slide_id, role,
                   order_index=None, order_type=None, product_id=None,
                   object_kind='raw', source_station=None, target_station=None,
                   size=None, material_index=None, note=''):
        """Create a rich planner-side item for one object/slot assignment."""
        if size is None:
            size = self._raw_size(object_id) if 1 <= int(object_id) <= 8 else 1
        return InventoryItem(
            object_id=int(object_id),
            slot_index=int(slot_index),
            slide_id=int(slide_id),
            role=str(role),
            object_kind=str(object_kind),
            order_index=order_index,
            order_type=order_type,
            product_id=product_id,
            source_station=source_station,
            target_station=target_station,
            size=int(size),
            material_index=material_index,
            note=str(note or ''),
        )

    def _item_from_legacy(self, legacy, role=None, order=None, object_kind='raw'):
        """Convert an old dict-shaped load/return item into InventoryItem."""
        if isinstance(legacy, InventoryItem):
            return legacy
        if 'item' in legacy and isinstance(legacy['item'], InventoryItem):
            return legacy['item']

        order_index = None
        order_type = None
        product_id = None
        if order is not None:
            order_index = int(order.get('order_index'))
            order_type = 'produce' if int(order.get('order_type', 0)) == int(Order.OT_PRODUCE) else 'recycle'
            product_id = int(order.get('product_id'))

        slide_id = int(legacy['slide_id'])
        slot_index = abs(slide_id) % 10
        return self._make_item(
            object_id=int(legacy.get('object_id', legacy.get('raw'))),
            slot_index=slot_index,
            slide_id=slide_id,
            role=role or legacy.get('role', 'raw'),
            order_index=legacy.get('order_index', order_index),
            order_type=legacy.get('order_type', order_type),
            product_id=legacy.get('product_id', product_id),
            object_kind=legacy.get('object_kind', object_kind),
            source_station=legacy.get('station_id', legacy.get('source_station')),
            target_station=legacy.get('target_station'),
            size=legacy.get('size'),
            material_index=legacy.get('material_index'),
            note=legacy.get('note', ''),
        )

    def _items_to_command(self, items):
        return command_from_items(items)

    def _add_step_from_items(self, type_, action, items, station_id, depends_on,
                             validate_slide_len=True):
        item_list = list(items or [])
        object_ids, slide_ids = self._items_to_command(item_list)
        sid = self._add_step(
            type_, action, object_ids, station_id, depends_on, slide_ids,
            validate_slide_len=validate_slide_len,
        )
        if item_list:
            self.step_items[int(sid)] = item_list
            self.get_logger().info(f'[AMR ITEM step {sid}] {describe_items(item_list)}')
        return sid

    def _append_amr_assemble_batches(self, produce_orders, station_info):
        if not produce_orders:
            return

        for i in range(0, len(produce_orders), len(ASSEMBLY_SLOT_INDICES)):
            batch = produce_orders[i:i + len(ASSEMBLY_SLOT_INDICES)]
            self._append_amr_assemble_batch(batch, station_info)

    def _append_amr_assemble_batch(self, batch, station_info):
        load_items = []
        order_runtime = []

        raw_slot_capacity_used = {slot: 0 for slot in RAW_SLOT_INDICES}
        raw_slot_orders = {slot: set() for slot in RAW_SLOT_INDICES}

        for batch_index, po in enumerate(batch):
            if int(po['product_id']) not in AMR_CAPABLE_PRODUCTS:
                raise RuntimeError(f'PRODUCE {po["product_id"]}: AMR 조립 가능 제품이 아닙니다')

            order_index = int(po['order_index'])
            materials = list(po['materials'])
            if not materials:
                raise RuntimeError(f'PRODUCE {po["product_id"]}: material list가 비어 있습니다')

            assembly_slot = ASSEMBLY_SLOT_INDICES[batch_index]
            assembly_slide = self._encode_order_slide(order_index, assembly_slot)
            used_slides = [assembly_slide]

            # Base material goes directly to assembly slot 7 or 8.
            base_source = po['sources'][0]
            base_item = self._make_item(
                object_id=int(materials[0]),
                slot_index=assembly_slot,
                slide_id=assembly_slide,
                role=ROLE_AMR_ASSEMBLE_BASE,
                order_index=order_index,
                order_type='produce',
                product_id=int(po['product_id']),
                object_kind='raw',
                source_station=int(base_source['station_id']),
                material_index=0,
            )
            load_items.append({
                'raw': int(materials[0]),
                'object_id': int(materials[0]),
                'station_id': int(base_source['station_id']),
                'slide_id': assembly_slide,
                'depends_on': self._source_depends(base_source),
                'item': base_item,
            })

            # Upper materials go to raw slides. The same order cannot use the same
            # physical raw slide twice. Different orders may share a physical slide
            # while respecting capacity 3 units.
            for source in po['sources'][1:]:
                slot_index = self._allocate_raw_slot_for_order(
                    order_index,
                    int(source['raw']),
                    raw_slot_capacity_used,
                    raw_slot_orders,
                )
                slide_id = self._encode_order_slide(order_index, slot_index)
                used_slides.append(slide_id)
                upper_item = self._make_item(
                    object_id=int(source['raw']),
                    slot_index=slot_index,
                    slide_id=slide_id,
                    role=ROLE_AMR_ASSEMBLE_UPPER,
                    order_index=order_index,
                    order_type='produce',
                    product_id=int(po['product_id']),
                    object_kind='raw',
                    source_station=int(source['station_id']),
                    material_index=int(source.get('material_index', len(used_slides) - 1)),
                )
                load_items.append({
                    'raw': int(source['raw']),
                    'object_id': int(source['raw']),
                    'station_id': int(source['station_id']),
                    'slide_id': slide_id,
                    'depends_on': self._source_depends(source),
                    'item': upper_item,
                })

            order_runtime.append({
                'order': po,
                'assembly_slide': assembly_slide,
                'used_slides': used_slides,
            })

        load_step_ids = self._append_grouped_load_steps(load_items)

        produce_depends = list(load_step_ids)
        deferred_wb_delivery = self.deferred_wb_product_delivery
        if deferred_wb_delivery is not None:
            produce_depends.append(int(deferred_wb_delivery['load_step_id']))

        product_ids = []
        produce_slides = []
        product_items = []
        for runtime in order_runtime:
            po = runtime['order']
            product_ids.append(int(po['product_id']))
            produce_slides.extend(int(s) for s in runtime['used_slides'])
            product_item = self._make_item(
                object_id=int(po['product_id']),
                slot_index=int(runtime['assembly_slide']) % 10,
                slide_id=int(runtime['assembly_slide']),
                role=ROLE_AMR_ASSEMBLE_PRODUCT,
                order_index=int(po['order_index']),
                order_type='produce',
                product_id=int(po['product_id']),
                object_kind='product',
                target_station=int(station_info['customer_id']),
            )
            product_items.append(product_item)

        assemble_sid = self._add_step(
            Step.AMR,
            Step.PRODUCE,
            product_ids,
            station_info['customer_id'],
            produce_depends,
            produce_slides,
            validate_slide_len=False,
        )
        self.step_items[int(assemble_sid)] = list(product_items)
        self.get_logger().info(
            f'[AMR batch ASSEMBLE] products={product_ids}, slides={produce_slides}, '
            f'depends_on={self._unique_ints(produce_depends)}'
        )

        unload_items = list(product_items)
        unload_depends = [int(assemble_sid)]
        if deferred_wb_delivery is not None:
            unload_items.insert(0, deferred_wb_delivery['item'])
            unload_depends.append(int(deferred_wb_delivery['load_step_id']))
            self.deferred_wb_product_delivery = None

        self._add_step_from_items(
            Step.AMR,
            Step.UNLOAD,
            unload_items,
            station_info['customer_id'],
            unload_depends,
        )

    def _append_wb_produce(self, po, station_info, defer_customer_delivery=False):
        wb_id = station_info['wb_id']
        customer_id = station_info['customer_id']
        order_index = int(po['order_index'])

        load_items = []
        wb_depends = []
        used_raw_slots = set()

        for source in po['sources']:
            if source['kind'] == 'recycle':
                wb_depends.extend(self._source_depends(source))
                continue

            slot_index = self._allocate_distinct_raw_slot(used_raw_slots, source['raw'])
            slide_id = self._encode_order_slide(order_index, slot_index)
            wb_raw_item = self._make_item(
                object_id=int(source['raw']),
                slot_index=slot_index,
                slide_id=slide_id,
                role=ROLE_PRODUCE_WB_RAW,
                order_index=order_index,
                order_type='produce',
                product_id=int(po['product_id']),
                object_kind='raw',
                source_station=int(source['station_id']),
                material_index=int(source.get('material_index', len(load_items))),
            )
            load_items.append({
                'raw': int(source['raw']),
                'object_id': int(source['raw']),
                'station_id': int(source['station_id']),
                'slide_id': slide_id,
                'depends_on': [],
                'item': wb_raw_item,
            })

        load_step_ids = self._append_grouped_load_steps(load_items)
        if load_step_ids:
            unload_depends = list(load_step_ids)
            if self.last_wb_clear_step_id is not None:
                unload_depends.append(self.last_wb_clear_step_id)
            unload_sid = self._add_step_from_items(
                Step.AMR,
                Step.UNLOAD,
                [self._item_from_legacy(item, ROLE_PRODUCE_WB_RAW, po) for item in load_items],
                wb_id,
                unload_depends,
            )
            wb_depends.append(unload_sid)
        elif self.last_wb_clear_step_id is not None:
            wb_depends.append(self.last_wb_clear_step_id)

        wb_sid = self._add_step(
            Step.WB,
            Step.PRODUCE,
            [po['product_id']],
            wb_id,
            wb_depends,
            [],
            validate_slide_len=False,
        )

        product_slide = self._encode_order_slide(order_index, PRODUCT_SLOT_INDEX)
        wb_product_item = self._make_item(
            object_id=int(po['product_id']),
            slot_index=PRODUCT_SLOT_INDEX,
            slide_id=product_slide,
            role=ROLE_WB_PRODUCT,
            order_index=order_index,
            order_type='produce',
            product_id=int(po['product_id']),
            object_kind='product',
            source_station=wb_id,
            target_station=customer_id,
        )
        product_load_sid = self._add_step_from_items(
            Step.AMR,
            Step.LOAD,
            [wb_product_item],
            wb_id,
            [wb_sid],
        )
        self.last_wb_clear_step_id = product_load_sid

        if defer_customer_delivery:
            self.deferred_wb_product_delivery = {
                'item': wb_product_item,
                'load_step_id': int(product_load_sid),
            }
            self.get_logger().info(
                f'[WB product merge] product={po["product_id"]}, '
                f'load_step={product_load_sid}: 다음 AMR 생산품과 Customer 공동 하역'
            )
            return

        self._add_step_from_items(
            Step.AMR,
            Step.UNLOAD,
            [wb_product_item],
            customer_id,
            [product_load_sid],
        )

    def _append_recycle(self, ro, station_info):
        """Append one recycle job using a WB-continuous AMR pre-load pipeline.

        Scheduling intent:
          * AMR may LOAD the next recycle product from CUSTOMER while the WB is
            still recycling the previous product.
          * AMR may not UNLOAD to the WB until the previous WB job has finished.
          * Recycle products always use physical AMR product slot 1.

        This creates the practical pipeline:
            LOAD product i+1 from CUSTOMER  ||  WB RECYCLE product i
            UNLOAD product i+1 to WB        after WB RECYCLE product i
        """
        wb_id = station_info['wb_id']
        customer_id = station_info['customer_id']
        order_index = int(ro['order_index'])

        slot_index = self._choose_recycle_product_slot(ro)
        slide_id = self._encode_order_slide(order_index, slot_index)

        load_depends = []

        # Keep AMR product transport as a one-step lookahead pipeline.  After the
        # previous product is dropped at WB, the AMR is free to go to CUSTOMER and
        # transport the next product while the WB is working.
        if self.last_recycle_unload_to_wb_step_id is not None:
            load_depends.append(self.last_recycle_unload_to_wb_step_id)

        # Do not reuse physical slot 1 until the previous product occupying it
        # has been unloaded to WB.
        slot_free_after = self.recycle_product_slot_free_after.get(int(slot_index))
        if slot_free_after is not None:
            load_depends.append(int(slot_free_after))

        load_sid = self._add_step(
            Step.AMR,
            Step.LOAD,
            [ro['product_id']],
            customer_id,
            load_depends,
            [slide_id],
        )

        unload_depends = [load_sid]
        if self.last_wb_clear_step_id is not None:
            # AMR-WB interaction is only allowed after the previous WB job clears.
            unload_depends.append(self.last_wb_clear_step_id)

        unload_sid = self._add_step(
            Step.AMR,
            Step.UNLOAD,
            [ro['product_id']],
            wb_id,
            unload_depends,
            [slide_id],
        )

        recycle_sid = self._add_step(
            Step.WB,
            Step.RECYCLE,
            [ro['product_id']],
            wb_id,
            [unload_sid],
            [],
            validate_slide_len=False,
        )

        self.recycle_step_by_order_index[order_index] = recycle_sid

        # WB becomes unavailable until the recycle action finishes.
        self.last_wb_clear_step_id = recycle_sid

        # The product slot is freed as soon as the product is unloaded to WB, not
        # after WB recycling.  This lets AMR reuse the same physical slot while WB
        # is processing, as long as the AMR does not touch the WB.
        self.recycle_product_slot_free_after[int(slot_index)] = unload_sid
        self.last_recycle_unload_to_wb_step_id = unload_sid

        self.get_logger().info(
            f'[RECYCLE pipeline] order={order_index}, product={ro["product_id"]}, '
            f'slot={slot_index}, slide_id={slide_id}, '
            f'load_depends={self._unique_ints(load_depends)}, '
            f'unload_depends={self._unique_ints(unload_depends)}'
        )

        if ro['waste_materials']:
            self.deferred_waste_jobs.append({
                'order': ro,
                'recycle_step_id': recycle_sid,
                'materials': list(ro['waste_materials']),
            })

    def _choose_recycle_product_slot(self, ro):
        """Return the only physical slot allowed for a recycle product."""
        return int(PRODUCT_SLOT_INDEX)

    def _append_recycle_interleaved_group(self, recycle_orders, station_info):
        """Plan standalone recycle orders as an interleaved WB/AMR pipeline.

        This is the recycling counterpart to the production-side parallelism:
        recycling itself is WB-only, so the planner keeps the WB busy by using
        AMR time during WB processing for two jobs:

          1. return the raw materials generated by the previous recycle job;
          2. transport the next recycle product from CUSTOMER.

        Product transport rule:
          * only one recycle product may be carried at a time;
          * every recycle product uses physical product slot 1.

        WB interaction rule:
          * AMR never interacts with WB while WB is processing;
          * at each WB exchange point, AMR loads the previous raw results first,
            then unloads the next recycle product to WB, then the WB starts the
            next recycle action.
        """
        if not recycle_orders:
            return

        jobs = self._prepare_interleaved_recycle_jobs(recycle_orders, station_info)
        if not jobs:
            return

        jobs = self._order_recycle_jobs_by_return_overlap(jobs)
        self.get_logger().info('===== interleaved recycle order =====')
        for idx, job in enumerate(jobs):
            self.get_logger().info(
                f'{idx + 1}. order={job["order_index"]}, product={job["product_id"]}, '
                f'return_targets={dict(job["target_counts"])}, '
                f'wb_time={self._estimate_recycle_job_time(job):.2f}s'
            )
        self.get_logger().info('====================================')

        loaded_queue = []
        next_index = 0
        last_amr_step = None

        # Initial load contains exactly one recycle product.
        next_index, last_amr_step = self._load_next_recycle_product(
            jobs,
            next_index,
            loaded_queue,
            station_info,
            last_amr_step=last_amr_step,
            current_wb_job=None,
            initial=True,
        )
        if not loaded_queue:
            raise RuntimeError('RECYCLE 계획 오류: customer에서 운반할 recycle product가 없습니다')

        current_job = loaded_queue.pop(0)
        current_unload_sid = self._append_recycle_product_unload_to_wb(
            current_job,
            station_info,
            depends_on=self._deps([last_amr_step, self.last_wb_clear_step_id]),
        )
        current_wb_sid = self._append_recycle_wb_step(current_job, station_info, current_unload_sid)

        last_amr_step = current_unload_sid
        self.last_wb_clear_step_id = current_wb_sid
        self.last_recycle_unload_to_wb_step_id = current_unload_sid

        while True:
            # While the current WB job is running, transport exactly one next
            # product if no product is already waiting on the AMR.
            if not loaded_queue and next_index < len(jobs):
                next_index, last_amr_step = self._load_next_recycle_product(
                    jobs,
                    next_index,
                    loaded_queue,
                    station_info,
                    last_amr_step=last_amr_step,
                    current_wb_job=current_job,
                    initial=False,
                )

            if loaded_queue:
                next_job = loaded_queue.pop(0)

                # WB exchange point: previous recycle is done, AMR is ready with
                # the next product.  Load previous raw results from WB first, then
                # put the next product into WB.
                raw_load_sid = self._append_recycle_raw_load_from_wb(
                    current_job,
                    station_info,
                    depends_on=self._deps([current_wb_sid, last_amr_step]),
                )
                exchange_dep = raw_load_sid if raw_load_sid is not None else current_wb_sid
                if raw_load_sid is None and last_amr_step is not None:
                    # No raw to load, but AMR must still be at the state represented
                    # by last_amr_step before it can unload the next product.
                    exchange_dep = [current_wb_sid, last_amr_step]

                next_unload_sid = self._append_recycle_product_unload_to_wb(
                    next_job,
                    station_info,
                    depends_on=self._deps([exchange_dep]),
                )
                next_wb_sid = self._append_recycle_wb_step(next_job, station_info, next_unload_sid)

                # While the next WB recycle is running, return the raw produced by
                # the previous recycle job.
                last_amr_step = self._append_recycle_raw_returns(
                    current_job,
                    station_info,
                    depends_on=[next_unload_sid],
                    final_return=False,
                ) or next_unload_sid

                self.last_wb_clear_step_id = next_wb_sid
                self.last_recycle_unload_to_wb_step_id = next_unload_sid
                current_job = next_job
                current_wb_sid = next_wb_sid
                continue

            # No next product remains.  After the last WB recycle finishes, collect
            # and return its raw results, then the normal GOAL step can be appended
            # by build_plan().
            raw_load_sid = self._append_recycle_raw_load_from_wb(
                current_job,
                station_info,
                depends_on=self._deps([current_wb_sid, last_amr_step]),
            )
            if raw_load_sid is not None:
                last_amr_step = self._append_recycle_raw_returns(
                    current_job,
                    station_info,
                    depends_on=[raw_load_sid],
                    final_return=True,
                ) or raw_load_sid
            else:
                last_amr_step = current_wb_sid

            self.last_wb_clear_step_id = last_amr_step
            break

    def _prepare_interleaved_recycle_jobs(self, recycle_orders, station_info):
        jobs = []
        for ro in recycle_orders:
            return_items = []
            for raw in list(ro.get('waste_materials', [])):
                raw = int(raw)
                target_station = int(self._take_waste_target(raw, station_info))
                return_items.append({
                    'raw': raw,
                    'target_station': target_station,
                    'size': self._raw_size(raw),
                })

            target_counts = Counter(int(item['target_station']) for item in return_items)
            jobs.append({
                'order': ro,
                'order_index': int(ro['order_index']),
                'product_id': int(ro['product_id']),
                'materials': list(ro['materials']),
                'return_items': return_items,
                'target_counts': target_counts,
                'slot_index': None,
                'slide_id': None,
                'product_load_step_id': None,
                'product_unload_step_id': None,
                'wb_step_id': None,
            })
        return jobs

    def _order_recycle_jobs_by_return_overlap(self, jobs):
        """Greedily cluster recycle jobs whose return stations overlap.

        The goal is to make the AMR repeatedly visit similar return stations while
        the WB is processing the next product.  Ties prefer longer WB jobs because
        they provide more AMR time to return raw and transport the next product.
        """
        remaining = list(jobs)
        ordered = []
        active_targets = Counter()

        while remaining:
            if not ordered:
                idx = max(
                    range(len(remaining)),
                    key=lambda i: (
                        sum(remaining[i]['target_counts'].values()),
                        self._estimate_recycle_job_time(remaining[i]),
                        -remaining[i]['order_index'],
                    ),
                )
            else:
                idx = max(
                    range(len(remaining)),
                    key=lambda i: (
                        self._target_overlap_count(active_targets, remaining[i]['target_counts']),
                        self._estimate_recycle_job_time(remaining[i]),
                        -remaining[i]['order_index'],
                    ),
                )
            job = remaining.pop(idx)
            ordered.append(job)
            active_targets.update(job['target_counts'])

        return ordered

    def _target_overlap_count(self, left_counts, right_counts):
        return sum(min(int(left_counts[k]), int(right_counts[k])) for k in right_counts)

    def _estimate_recycle_job_time(self, job):
        return self._product_connection_count(job['product_id']) * float(
            self.config.wb_recycle_time_sec_per_connection
        )

    def _load_next_recycle_product(self, jobs, start_index, loaded_queue, station_info,
                                   last_amr_step, current_wb_job=None, initial=False):
        """Load exactly one recycle product from CUSTOMER into slot 1."""
        if start_index >= len(jobs):
            return start_index, last_amr_step

        if loaded_queue:
            # The one-product capacity is already occupied.
            return start_index, last_amr_step

        job = jobs[start_index]
        job['slot_index'] = int(PRODUCT_SLOT_INDEX)
        job['slide_id'] = self._encode_order_slide(
            job['order_index'], PRODUCT_SLOT_INDEX
        )

        depends = [] if last_amr_step is None else [int(last_amr_step)]
        item = self._make_item(
            object_id=int(job['product_id']),
            slot_index=int(PRODUCT_SLOT_INDEX),
            slide_id=int(job['slide_id']),
            role=ROLE_RECYCLE_PRODUCT,
            order_index=int(job['order_index']),
            order_type='recycle',
            product_id=int(job['product_id']),
            object_kind='product',
            source_station=int(station_info['customer_id']),
            target_station=int(station_info['wb_id']),
        )
        job['product_item'] = item

        objects, slides = self._items_to_command([item])
        load_sid = self._add_step_from_items(
            Step.AMR,
            Step.LOAD,
            [item],
            station_info['customer_id'],
            depends,
        )
        job['product_load_step_id'] = int(load_sid)
        loaded_queue.append(job)

        self.get_logger().info(
            f'[RECYCLE product load] objects={objects}, slots={[PRODUCT_SLOT_INDEX]}, '
            f'slides={slides}, depends_on={self._unique_ints(depends)}, initial={initial}'
        )
        return start_index + 1, int(load_sid)

    def _append_recycle_product_unload_to_wb(self, job, station_info, depends_on):
        item = job.get('product_item')
        if item is None:
            item = self._make_item(
                object_id=int(job['product_id']),
                slot_index=int(job['slot_index']),
                slide_id=int(job['slide_id']),
                role=ROLE_RECYCLE_PRODUCT,
                order_index=int(job['order_index']),
                order_type='recycle',
                product_id=int(job['product_id']),
                object_kind='product',
                target_station=int(station_info['wb_id']),
            )
            job['product_item'] = item
        unload_sid = self._add_step_from_items(
            Step.AMR,
            Step.UNLOAD,
            [item],
            station_info['wb_id'],
            depends_on,
        )
        job['product_unload_step_id'] = int(unload_sid)
        return int(unload_sid)

    def _append_recycle_wb_step(self, job, station_info, unload_sid):
        wb_sid = self._add_step(
            Step.WB,
            Step.RECYCLE,
            [job['product_id']],
            station_info['wb_id'],
            [unload_sid],
            [],
            validate_slide_len=False,
        )
        job['wb_step_id'] = int(wb_sid)
        self.recycle_step_by_order_index[int(job['order_index'])] = int(wb_sid)
        self.get_logger().info(
            f'[RECYCLE exchange] order={job["order_index"]}, product={job["product_id"]}, '
            f'slot={job["slot_index"]}, slide_id={job["slide_id"]}, '
            f'unload_step={unload_sid}, wb_step={wb_sid}'
        )
        return int(wb_sid)

    def _append_recycle_raw_load_from_wb(self, job, station_info, depends_on):
        items = job.get('return_items', [])
        if not items:
            return None

        self._assign_return_slots(items)
        inv_items = []
        for material_index, item in enumerate(items):
            inv_item = item.get('item')
            if inv_item is None:
                inv_item = self._make_item(
                    object_id=int(item['raw']),
                    slot_index=int(item['slot_index']),
                    slide_id=int(item['slide_id']),
                    role=ROLE_RETURN_RAW,
                    order_index=int(job['order_index']),
                    order_type='recycle',
                    product_id=int(job['product_id']),
                    object_kind='raw',
                    source_station=int(station_info['wb_id']),
                    target_station=int(item['target_station']),
                    size=int(item.get('size', self._raw_size(item['raw']))),
                    material_index=material_index,
                )
                item['item'] = inv_item
            inv_items.append(inv_item)
        objects, slides = self._items_to_command(inv_items)
        load_sid = self._add_step_from_items(
            Step.AMR,
            Step.LOAD,
            inv_items,
            station_info['wb_id'],
            depends_on,
        )
        job['raw_load_step_id'] = int(load_sid)
        self.get_logger().info(
            f'[RECYCLE raw pickup] product={job["product_id"]}, objects={objects}, '
            f'slides={slides}, depends_on={self._unique_ints(depends_on)}'
        )
        return int(load_sid)

    def _append_recycle_raw_returns(self, job, station_info, depends_on, final_return=False):
        items = job.get('return_items', [])
        if not items:
            return None

        grouped = defaultdict(list)
        for item in items:
            grouped[int(item['target_station'])].append(item)

        station_order = self._order_return_stations(
            list(grouped.keys()),
            station_info,
            final_trip=bool(final_return),
        )
        self.get_logger().info(
            f'[RECYCLE raw return] product={job["product_id"]}, '
            f'route={station_order}, final={final_return}'
        )

        prev_step = depends_on[0] if len(depends_on) == 1 else None
        if prev_step is None:
            # Create a dependency-only no-op is unnecessary; just use all deps on
            # the first unload.
            prev_deps = self._deps(depends_on)
        else:
            prev_deps = [int(prev_step)]

        last_unload = None
        first = True
        for target_station in station_order:
            group_items = grouped[int(target_station)]
            inv_items = [item.get('item') or self._item_from_legacy(item, ROLE_RETURN_RAW) for item in group_items]
            deps = prev_deps if first else [int(last_unload)]
            last_unload = self._add_step_from_items(
                Step.AMR,
                Step.UNLOAD,
                inv_items,
                int(target_station),
                deps,
            )
            first = False

        return int(last_unload) if last_unload is not None else None

    def _deps(self, values):
        out = []
        for value in values or []:
            if value is None:
                continue
            if isinstance(value, (list, tuple, set)):
                out.extend(int(v) for v in value if v is not None)
            else:
                out.append(int(value))
        return self._unique_ints(out)

    def _append_deferred_waste_returns(self, station_info):
        """Return leftover recycle materials with minimum WB round-trips.

        Strategy used here:
        1. Collect all leftover recycle results first.
        2. Assign each raw material to a return target station.
        3. Pack as many materials as possible into each AMR trip using raw slides 2~6
           and slide capacity 3 units.
        4. If everything fits in one trip, visit stations in the order:
              close to WB and far from GOAL  ->  close to GOAL
           so the last unload is near the final home/goal.
        5. If everything does not fit, middle trips consume WB-near/GOAL-far
           inventory first, so the final trip can carry all remaining materials
           and finish near GOAL.
        """
        return_items = []

        for job in self.deferred_waste_jobs:
            recycle_step_id = int(job['recycle_step_id'])
            for raw in list(job.get('materials', [])):
                raw = int(raw)
                target_station = int(self._take_waste_target(raw, station_info))
                return_items.append({
                    'raw': raw,
                    'target_station': target_station,
                    'source_dep': recycle_step_id,
                    'size': self._raw_size(raw),
                })

        if not return_items:
            return

        trips = self._build_return_trips(return_items, station_info)
        if not trips:
            return

        self.get_logger().info('===== recycle return packing =====')
        self.get_logger().info(
            f'return_items={len(return_items)}, total_size={sum(i["size"] for i in return_items)}, '
            f'trips={len(trips)}'
        )

        prev_trip_last_unload = None
        wb_id = int(station_info['wb_id'])

        for trip_index, trip_items in enumerate(trips):
            final_trip = (trip_index == len(trips) - 1)
            self._assign_return_slots(trip_items)

            objects = [int(item['raw']) for item in trip_items]
            slides = [int(item['slide_id']) for item in trip_items]

            load_depends = [int(item['source_dep']) for item in trip_items]
            if self.last_wb_clear_step_id is not None:
                load_depends.append(int(self.last_wb_clear_step_id))
            if prev_trip_last_unload is not None:
                # Prevent the next WB LOAD from starting while the AMR is still
                # unloading the previous return trip.
                load_depends.append(int(prev_trip_last_unload))

            inv_items = []
            for material_index, item in enumerate(trip_items):
                inv_item = item.get('item')
                if inv_item is None:
                    inv_item = self._make_item(
                        object_id=int(item['raw']),
                        slot_index=int(item['slot_index']),
                        slide_id=int(item['slide_id']),
                        role=ROLE_RETURN_RAW,
                        order_index=None,
                        order_type='recycle',
                        product_id=None,
                        object_kind='raw',
                        source_station=wb_id,
                        target_station=int(item['target_station']),
                        size=int(item.get('size', self._raw_size(item['raw']))),
                        material_index=material_index,
                    )
                    item['item'] = inv_item
                inv_items.append(inv_item)

            load_sid = self._add_step_from_items(
                Step.AMR,
                Step.LOAD,
                inv_items,
                wb_id,
                load_depends,
            )

            grouped = defaultdict(list)
            for item in trip_items:
                grouped[int(item['target_station'])].append(item)

            station_order = self._order_return_stations(
                list(grouped.keys()),
                station_info,
                final_trip=final_trip,
            )

            self.get_logger().info(
                f'[RETURN trip {trip_index + 1}/{len(trips)}] '
                f'load_size={sum(i["size"] for i in trip_items)}, '
                f'objects={objects}, slides={slides}, '
                f'route={station_order}, final_trip={final_trip}'
            )

            prev_step = load_sid
            for target_station in station_order:
                items = grouped[int(target_station)]
                inv_items = [item.get('item') or self._item_from_legacy(item, ROLE_RETURN_RAW) for item in items]
                prev_step = self._add_step_from_items(
                    Step.AMR,
                    Step.UNLOAD,
                    inv_items,
                    int(target_station),
                    [prev_step],
                )

            prev_trip_last_unload = prev_step
            # For future WB interactions, wait until the current return trip is done.
            self.last_wb_clear_step_id = int(prev_trip_last_unload)

        self.get_logger().info('==================================')

    def _build_return_trips(self, return_items, station_info):
        """Pack return items into the fewest practical AMR trips.

        The first/middle trips consume stations that are near WB and far from GOAL.
        The final trip keeps GOAL-near stations, so the plan can finish close to home.
        """
        items = [dict(item) for item in return_items]
        if self._pack_return_slots(items) is not None:
            return [items]

        remaining = sorted(
            items,
            key=lambda item: (
                self._return_station_score(item['target_station'], station_info),
                self._travel_time(station_info['wb_id'], item['target_station']),
                -int(item['size']),
                int(item['raw']),
            ),
        )

        trips = []
        while remaining:
            if self._pack_return_slots(remaining) is not None:
                trips.append(list(remaining))
                break

            selected, remaining = self._select_middle_return_trip(remaining, station_info)
            if not selected:
                raise RuntimeError(
                    'recycle 반환 packing 실패: AMR raw slide에 적재 가능한 항목을 선택하지 못했습니다'
                )
            trips.append(selected)

        return trips

    def _select_middle_return_trip(self, remaining, station_info):
        """Select one full middle trip while leaving GOAL-near items for the last trip."""
        selected = []
        rest = []

        # Try WB-near/GOAL-far materials first.  Keep adding while the whole trip
        # remains physically packable in raw slides 2~6.
        ordered = sorted(
            remaining,
            key=lambda item: (
                self._return_station_score(item['target_station'], station_info),
                self._travel_time(station_info['wb_id'], item['target_station']),
                -int(item['size']),
                int(item['raw']),
            ),
        )

        for item in ordered:
            candidate = selected + [item]
            if self._pack_return_slots(candidate) is not None:
                selected.append(item)
            else:
                rest.append(item)

        # Keep the original dict objects, but put the rest back in the same priority
        # order for the next trip.
        return selected, rest

    def _assign_return_slots(self, trip_items):
        assignments = self._pack_return_slots(trip_items)
        if assignments is None:
            raise RuntimeError(
                f'recycle 반환 packing 실패: trip_items={trip_items}'
            )

        for item, slot_index in zip(trip_items, assignments):
            item['slot_index'] = int(slot_index)
            # Negative slide_id encodes the return target station and physical raw slide.
            local_station = self._local_station_id_for_slide(item['target_station'])
            item['slide_id'] = self._encode_return_slide(local_station, slot_index)

    def _pack_return_slots(self, items):
        """Return slot assignment for items, or None if the items do not fit.

        Each raw slide has RAW_SLIDE_CAPACITY_UNITS capacity.  raw 1~4 uses one
        unit, raw 5~8 uses two units.  Multiple return objects may share the same
        physical raw slide as long as the capacity is respected.
        """
        if not items:
            return []

        capacity_used = {int(slot): 0 for slot in RAW_SLOT_INDICES}
        assignments = [None] * len(items)

        # Put large raw materials first to avoid leaving unusable capacity holes.
        order = sorted(
            range(len(items)),
            key=lambda i: (-int(items[i].get('size', self._raw_size(items[i]['raw']))), i),
        )

        for item_index in order:
            item = items[item_index]
            size = int(item.get('size', self._raw_size(item['raw'])))
            candidates = [
                int(slot) for slot in RAW_SLOT_INDICES
                if capacity_used[int(slot)] + size <= RAW_SLIDE_CAPACITY_UNITS
            ]
            if not candidates:
                return None

            # Use the most-filled feasible slide first.  This preserves empty slides
            # and usually reduces the number of active physical slides.
            slot_index = max(candidates, key=lambda slot: capacity_used[slot])
            capacity_used[slot_index] += size
            assignments[item_index] = int(slot_index)

        return assignments

    def _order_return_stations(self, station_ids, station_info, final_trip):
        station_ids = [int(sid) for sid in station_ids]
        if not station_ids:
            return []

        if final_trip:
            # WB close + GOAL far first, GOAL close last.
            return sorted(
                station_ids,
                key=lambda sid: (
                    self._return_station_score(sid, station_info),
                    self._travel_time(station_info['wb_id'], sid),
                    sid,
                ),
            )

        # A middle trip must come back to WB for the next load.  Visit farther
        # stations first and finish near WB to reduce the return-to-WB leg.
        return sorted(
            station_ids,
            key=lambda sid: (
                -self._travel_time(station_info['wb_id'], sid),
                self._travel_time(sid, STATION_START_GOAL),
                sid,
            ),
        )

    def _return_station_score(self, station_id, station_info):
        """Low score means good to process early: near WB, far from GOAL."""
        station_id = int(station_id)
        return (
            self._travel_time(station_info['wb_id'], station_id)
            - self._travel_time(station_id, STATION_START_GOAL)
        )

    def _local_station_id_for_slide(self, station_id):
        """Use local 1~8 station number inside negative return slide IDs.

        A side stations are already 1~8.  B side stations are 10~17, so the local
        station number is station_id - 9.
        """
        station_id = int(station_id)
        if 10 <= station_id <= 17:
            return station_id - 9
        return station_id

    def _append_grouped_load_steps(self, load_items):
        if not load_items:
            return []

        grouped = defaultdict(list)
        for item in load_items:
            grouped[int(item['station_id'])].append(item)

        load_step_ids = []
        prev_load = None
        for station_id, items in self._order_grouped_by_travel(grouped).items():
            depends = []
            for item in items:
                depends.extend(item.get('depends_on', []))
            if prev_load is not None:
                depends.append(prev_load)

            # If this LOAD is at the fixed WB, do not interact while WB is active.
            if int(station_id) == int(self.config.fixed_workbench_station_id) \
                    and self.last_wb_clear_step_id is not None:
                depends.append(self.last_wb_clear_step_id)

            inv_items = [self._item_from_legacy(item) for item in items]
            sid = self._add_step_from_items(
                Step.AMR,
                Step.LOAD,
                inv_items,
                station_id,
                depends,
            )
            load_step_ids.append(sid)
            prev_load = sid
        return load_step_ids

    def _add_step(self, type_, action, object_ids, station_id, depends_on, slide_ids,
                  validate_slide_len=True):
        object_ids = [int(x) for x in object_ids]
        slide_ids = [int(x) for x in slide_ids]

        if validate_slide_len and action in (Step.LOAD, Step.UNLOAD):
            if len(object_ids) != len(slide_ids):
                raise RuntimeError(
                    'Plan D step 생성 오류: LOAD/UNLOAD object_ids와 slide_ids 길이가 다름 '
                    f'objects={object_ids}, slide_ids={slide_ids}'
                )

        step = Step()
        step.step_id = int(self.step_id)
        step.type = int(type_)
        step.action = int(action)
        step.object_ids = object_ids
        step.station_id = int(station_id if station_id is not None else -1)
        step.depends_on = self._unique_ints(depends_on)
        step.slide_ids = slide_ids
        self.steps.append(step)
        self.step_id += 1
        self.current_station = int(step.station_id)
        return int(step.step_id)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _is_wb_only(self, order):
        return int(order['product_id']) in WB_ONLY_PRODUCTS

    def _raw_size(self, raw):
        return 1 if 1 <= int(raw) <= 4 else 2

    def _digits(self, product_id):
        return [int(d) for d in str(int(product_id))]

    def _encode_order_slide(self, order_index, slot_index):
        return int(order_index) * 10 + int(slot_index)

    def _encode_return_slide(self, local_station_id, slot_index):
        return -((int(local_station_id) * 10) + int(slot_index))

    def _expand_station_object(self, object_id):
        object_id = int(object_id)
        if object_id in BATCH_TO_RAW:
            return [BATCH_TO_RAW[object_id]] * int(BATCH_SIZE)
        if 1 <= object_id <= 8:
            return [object_id]
        return []

    def _allocate_raw_slot_for_order(self, order_index, raw, capacity_used, slot_orders):
        raw_size = self._raw_size(raw)
        candidates = []
        for slot in RAW_SLOT_INDICES:
            if order_index in slot_orders[slot]:
                continue
            if capacity_used[slot] + raw_size <= RAW_SLIDE_CAPACITY_UNITS:
                candidates.append(slot)
        if not candidates:
            raise RuntimeError(
                f'AMR raw slide 용량 부족: order={order_index}, raw={raw}, '
                f'capacity_used={capacity_used}'
            )
        # Choose the most-filled feasible slide to preserve empty slides.
        slot = max(candidates, key=lambda s: capacity_used[s])
        capacity_used[slot] += raw_size
        slot_orders[slot].add(order_index)
        return int(slot)

    def _allocate_distinct_raw_slot(self, used_slots, raw):
        for slot in RAW_SLOT_INDICES:
            if slot not in used_slots:
                used_slots.add(slot)
                return int(slot)
        raise RuntimeError(f'raw slide 부족: raw={raw}, used_slots={sorted(used_slots)}')

    def _take_stock(self, raw, stock_by_raw):
        items = stock_by_raw.get(int(raw), [])
        if not items:
            return None
        if self.use_time_cost:
            idx = min(
                range(len(items)),
                key=lambda i: self._travel_time(self.current_station, items[i]['station_id']),
            )
            return items.pop(idx)
        return items.pop(0)

    def _take_waste_target(self, raw, station_info):
        targets = station_info['waste_targets_by_raw'].get(int(raw), [])
        if targets:
            if self.use_time_cost:
                idx = min(
                    range(len(targets)),
                    key=lambda i: self._travel_time(station_info['wb_id'], targets[i]['station_id']),
                )
                return int(targets.pop(idx)['station_id'])
            return int(targets.pop(0)['station_id'])
        return int(station_info['storage_ids'][0])

    def _source_depends(self, source):
        if source.get('kind') != 'recycle':
            return []
        recycle_order = source['recycle_order']
        sid = self.recycle_step_by_order_index.get(recycle_order['order_index'])
        return [] if sid is None else [sid]

    def _linked_recycles(self, produce_orders):
        seen = set()
        linked = []
        for produce_order in produce_orders:
            for source in produce_order.get('sources', []):
                if source.get('kind') != 'recycle':
                    continue
                recycle_order = source['recycle_order']
                key = recycle_order['order_index']
                if key not in seen:
                    seen.add(key)
                    linked.append(recycle_order)
        linked.sort(key=lambda ro: self._recycle_reuse_score(ro), reverse=True)
        return linked

    def _recycle_reuse_score(self, recycle_order):
        return len(recycle_order.get('reuse_materials', []))

    def _build_material_model(self, orders):
        produce_materials = []
        recycle_materials = []
        for order in orders:
            if order['order_type'] == Order.OT_PRODUCE:
                produce_materials.extend(order['materials'])
            elif order['order_type'] == Order.OT_RECYCLE:
                recycle_materials.extend(order['materials'])

        common = self._multiset_common(produce_materials, recycle_materials)
        produce_initial = self._subtract_multiset(produce_materials, common)
        recycle_leftover = self._subtract_multiset(recycle_materials, common)
        return {
            'produce_materials': produce_materials,
            'recycle_materials': recycle_materials,
            'common_reuse': common,
            'produce_initial': produce_initial,
            'recycle_leftover': recycle_leftover,
            'produce_initial_counts': Counter(produce_initial),
            'recycle_leftover_counts': Counter(recycle_leftover),
        }

    def _multiset_common(self, left, right):
        count = Counter(right)
        out = []
        for item in left:
            if count[item] > 0:
                out.append(item)
                count[item] -= 1
        return out

    def _subtract_multiset(self, base, remove):
        count = Counter(remove)
        out = []
        for item in base:
            if count[item] > 0:
                count[item] -= 1
            else:
                out.append(item)
        return out

    def _multiset_overlap_count(self, left, right):
        return len(self._multiset_common(left, right))

    def _station_coord(self, station_id):
        station_id = int(station_id)
        if station_id in self.station_coords:
            return self.station_coords[station_id]
        return (float(station_id), 0.0)

    def _travel_time(self, from_station, to_station):
        if not self.use_time_cost:
            return 0.0
        if from_station is None or to_station is None:
            return 0.0
        if int(from_station) == int(to_station):
            return 0.0

        dist = None
        if self.waypoint_cost_map is not None:
            dist = self.waypoint_cost_map.station_distance(
                int(from_station),
                int(to_station),
            )

        if dist is None:
            x1, y1 = self._station_coord(int(from_station))
            x2, y2 = self._station_coord(int(to_station))
            dist = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

        travel_time = float(dist) / max(self.amr_speed_mps, 1e-6)
        travel_time += float(self.config.nav_align_time_avg)

        if (
            self.waypoint_cost_map is not None
            and self.waypoint_cost_map.station_uses_post_process(int(to_station))
        ):
            travel_time += float(self.config.nav_post_time_avg)

        return travel_time

    def _order_grouped_by_travel(self, grouped):
        if not grouped:
            return {}
        remaining = [(int(station), items) for station, items in grouped.items()]
        ordered = {}
        current = self.current_station
        while remaining:
            if self.use_time_cost:
                idx = min(
                    range(len(remaining)),
                    key=lambda i: self._travel_time(current, remaining[i][0]),
                )
            else:
                idx = 0
            station_id, items = remaining.pop(idx)
            ordered[int(station_id)] = items
            current = int(station_id)
        return ordered

    def _unique_ints(self, values: Iterable[int]):
        out = []
        seen = set()
        for value in values or []:
            ivalue = int(value)
            if ivalue not in seen:
                seen.add(ivalue)
                out.append(ivalue)
        return out

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log_plan_summary(self, orders):
        self.get_logger().info('===== Plan D 실행 계획 요약 =====')
        for order in orders:
            pid = int(order['product_id'])
            name = PRODUCT_NAMES.get(pid, str(pid))
            if order['order_type'] == Order.OT_PRODUCE:
                path = 'WB 전용' if self._is_wb_only(order) else 'AMR 조립'
                self.get_logger().info(
                    f'[order {order["order_index"]}] PRODUCE {pid} ({name}) | '
                    f'{path} | materials={order["materials"]}'
                )
                for source in order.get('sources', []):
                    if source['kind'] == 'recycle':
                        ro = source['recycle_order']
                        self.get_logger().info(
                            f'  raw {source["raw"]}: RECYCLE order {ro["order_index"]} '
                            f'product={ro["product_id"]} 결과를 WB에서 LOAD/재사용'
                        )
                    else:
                        self.get_logger().info(
                            f'  raw {source["raw"]}: station={source["station_id"]}에서 LOAD'
                        )
            elif order['order_type'] == Order.OT_RECYCLE:
                self.get_logger().info(
                    f'[order {order["order_index"]}] RECYCLE {pid} ({name}) | '
                    f'materials={order["materials"]}'
                )
                if order.get('reuse_materials'):
                    self.get_logger().info(f'  reuse={order["reuse_materials"]}')
                if order.get('waste_materials'):
                    self.get_logger().info(f'  return/waste={order["waste_materials"]}')
        self.get_logger().info('===============================')

    def _product_connection_count(self, product_id):
        materials = PRODUCT_MATERIALS.get(int(product_id), self._digits(int(product_id)))
        return max(0, len(materials) - 1)

    def _estimate_arm_action_time(self, step):
        n_objects = len(step.object_ids)
        if step.action == Step.LOAD:
            return n_objects * float(self.config.amr_load_time_sec_per_item)
        if step.action == Step.UNLOAD:
            return n_objects * float(self.config.amr_unload_time_sec_per_item)
        if step.action == Step.PRODUCE:
            connections = sum(
                self._product_connection_count(pid)
                for pid in list(step.object_ids)
            )
            return connections * float(self.config.amr_assemble_time_sec_per_connection)
        return 0.0

    def _estimate_wb_action_time(self, step):
        if not step.object_ids:
            return 0.0
        product_id = int(step.object_ids[0])
        connections = self._product_connection_count(product_id)
        if step.action == Step.RECYCLE:
            return connections * float(self.config.wb_recycle_time_sec_per_connection)
        return connections * float(self.config.wb_produce_time_sec_per_connection)

    def _log_steps(self, steps):
        type_map = {Step.AMR: 'AMR', Step.WB: 'WB '}
        action_map = {
            Step.LOAD: 'LOAD   ',
            Step.UNLOAD: 'UNLOAD ',
            Step.RECYCLE: 'RECYCLE',
            Step.GOAL: 'GOAL   ',
        }
        self.get_logger().info('===== Plan D 스텝 시퀀스 =====')
        estimate_amr_station = STATION_START_GOAL
        for step in steps:
            est_time = 0.0
            if step.type == Step.AMR:
                move_time = self._travel_time(estimate_amr_station, int(step.station_id))
                move_time += float(self.config.nav_overhead_sec)
                arm_time = self._estimate_arm_action_time(step)
                # AMR ASSEMBLE은 이동과 조립이 병렬이므로 max로 추정한다.
                if step.action == Step.PRODUCE:
                    est_time = max(move_time, arm_time)
                elif step.action == Step.GOAL:
                    est_time = move_time
                else:
                    est_time = move_time + arm_time
                estimate_amr_station = int(step.station_id)
            elif step.type == Step.WB:
                est_time = self._estimate_wb_action_time(step)

            action_name = (
                'ASSEMBLE'
                if step.type == Step.AMR and step.action == Step.PRODUCE
                else 'PRODUCE'
                if step.type == Step.WB and step.action == Step.PRODUCE
                else action_map.get(step.action, '?')
            )
            self.get_logger().info(
                f'[{step.step_id:2d}] {type_map.get(step.type, "??")} | '
                f'{action_name} | '
                f'objects={list(step.object_ids)} | '
                f'station={step.station_id} | '
                f'slide_ids={list(step.slide_ids)} | '
                f'depends_on={list(step.depends_on)} | '
                f'est={est_time:.2f}s'
            )
        self.get_logger().info('============================')
