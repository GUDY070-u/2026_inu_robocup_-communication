"""Order parsing and lifecycle material model helpers."""

import copy
from collections import Counter

from sml_plan_d_msgs.msg import Order

# 메시지에 OT_LIFECYCLE이 없을 수도 있으므로 3을 fallback으로 사용
ORDER_TYPE_LIFECYCLE = getattr(Order, 'OT_LIFECYCLE', 3)


class OrderParserMixin:
    def _parse_orders(self, order_list):
        produce_orders = []
        recycle_orders = []

        for order in order_list:
            parsed = {
                'order_type':       order.order_type,
                'product_id':       order.product_id,
                'materials':        self._parse_product_id(order.product_id),
                # material_sources: [(material, source, dep_recycle, object_id, token_ref), ...]
                # source가 station_id이면 AMR이 object_id를 source에서 LOAD한다.
                # source가 'WB'이면 해당 material은 RECYCLE 후 WB에서 바로 재사용한다.
                'material_sources': [],
                'waste_materials':  [],
                # waste_items: [{'raw': m, 'object_id': x, 'station_id': y, 'token_ref': z}, ...]
                'waste_items':      [],
                'reuse_materials':  [],
                'source_after_produce': False,
            }
            if order.order_type == Order.OT_PRODUCE:
                produce_orders.append(parsed)
            elif order.order_type == Order.OT_RECYCLE:
                recycle_orders.append(parsed)
            elif order.order_type == ORDER_TYPE_LIFECYCLE:
                # 단일 lifecycle order가 들어오는 경우만 produce→recycle 내부 연결로 해석한다.
                # 별도 OT_RECYCLE order는 항상 CUSTOMER에 있다고 가정한다.
                produce_part = copy.deepcopy(parsed)
                recycle_part = copy.deepcopy(parsed)
                produce_part['order_type'] = Order.OT_PRODUCE
                recycle_part['order_type'] = Order.OT_RECYCLE
                recycle_part['source_after_produce'] = True
                produce_orders.append(produce_part)
                recycle_orders.append(recycle_part)

        # 재료 개수가 적은 PRODUCE부터 처리하면 재사용/재고 배정이 단순해진다.
        produce_orders.sort(key=lambda o: len(o['materials']))
        return produce_orders, recycle_orders

    def _parse_product_id(self, product_id):
        return [int(d) for d in str(product_id)]

    def _multiset_common_preserve_order(self, left, right):
        right_count = Counter(right)
        common = []
        for item in left:
            if right_count[item] > 0:
                common.append(item)
                right_count[item] -= 1
        return common

    def _subtract_preserve_order(self, base, remove):
        remove_count = Counter(remove)
        result = []
        for item in base:
            if remove_count[item] > 0:
                remove_count[item] -= 1
            else:
                result.append(item)
        return result

    def _build_material_model(self, produce_orders, recycle_orders):
        produce_materials = []
        recycle_materials = []

        for order in produce_orders:
            produce_materials.extend(order['materials'])
        for order in recycle_orders:
            recycle_materials.extend(order['materials'])

        common = self._multiset_common_preserve_order(
            produce_materials, recycle_materials
        )
        produce_initial = self._subtract_preserve_order(produce_materials, common)
        recycle_leftover = self._subtract_preserve_order(recycle_materials, common)

        return {
            'produce_materials': produce_materials,
            'recycle_materials': recycle_materials,
            'common_reuse': common,
            'produce_initial': produce_initial,
            'recycle_leftover': recycle_leftover,
            'produce_initial_counts': Counter(produce_initial),
            'recycle_leftover_counts': Counter(recycle_leftover),
        }

