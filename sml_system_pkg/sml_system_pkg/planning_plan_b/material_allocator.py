"""Material source and waste target allocation."""

from collections import Counter


class MaterialAllocatorMixin:
    def _register_recycle_releases(self, recycle_orders):
        """RECYCLE 후 나올 재료 등록 → {material: [recycle_order, ...]} (개수 단위)"""
        recycle_available = {}
        for order in recycle_orders:
            if order.get('source_after_produce', False):
                continue
            for material in order['materials']:
                recycle_available.setdefault(material, []).append(order)
        return recycle_available

    def _assign_material_sources(self, produce_orders, recycle_available, stock_tokens, wb_id):
        """
        재료 출처 결정 + 가상 차감
        우선순위: RECYCLE 후 재료 → 초기 재고
        material_sources = [(material, source, dep_recycle, object_id, token_ref), ...]
        """
        for order in produce_orders:
            for material in order['materials']:

                # 1순위: RECYCLE 후 WB에 생기는 재료
                if material in recycle_available and recycle_available[material]:
                    recycle_order = recycle_available[material].pop(0)
                    order['material_sources'].append(
                        (material, 'WB', recycle_order, material, None)
                    )

                # 2순위: 초기 재고에서 가져오기
                else:
                    found = self._find_in_stock(material, stock_tokens, wb_id, order)
                    if found is not None:
                        station_id, object_id, token_ref = found
                        order['material_sources'].append(
                            (material, station_id, None, object_id, token_ref)
                        )
                    else:
                        raise RuntimeError(f'재료 {material}를 구할 수 없음')

    def _find_in_stock(self, material, stock_tokens, wb_id=None, order=None):
        candidates = [
            token for token in stock_tokens
            if token['raw'] == material and token['remaining'] > 0
        ]
        if not candidates:
            return None

        if self.use_time_cost:
            used_station_counts = Counter()
            if order is not None:
                for (_, source, dep, _, _) in order['material_sources']:
                    if dep is None and isinstance(source, int):
                        used_station_counts[source] += 1

            def cost(token):
                station_id = token['station_id']
                group_bonus = 1.5 * used_station_counts[station_id]
                return self._travel_time(station_id, wb_id) - group_bonus

            chosen = min(candidates, key=cost)
        else:
            chosen = candidates[0]

        # AMR/ARM에는 batch ID(10,20,...,80)를 그대로 보내지 않는다.
        # batch token 하나가 여러 raw를 담고 있으면, 필요한 개수만큼 raw ID로 분해해서 보낸다.
        # 예: object_id=40, BATCH_SIZE=5인 token에서 raw 4를 두 번 쓰면
        #     AMR step object_ids에는 [4, 4]가 들어간다.
        use_index = chosen['capacity'] - chosen['remaining']
        chosen['remaining'] -= 1

        amr_object_id = material
        raw_token_ref = (chosen['ref'], use_index)
        return chosen['station_id'], amr_object_id, raw_token_ref

    def _assign_waste_materials(
        self, recycle_orders, produce_orders, waste_target_tokens,
        fallback_storage_id, wb_id
    ):
        """RECYCLE 후 재료 → reuse / waste 분류 및 waste 반납 target 결정."""
        reused_by_recycle = {id(order): [] for order in recycle_orders}

        for po in produce_orders:
            for (material, source, dep_recycle, object_id, token_ref) in po['material_sources']:
                if dep_recycle is not None:
                    reused_by_recycle[id(dep_recycle)].append(material)

        for order in recycle_orders:
            remaining = list(order['materials'])
            for material in reused_by_recycle[id(order)]:
                if material in remaining:
                    remaining.remove(material)
                    order['reuse_materials'].append(material)

            order['waste_materials'] = remaining
            order['waste_items'] = self._assign_waste_targets(
                remaining, waste_target_tokens, fallback_storage_id, wb_id
            )

    def _assign_waste_targets(self, waste_materials, waste_target_tokens, fallback_storage_id, wb_id):
        assignments = []
        fallback_ref_base = 100000

        for i, material in enumerate(waste_materials):
            target = self._find_waste_target(material, waste_target_tokens, wb_id)
            if target is not None:
                station_id, object_id, token_ref = target
                assignments.append({
                    'raw': material,
                    'station_id': station_id,
                    'object_id': object_id,
                    'token_ref': token_ref,
                })
            else:
                # target station을 찾지 못하면 첫 storage에 raw ID 그대로 반납한다.
                assignments.append({
                    'raw': material,
                    'station_id': fallback_storage_id,
                    'object_id': material,
                    'token_ref': fallback_ref_base + i,
                })

        return assignments

    def _find_waste_target(self, material, waste_target_tokens, wb_id=None):
        candidates = [
            token for token in waste_target_tokens
            if token['raw'] == material and token['remaining'] > 0
        ]
        if not candidates:
            return None

        if self.use_time_cost:
            chosen = min(
                candidates,
                key=lambda token: self._travel_time(wb_id, token['station_id'])
            )
        else:
            chosen = candidates[0]

        # Waste target도 batch ID가 아니라 실제 raw ID로 반납하도록 step을 만든다.
        use_index = chosen['capacity'] - chosen['remaining']
        chosen['remaining'] -= 1

        amr_object_id = material
        raw_token_ref = (chosen['ref'], use_index)
        return chosen['station_id'], amr_object_id, raw_token_ref

    # --------------------------------------------------------
    # Step 3: WB 시퀀스 결정
    # --------------------------------------------------------

