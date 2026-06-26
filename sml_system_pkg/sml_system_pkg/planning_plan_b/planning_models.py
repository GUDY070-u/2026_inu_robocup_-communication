"""Small internal data containers used by the planner modules."""

from dataclasses import dataclass
from typing import Dict, List, Any


@dataclass
class ArenaInfo:
    station_items: Dict[int, List[int]]
    stock_tokens: List[dict]
    waste_target_tokens: List[dict]
    wb_id: int
    customer_id: int
    storage_id: int

    def as_tuple(self):
        return (
            self.station_items,
            self.stock_tokens,
            self.waste_target_tokens,
            self.wb_id,
            self.customer_id,
            self.storage_id,
        )


PlanningOrder = Dict[str, Any]
