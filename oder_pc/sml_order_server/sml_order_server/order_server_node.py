#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from sml_messages.msg import Task, Order, Station


# Station Type
ST_STORAGE = 1
ST_WORKBENCH = 2
ST_CUSTOMER = 3
ST_HYBRID = 4

# Order Type
ORDER_PRODUCTION = 1
ORDER_RECYCLING = 2


RAW_TO_BATCH = {
    1: 10,
    2: 20,
    3: 30,
    4: 40,
    5: 50,
    6: 60,
    7: 70,
    8: 80,
}

MIXED_BATCH = 90


PRODUCT_DB = {
    34: ("Battery", [3, 4]),
    13: ("Magnet", [1, 3]),
    81: ("E-Stop", [8, 1]),
    442: ("Carrot", [4, 4, 2]),
    241: ("Traffic Light", [2, 4, 1]),
    462: ("Small Tree", [4, 6, 2]),
    711: ("Hammer", [7, 1, 1]),
    4482: ("Big Carrot", [4, 4, 8, 2]),
    8518: ("Burger", [8, 5, 1, 8]),
    48132: ("Ice Cream", [4, 8, 1, 3, 2]),
    46262: ("Big Tree", [4, 6, 2, 6, 2]),
}


class OrderServer(Node):
    def __init__(self):
        super().__init__("order_server")

        self.task_pub = self.create_publisher(Task, "/sml/task", 10)
        self.published = False

        self.use_batches = self.get_input_int(
            "batches를 사용할까요? (1: 사용, 2: 사용 안 함): ",
            valid_values=[1, 2],
        ) == 1

        self.game_type = self.get_input_int(
            "경기 종류 입력 (1: 생산, 2: 재활용, 3: 라이프 사이클): ",
            valid_values=[1, 2, 3],
        )

        self.recycled_to_storage = False
        if self.game_type in [2, 3]:
            self.recycled_to_storage = self.get_input_int(
                "재활용 후 분해된 원자재를 ST_STORAGE에 배치할까요? (1: 예, 2: 아니오): ",
                valid_values=[1, 2],
            ) == 1

        self.random_order = self.get_input_int(
            "오더를 랜덤 생성할까요? (1: 예, 2: 순차 생성): ",
            valid_values=[1, 2],
        ) == 1

        self.allow_duplicate = self.get_input_int(
            "중복 제품을 허용할까요? (1: 예, 2: 아니오): ",
            valid_values=[1, 2],
        ) == 1

        self.order_count = self.get_input_int(
            "오더 물체 수 입력: ",
            min_value=1,
        )

        self.station_count = self.get_input_int(
            "스테이션 수 입력: ",
            min_value=3,
        )

        self.task, self.arena_layout = self.generate_task()
        self.print_official_style(self.task, self.arena_layout)

        self.timer = self.create_timer(3.0, self.publish_task)

    def get_input_int(self, msg, valid_values=None, min_value=None):
        while True:
            try:
                value = int(input(msg))

                if valid_values is not None and value not in valid_values:
                    print(f"입력 가능 값: {valid_values}")
                    continue

                if min_value is not None and value < min_value:
                    print(f"{min_value} 이상의 값을 입력하세요.")
                    continue

                return value

            except ValueError:
                print("정수를 입력하세요.")

    def convert_materials_to_batches(self, material_ids):
        if not self.use_batches:
            return material_ids

        unique_materials = set(material_ids)

        if len(unique_materials) >= 2:
            return [MIXED_BATCH]

        material_id = material_ids[0]
        return [RAW_TO_BATCH.get(material_id, material_id)]

    def select_products(self):
        product_ids = list(PRODUCT_DB.keys())

        if self.random_order:
            if self.allow_duplicate:
                return [
                    random.choice(product_ids)
                    for _ in range(self.order_count)
                ]

            if self.order_count > len(product_ids):
                raise ValueError(
                    "중복 제품을 허용하지 않을 경우, "
                    "오더 물체 수는 제품 종류 수보다 클 수 없습니다."
                )

            return random.sample(product_ids, self.order_count)

        if not self.allow_duplicate and self.order_count > len(product_ids):
            raise ValueError(
                "중복 제품을 허용하지 않을 경우, "
                "오더 물체 수는 제품 종류 수보다 클 수 없습니다."
            )

        if self.allow_duplicate:
            return [
                product_ids[i % len(product_ids)]
                for i in range(self.order_count)
            ]

        return product_ids[:self.order_count]

    def split_materials(self, materials, storage_count):
        buckets = [[] for _ in range(storage_count)]

        if storage_count <= 0:
            return buckets

        for i, material in enumerate(materials):
            buckets[i % storage_count].append(material)

        return buckets

    def generate_task(self):
        task = Task()
        selected_products = self.select_products()

        for i, product_id in enumerate(selected_products):
            order = Order()

            if self.game_type == 1:
                order.order_type = ORDER_PRODUCTION

            elif self.game_type == 2:
                order.order_type = ORDER_RECYCLING

            else:
                if i % 2 == 0:
                    order.order_type = ORDER_PRODUCTION
                else:
                    order.order_type = ORDER_RECYCLING

            order.product_id = product_id
            task.order_list.append(order)

        storage_materials = []
        customer_products = []

        for order in task.order_list:
            _, material_ids = PRODUCT_DB[order.product_id]

            if order.order_type == ORDER_PRODUCTION:
                storage_materials.extend(
                    self.convert_materials_to_batches(material_ids)
                )

            elif order.order_type == ORDER_RECYCLING:
                customer_products.append(order.product_id)

                if self.recycled_to_storage:
                    storage_materials.extend(
                        self.convert_materials_to_batches(material_ids)
                    )

        arena_layout = []

        storage_station_ids = []

        for station_id in range(1, self.station_count + 1):
            if station_id == 2:
                continue

            if station_id == self.station_count:
                continue

            if station_id == 1:
                storage_station_ids.append(station_id)
            else:
                if random.random() >= 0.2:
                    storage_station_ids.append(station_id)

        storage_buckets = self.split_materials(
            storage_materials,
            len(storage_station_ids)
        )

        storage_index = 0

        for station_id in range(1, self.station_count + 1):

            if station_id == 2:
                station_type = ST_WORKBENCH
                material_ids = []

            elif station_id == self.station_count:
                station_type = ST_CUSTOMER
                material_ids = customer_products

            elif station_id in storage_station_ids:
                station_type = ST_STORAGE
                material_ids = storage_buckets[storage_index]
                storage_index += 1

            else:
                station_type = ST_HYBRID
                material_ids = []

            arena_layout.append(
                {
                    "station_type": station_type,
                    "station_id": station_id,
                    "material_ids": material_ids,
                }
            )

            station_msg = Station()
            station_msg.station_type = station_type
            station_msg.station_id = station_id
            station_msg.material_ids = material_ids

            task.arena_layout.append(station_msg)

        return task, arena_layout

    def get_order_comment(self, order):
        return PRODUCT_DB.get(order.product_id, ("unknown", []))[0]

    def print_official_style(self, task, arena_layout):
        if self.game_type == 1:
            title = "Production"
        elif self.game_type == 2:
            title = "Recycling"
        else:
            title = "Life-Cycle"

        print(f"\n# {title}\n")
        print(f"# batches = {'ON' if self.use_batches else 'OFF'}")

        if self.game_type in [2, 3]:
            print(
                f"# recycled_to_storage = "
                f"{'ON' if self.recycled_to_storage else 'OFF'}"
            )

        print(
            f"# random_order = "
            f"{'ON' if self.random_order else 'OFF'}"
        )
        print(
            f"# allow_duplicate = "
            f"{'ON' if self.allow_duplicate else 'OFF'}"
        )

        print()

        print("order_list = ")
        print("{")
        for order in task.order_list:
            label = "P" if order.order_type == ORDER_PRODUCTION else "R"
            comment = self.get_order_comment(order)

            print(
                f"   order_type = {order.order_type} ; "
                f"product_id = {order.product_id:<18} "
                f"# {comment:<18} ({label})"
            )
        print("}\n")

        print("arena_layout = ")
        print("{")
        for station in arena_layout:
            material_text = ", ".join(
                str(x) for x in station["material_ids"]
            )

            print(
                f"   station_type = {station['station_type']}; "
                f"station_id = {station['station_id']}; "
                f"material_ids = {{{material_text}}}"
            )
        print("}\n")

    def publish_task(self):
        if self.published:
            return

        self.task_pub.publish(self.task)
        self.get_logger().info("Task with order_list and arena_layout published to /sml/task")

        self.published = True


def main(args=None):
    rclpy.init(args=args)

    node = OrderServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()