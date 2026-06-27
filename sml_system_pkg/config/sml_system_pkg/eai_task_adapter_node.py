#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException

from sml_messages.msg import Task as EaiTask

from sml_msgs.msg import Task as SmlTask
from sml_msgs.msg import Order as SmlOrder
from sml_msgs.msg import Station as SmlStation

from sml_system_pkg.arena_side_utils import (
    normalize_side,
    side_to_eai_topic,
    remap_eai_station_id_to_amr_id,
    remap_eai_station_type_to_internal_type,
)


class EaiTaskAdapterNode(Node):
    """
    공식 eai_task_server의 sml_messages/msg/Task를
    우리 planner가 사용하는 sml_msgs/msg/Task로 변환하는 어댑터 노드.

    반영 규칙:
    - side:=a -> /eai/task/side_a 구독
    - side:=b -> /eai/task/side_b 구독
    - A 경기장 AMR station id는 1~8
    - B 경기장 AMR station id는 9~16
    - B 조립로봇 위치는 15
    - 공식 HYBRID station은 내부적으로 WORKBENCH로 변환
    - name은 내부 msg에 복사하지 않고 remap/log 용도로만 사용
    """

    def __init__(self):
        super().__init__('eai_task_adapter_node')

        self.declare_parameter('side', 'a')
        self.declare_parameter('output_topic', '/sml/task')

        self.side = normalize_side(self.get_parameter('side').value)
        self.input_topic = side_to_eai_topic(self.side)
        self.output_topic = self.get_parameter('output_topic').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(SmlTask, self.output_topic, qos)
        self.sub = self.create_subscription(
            EaiTask,
            self.input_topic,
            self.task_callback,
            qos,
        )

        self.get_logger().info('[ADAPTER] eai_task_adapter_node 시작')
        self.get_logger().info(f'[ADAPTER] selected side: {self.side}')
        self.get_logger().info(
            f'[ADAPTER] input : {self.input_topic}  (sml_messages/msg/Task)'
        )
        self.get_logger().info(
            f'[ADAPTER] output: {self.output_topic} (sml_msgs/msg/Task)'
        )

    def task_callback(self, msg: EaiTask):
        self.get_logger().info(
            f'[ADAPTER] 공식 Task 수신: '
            f'{len(msg.order_list)} orders, {len(msg.arena_layout)} stations'
        )

        converted = SmlTask()

        converted.order_list = []
        for src_order in msg.order_list:
            dst_order = SmlOrder()
            dst_order.order_type = int(src_order.order_type)
            dst_order.product_id = int(src_order.product_id)
            converted.order_list.append(dst_order)

            self.get_logger().info(
                '[ADAPTER] Order 변환: '
                f'name={getattr(src_order, "name", "no_name")}, '
                f'type={dst_order.order_type}, '
                f'product_id={dst_order.product_id}'
            )

        converted.arena_layout = []

        st_storage = getattr(SmlStation, 'ST_STORAGE', 1)
        st_workbench = getattr(SmlStation, 'ST_WORKBENCH', 2)
        st_customer = getattr(SmlStation, 'ST_CUSTOMER', 3)

        for src_station in msg.arena_layout:
            dst_station = SmlStation()

            src_name = getattr(src_station, 'name', '')
            official_id = int(src_station.station_id)
            official_type = int(src_station.station_type)

            amr_station_id = remap_eai_station_id_to_amr_id(
                src_name,
                official_id,
            )
            internal_type = remap_eai_station_type_to_internal_type(
                src_name,
                official_type,
                st_storage=st_storage,
                st_workbench=st_workbench,
                st_customer=st_customer,
            )

            dst_station.station_id = int(amr_station_id)
            dst_station.station_type = int(internal_type)
            dst_station.material_ids = list(src_station.material_ids)

            converted.arena_layout.append(dst_station)

            self.get_logger().info(
                '[ADAPTER] Station 변환: '
                f'name={src_name}, '
                f'official_id={official_id} -> amr_id={dst_station.station_id}, '
                f'official_type={official_type} -> internal_type={dst_station.station_type}, '
                f'materials={list(dst_station.material_ids)}'
            )

        self.pub.publish(converted)
        self.get_logger().info(
            f'[ADAPTER] 변환 Task 발행 완료 -> {self.output_topic}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EaiTaskAdapterNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()