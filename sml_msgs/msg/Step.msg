# Executor type
int32 AMR=0
int32 WB=1

# Step action
int32 LOAD=0
int32 UNLOAD=1
int32 PRODUCE=2
int32 RECYCLE=3
int32 GOAL=4

int32   step_id
int32   type
int32   action
int32[] object_ids
int32   station_id
int32[] depends_on

# Plan D AMR internal inventory ids.
# Normal order inventory:
#   slide_id = order_index * 10 + slot_index
#   order_index: 0-based order_list index
#   slot_index : 0~4 raw slides, 5/6 assembly slots, 9 product slot
# Return-to-storage inventory:
#   slide_id = -(local_station_id * 10 + slot_index)
#   local_station_id: 1~8 planner-local station id; home 0 is not used for returns.
# No-object/no-slot step uses an empty array.
int32[] slide_ids
