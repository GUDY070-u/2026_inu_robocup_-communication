# Step.msg
# Plan D interface

# executor type
int32 AMR=0
int32 WB=1

# action type
int32 LOAD=0
int32 UNLOAD=1
int32 PRODUCE=2
int32 RECYCLE=3
int32 GOAL=4

# slide_id encoding used by Plan D
#
# Normal order-linked inventory:
#   slide_id = order_index * 10 + slot_index
#   order_index: 0~9, based on order_list order
#   slot_index:
#     0~4 : raw material slides
#     5   : AMR assembly slot 0
#     6   : AMR assembly slot 1
#     9   : product slot, only for recycle products and WB-only products
#
# Return-to-storage inventory:
#   slide_id = -(local_station_id * 10 + slot_index)
#   local_station_id: 1~8, home 0 is not used as a return target
#
# Empty/no object step:
#   object_ids=[] and slide_ids=[]

int32   step_id
int32   type
int32   action
int32[] object_ids
int32   station_id
int32[] depends_on
int32[] slide_ids
