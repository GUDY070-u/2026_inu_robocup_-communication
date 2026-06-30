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
int32[] slide_ids
int32   station_id
int32[] depends_on

