Node origin
Node destination

# There re various ways to traverse an edge. The options are listed below, to be used as content of `action_type`
string ACTION_DRIVE=DRIVE
string ACTION_OPEN_DOOR=OPEN_DOOR
string ACTION_PUSH_OBJECT=PUSH_OBJECT
#string ACTION_USE_ELEVATOR=USE_ELEVATOR
string action_type  # Any of ACTION_*

float32 cost  # How expensive is is to travel along this edge?
