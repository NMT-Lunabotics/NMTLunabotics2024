#!/usr/bin/env bash

rostopic pub --once /move_base/result move_base_msgs/MoveBaseActionResult "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
status:
  goal_id:
    stamp:
      secs: 0
      nsecs: 0
    id: ''
  status: 3  # This represents success
  text: 'Goal reached.'
result: {}"

