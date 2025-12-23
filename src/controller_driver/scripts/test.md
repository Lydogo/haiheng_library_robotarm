'''bash
curl -X POST "http://0.0.0.0:7550/plan_to_position"\
    -H "Content-Type: application/json"\
    -d '{
      "group_name": "Right_arm",
      "position": {
          "x": 0.128247,
          "y": -0.361356,
          "z": 0.755561
          },
        "orientation": {
          "w": 0.707706,
          "x": 0.005255,
          "y": -0.006008,
          "z": 0.706462
          },
      "is_straight_constraint": false,
      "eff_step": 0.01
}'

curl -X POST "http://0.0.0.0:7550/plan_to_position"\
  -H "Content-Type: application/json"\
  -d '{
    "group_name": "Right_arm",
    "position": {
      "x": 0.363736, 
      "y": -0.566362, 
      "z": 0.606334
    },
    "orientation": {
      "w": -0.282576, 
      "x": 0.959063, 
      "y": 0.005277, 
      "z": -0.017909
    },
    "is_straight_constraint": false,
    "is_wall":false,
    "eff_step": 0.01
}'

curl -X POST "http://0.0.0.0:7550/calculate_target_position_from_pixel"\
  -H "Content-Type: application/json"\
  -d '{
    "group_name": "Right_arm",
    "x": 960,
    "y": 540,
    "d": 0.25,
    "theta": 30
  }'
curl -X GET "http://0.0.0.0:7550/get_current_pose_http?group_name=Right_arm"

'''