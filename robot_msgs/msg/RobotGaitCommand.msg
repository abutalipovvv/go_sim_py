# RobotGaitCommand.msg
# ID робота
uint16 robot_id
# Тип походки
uint16 gait_type
# Высота корпуса
float32 body_height
# Высота шага
float32 leg_height


# RobotGaitConstants.msg
uint16 GAIT_TYPE_IDLE = 0
uint16 GAIT_TYPE_TROT = 1
uint16 GAIT_TYPE_TROT_RUN = 2
uint16 GAIT_TYPE_CLIMB_STAIR = 3
uint16 GAIT_TYPE_TROT_OBST = 4