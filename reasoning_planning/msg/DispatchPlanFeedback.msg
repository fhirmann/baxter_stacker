bool success


# constants used for below error field
int8 NO_ERROR = 0
int8 MOTION_PLAN_NOT_FOUND = 1
int8 BLOCK_OUT_OF_RANGE = 2
int8 SERVICE_NOT_REACHABLE = 3
int8 OTHER_ERROR = 4
int8 PERCEPTION_DETECTED_DIFFERENT_POSITION_THAN_EXPECTED = 5
int8 MULTIPLE_SAME_BLOCKS_IN_THE_SCENE_DETECTED = 6
int8 NO_OTHER_SAME_BLOCK_IN_THE_SCENE = 7
int8 DIFFERENT_NUMBER_OF_BLOCKS_BETWEEN_PERCEPTION_AND_SCENE_DB = 8


#error codes for put down + 10
int8 PUT_DOWN_NO_ERROR = 10
int8 PUT_DOWN_HOLD_NO_BLOCK = 11      # arm already picked up a block, e.g. pickup has been called twice
int8 PUT_DOWN_PLAN_PRE_RELEASE = 12   # no motion plan found for pre-release pose
int8 PUT_DOWN_MOVEIT_SCENE_ERROR = 13 # moveit planning scene was not able to remove or add a block from or to scene
int8 PUT_DOWN_PLAN_RELEASE = 14       # no motion plan found for release pose
int8 PUT_DOWN_PLAN_POST_RELEASE = 15  # no motion plan found for release-grasp pose

#error codes for pickup + 20
int8 PICK_UP_NO_ERROR = 20
int8 PICK_UP_HOLD_BLOCK = 21         # arm already picked up a block, e.g. pickup has been called twice
int8 PICK_UP_BLOCK_NOT_FOUND = 22    # block with given ID could not be found in database
int8 PICK_UP_PLAN_PRE_GRASP = 23     # no motion plan found for pre-grasp pose
int8 PICK_UP_MOVEIT_SCENE_ERROR = 24 # moveit planning scene was not able to remove or add a block from or to scene
int8 PICK_UP_PLAN_GRASP = 25         # no motion plan found for grasp pose
int8 PICK_UP_PLAN_POST_GRASP = 26    # no motion plan found for post-grasp pose

int8 error_code