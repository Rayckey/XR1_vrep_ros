#ifndef XR1IMUDEFINE
#define XR1IMUDEFINE

namespace XR1IMU {

enum IMUs {
    Ground_IMU =            0,
    Back_IMU =              1,
    Left_Shoulder_IMU =     2,
    Left_Arm_IMU =          3,
    Right_Shoulder_IMU =    4,
    Right_Arm_IMU =         5,
    Left_Hand_IMU =         7,
    Right_Hand_IMU =        8,
    Left_Thumb_IMU =            9,
    Right_Thumb_IMU =           10,
    Left_Index_IMU =            11,
    Right_Index_IMU =           12,
    Left_Middle_IMU =           13,
    Right_Middle_IMU =          14,
    Left_Ring_IMU =             15,
    Right_Ring_IMU =            16,
    Left_Pinky_IMU =            17,
    Right_Pinky_IMU =           18,
    IMU_total =             19,
};

#ifndef USING_IMU_WITH_ACTUATOR_CONTROLLER
enum ActuatorID {
    Left_Front_Wheel = 1 ,
    Right_Front_Wheel = 2,
    Back_Wheel = 3,
    Knee_X = 4          ,
    Back_Z = 5          ,
    Back_X = 6          ,
    Back_Y = 7          ,
    Neck_Z = 8          ,
    Neck_X = 9          ,
    Head = 10            ,
    Left_Shoulder_X = 11,
    Left_Shoulder_Y = 12 ,
    Left_Elbow_Z = 13    ,
    Left_Elbow_X = 14    ,
    Left_Wrist_Z = 15    ,
    Left_Wrist_Y = 16    ,
    Left_Wrist_X = 17    ,
    Right_Shoulder_X = 18,
    Right_Shoulder_Y = 19,
    Right_Elbow_Z = 20   ,
    Right_Elbow_X = 21   ,
    Right_Wrist_Z = 22   ,
    Right_Wrist_Y = 23   ,
    Right_Wrist_X = 24   ,
    Left_Thumb = 25      ,
    Left_Index = 26     ,
    Left_Middle = 27     ,
    Left_Ring = 28       ,
    Left_Pinky = 29      ,
    Right_Thumb = 30      ,
    Right_Index = 31      ,
    Right_Middle = 32     ,
    Right_Ring = 33       ,
    Right_Pinky = 34      ,
    Actuator_Total = 35,
};
#endif


}


#endif // XR1IMUDEFINE

