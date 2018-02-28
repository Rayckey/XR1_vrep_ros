#include "XR1.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "vrep_test/JointAngles.h"
#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>


#define PI 3.141592654

XR1::XR1(){

}

std::vector<geometry_msgs::TransformStamped> XR1::tfConversion(double JointAngles[26]){


 const static char* args1[] = {
  	"Base", "Knee" , "Back_Z" , "Back_X" , "Back_Y", "Neck_Z","Neck_X","Head", "Head_Tip"
  };
  const static std::vector<std::string> Torso_Frames_IDs(args1, args1 + 9);

  const static char* args2[] ={
    "Neck_Z","LS","LSX","LSY", "LEZ","LEX","LWZ","LWY","LWX","LEF"
  };
    const static std::vector<std::string> Left_Arm_Frames_IDs(args2, args2 + 9);

  const static char* args3[] ={
    "Neck_Z","RS","RSX","RSY", "REZ","REX","RWZ","RWY","RWX","REF"
  };
    const static std::vector<std::string> Right_Arm_Frames_IDs(args3, args3 + 9);

  std::vector<geometry_msgs::TransformStamped> res;

  // For Main Body
  for(int i = 0; i <Torso_Frames_IDs.size()-1; i++){
  	 geometry_msgs::TransformStamped transformStamped = DH2tf(i, JointAngles[i]);

  	 // transformStamped.header.stamp = ros::Time::now();
  	 transformStamped.header.frame_id = Torso_Frames_IDs[i];
  	 transformStamped.child_frame_id = Torso_Frames_IDs[i+1];
  	 res.push_back(transformStamped);
  }

  // For Left Arm
  for(int i = 0; i <Left_Arm_Frames_IDs.size()-1; i++){

  	int index = i + Torso_Frames_IDs.size()-1;
  	 geometry_msgs::TransformStamped transformStamped = DH2tf(index, JointAngles[index]);

  	 // transformStamped.header.stamp = ros::Time::now();
  	 transformStamped.header.frame_id = Left_Arm_Frames_IDs[i];
  	 transformStamped.child_frame_id = Left_Arm_Frames_IDs[i+1];
  	 res.push_back(transformStamped);
  }

   // For Right Arm
  for(int i = 0; i <Right_Arm_Frames_IDs.size()-1; i++){
  	int index = i + Torso_Frames_IDs.size() +Left_Arm_Frames_IDs.size()-2;
  	geometry_msgs::TransformStamped transformStamped = DH2tf(index, JointAngles[index]);

  	// transformStamped.header.stamp = ros::Time::now();
  	transformStamped.header.frame_id = Right_Arm_Frames_IDs[i];
  	transformStamped.child_frame_id = Right_Arm_Frames_IDs[i+1];
  	res.push_back(transformStamped);
  }

  return res;

}

double XR1::DHTableLookUp(const int row, const int col){
	const static int numJoint = 26;
	const static int numPara = 4;
	const static double la1 = 0.0478; 
	const static double la2 = 0.2157 ; 
	const static double la3 = 0.2160;
	const static double l1 = +3.9059e-1;
	const static double l2 = +8.2829e-1 - l1;
	const static double l3 = +9.3434e-1 -l1 - l2;
	const static double lx = +1.5779e-1;
	const static double lz = +1.1772e+0 - l3 -l2 -l1;	
	const static double l4 = +1.2624e+0 - l3 - l2 - l1;
	const static double l5 = +1.3164e+0 - l4 - l3 - l2 - l1;

  	// d a alpha offset
	const static double DHTable[numJoint*numPara] = {
	l1, 0, -PI/2, -PI/2,
    0, 0, PI/2, 0,
    l2, 0, -PI/2, 0,
    0, l3, -PI/2, -PI/2,
    0, 0, -PI/2, -PI/2, // Back Y
    l4, 0 ,-PI/2, -PI/2, //Head Begins Here IDX = 5
    0, l5, -PI/2, -PI/2,
    0, 0, 0, 0,
    lz, lx, 0, 0, // Left Arm Begins Here IDX = 8
    0, 0, -PI/2, -PI/2, 
    0, 0, PI/2, PI/2,
    0, la1, PI/2, PI/2,
    la2, 0, PI/2, PI/2,
    0, 0, -PI/2, 0,
    la3, 0, -PI/2, -PI/2,
    0, 0, -PI/2, -PI/2,
    0, 0, 0, 0,
    lz, -lx, 0, 0, // Right Arm Begins Here IDX = 17
    0, 0, -PI/2, -PI/2, 
    0, 0, PI/2, PI/2,
    0, -la1, PI/2, PI/2,
    la2, 0, PI/2, PI/2,
    0, 0, -PI/2, 0,
    la3, 0, -PI/2, -PI/2,
    0, 0, -PI/2, -PI/2,
    0, 0, 0, 0,
	};

	if(row>=numJoint || col >= numPara) return -9999;
	else return DHTable[row*4 + col];

 }

  geometry_msgs::TransformStamped XR1::DH2tf(int row, double angle){

  	geometry_msgs::TransformStamped temp;

  	double d = DHTableLookUp(row,0);
  	double a = DHTableLookUp(row,1);
  	double alpha = DHTableLookUp(row,2);
  	double theta = angle + DHTableLookUp(row,3);

  	temp.transform.translation.x = a*cos(theta);
  	temp.transform.translation.y = a*sin(theta);
  	temp.transform.translation.z = d;

  	tf2::Quaternion q;
  	q.setRPY(alpha, 0, theta);
  	temp.transform.rotation.x = q.x();
  	temp.transform.rotation.y = q.y();
  	temp.transform.rotation.z = q.z();
  	temp.transform.rotation.w = q.w();
  	return temp;
  }


void XR1::subscribeJointCurrentPosition(vrep_test::JointAngles msg){

  	static tf2_ros::TransformBroadcaster br;

	double JointAngles[26] ={
  		0, msg.Knee, msg.Back_Z, msg.Back_X, msg.Back_Y, msg. Neck_Z, msg. Neck_X, msg. Head,
		0, 0, msg. Left_Shoulder_X, msg. Left_Shoulder_Y, msg. Left_Elbow_Z, msg. Left_Elbow_X, msg. Left_Wrist_Z, msg. Left_Wrist_Y, msg. Left_Wrist_X,
		0, 0, msg. Right_Shoulder_X, msg. Right_Shoulder_Y, msg. Right_Elbow_Z, msg. Right_Elbow_X, msg. Right_Wrist_Z, msg. Right_Wrist_Y, msg. Right_Wrist_X
  	};

  	std::vector<geometry_msgs::TransformStamped> tfs =  tfConversion(JointAngles);
  	for(int i = 0; i< tfs.size(); i++){
  		tfs[i].header.stamp = ros::Time::now();
  		br.sendTransform(tfs[i]);
  	}

}

// void XR1::ActivateSubscriber(ros::NodeHandle nh){
//   ros::Subscriber sub = nh.subscribe("/JointAngle/Current" , 100 , &XR1::subscribeJointCurrentPosition, this);
// }