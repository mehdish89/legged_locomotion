#include <iostream>
#include <fstream>
#include <ros/ros.h>



// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MotionPlanRequest.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/JointState.h"

#include <geometry_msgs/PoseArray.h>

#include "daedalus_control/Stats.h"
#include "daedalus_control/Eval.h"

#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_listener.h>

#include <std_srvs/Empty.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>

#define IKFAST_HAS_LIBRARY


#include "ikfast.h"


#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <mutex>   

boost::mutex io_mutex;


using namespace std;

using namespace ikfast;



#ifdef IKFAST_NAMESPACE
using namespace IKFAST_NAMESPACE;
#endif


geometry_msgs::Pose normalize(geometry_msgs::Pose pose)
{
	double val = sqrt(pose.orientation.x * pose.orientation.x +
						pose.orientation.y * pose.orientation.y +
					 	pose.orientation.z * pose.orientation.z +
					 	pose.orientation.w * pose.orientation.w);
		
	pose.orientation.x /= val;
	pose.orientation.y /= val;
	pose.orientation.z /= val;
	pose.orientation.w /= val;

	return pose;
}


vector<double> inverseK(geometry_msgs::Pose pose, vector<IkReal> last = vector<IkReal>())
{
	pose = normalize(pose);
	Eigen::Affine3d p;
	tf::poseMsgToEigen(pose, p);
	Eigen::Matrix<double,4,4> m = p.matrix();

	IkReal *trans = new IkReal[4];
    IkReal *rot= new IkReal[9];

    for(int i=0;i<3;i++)
    	trans[i] = m(i,3);
    	
    for(int i=0;i<9;i++)
    	rot[i] = m(i/3, i%3);

	IkSolutionList<IkReal> solutions;

    ComputeIk(trans, rot, NULL, solutions);

    double lse = 1000000;
    vector<double> vals;

    // printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());

	if((int)solutions.GetNumSolutions()==0)
		return vals;    

    std::vector<IkReal> solvalues(GetNumJoints());
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        // printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        
        // for( std::size_t j = 0; j < solvalues.size(); ++j)
        //     printf("%.15f, ", solvalues[j]);
        // printf("\n");


        if(last.size() == 0)
        	return solvalues;

        double sum = 0;

        for(int k = 0;k<solvalues.size();k++)
        {
        	double pi = 3.14157;
        	double ej = abs(solvalues[k]-last[k]);
        	if(ej>=pi)
        		ej = 2*pi - ej;
        	sum+= ej*ej;
        }
        sum = sqrt(sum);

        if(sum<lse)
        {
        	lse = sum;
        	vals = solvalues;
        }
    }

   	return vals;
}

geometry_msgs::Pose toPose(IkReal *trans, IkReal *rot)
{
	Eigen::Matrix<double,4,4> m;
    for(int i=0;i<3;i++) {
    	if(trans[i]!=trans[i] || abs(trans[i]) > 6.3){
    		return geometry_msgs::Pose();
    		trans[i]=0;
    	}
    	m(i,3) = trans[i];
    	m(3,i) = 0;
    }
    for(int i=0;i<9;i++){

    	if(rot[i]!=rot[i] || abs(rot[i]) > 6.3){
    		return geometry_msgs::Pose();
    		rot[i]=0;
    	}
    	m(i/3, i%3) = rot[i];
	}


   	m(3,3) = 1;

    Eigen::Affine3d p(m);

   	// cout << p.matrix() << endl;

   	geometry_msgs::Pose pose;

   	tf::poseEigenToMsg(p, pose);

   	pose = normalize(pose);

   	

   	// cout << pose << endl;
   	/*

   	tf::poseMsgToEigen(pose, p);

   	cout << p.matrix() << endl;

	*/
   	//inverseK(pose);

   	return pose;
}

geometry_msgs::Pose forwardK(double joints[])
{
	IkReal *j = joints;
	IkReal *trans = new IkReal[4];
    IkReal *rot= new IkReal[9];

    for(int i=0;i<9;i++)
    	rot[i] = 0;
    
	// for(int i=0;i<5;i++)
	// 	cout << j[i] << " ";
	// cout << endl;

    ComputeFk(j, trans, rot);

    // for(int i=0;i<9;i++)
    // {
    // 	printf("%.4f, ", rot[i]);
    //     // cout << rot[i] << " ";
    //     if(i%3 == 2)
    //     	printf("%.4f \n", trans[i/3]);
    //         // cout << trans[i/3] << endl;
    // }
    // cout << endl;


    geometry_msgs::Pose pose;
    pose = toPose(trans, rot);



    return pose;
}

geometry_msgs::Pose forwardK(vector<double> vals)
{
	double *joints = new double[5];
	for(int i=0;i<5;i++)
		joints[i] = vals[i];
	return forwardK(joints);
}



void runFK()
{
    IkReal *j = new IkReal[5];
    for(int i=0;i<5;i++)
        cin >> j[i];

    IkReal *trans = new IkReal[4];
    IkReal *rot= new IkReal[9];
    cout << "OK" << endl;
    ComputeFk(j, trans, rot);
    cout << "after" << endl;
    if(trans!=NULL)
    {
        cout << "( ";
        cout << trans[0] << ", ";
        cout << trans[1] << ", ";
        cout << trans[2] << " )";
        cout << endl;
    }

    if(rot!=NULL)
    {
        cout << "[ ";
        for(int i=0;i<9;i++)
        {
            cout << rot[0];
            if(i%3 == 2)
                cout << endl;
            else
                cout << ", ";
        }
        cout << " ]" << endl;
    }

    cout << " ------- OR ------- " << endl << endl;

    for(int i=0;i<9;i++)
    {
        cout << rot[0] << " ";
        if(i%3 == 2)
            cout << trans[i/3] << " ";
    }
    cout << endl;

    cout << " ------- AFTER IK ------- " << endl << endl;    

    IkSolutionList<IkReal> solutions;

    ComputeIk(trans, rot, NULL, solutions);
    cout << solutions.GetNumSolutions() << endl;

    printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
    std::vector<IkReal> solvalues(GetNumJoints());
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j)
            printf("%.15f, ", solvalues[j]);
        printf("\n");
    }
}

int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch  = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
		ungetc(ch, stdin);
		cout << "\b \b";
		return 1;
	}

	return 0;
}


ros::Publisher *follow;

ros::ServiceClient client;



trajectory_msgs::JointTrajectoryPoint getTrajectoryPoint(robot_state::RobotStatePtr state, string group, double dur = 1) 
{
    const robot_state::JointModelGroup* jmg = state->getJointModelGroup(group);
    const std::vector<std::string> &joint_names = jmg->getJointModelNames();    
    std::vector<std::string> jn;

    for(int i=1;i<joint_names.size();i++)
        jn.push_back(joint_names[i]);
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(dur);
    for(int i=0;i<jn.size();i++) {
        point.positions.push_back(*state->getJointPositions(jn[i]));
        //point.velocities.push_back(*state->getJointVelocities(jn[i])*0);
        //point.accelerations.push_back(*state->getJointAccelerations(jn[i])*0);        
    }
    return point;
}



robot_model_loader::RobotModelLoader *rml;
robot_model::RobotModelPtr kinematic_model;

void invert(robot_state::RobotStatePtr state)
{
    vector <double> vals(5);
    state->copyJointGroupPositions("LEG2", vals);
    for(int i=0;i<vals.size();i++)
        vals[i] = -vals[i];
    state->setJointGroupPositions("LEG2", vals);

    state->copyJointGroupPositions("LEG3", vals);
    for(int i=0;i<vals.size();i++)
        vals[i] = -vals[i];
    state->setJointGroupPositions("LEG3", vals);
}

void reverse(vector <robot_state::RobotStatePtr> &traj)
{
    std::reverse(traj.begin(),traj.end()); 
}

void join(std::vector< robot_state::RobotStatePtr > &traj, std::vector< robot_state::RobotStatePtr > &other)
{
    for(int i=0;i<other.size();i++)
        traj.push_back(other[i]);
}

ofstream fout("valid_space.mat");

void validSpace(int leg)
{
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("LEG"+std::to_string(leg));    

    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();    

    Eigen::Affine3d end_effector_state = kinematic_state->getGlobalLinkTransform("leg"+std::to_string(leg)+"link5");

    geometry_msgs::Pose pose;

    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1; 


    
    for(double y=0; y<0.4; y+=0.01)
        for(double z=-0.4; z<0.4; z+=0.01) {
            bool has = false;
            double last_x;
            for(double x=-0.4; x<0.4; x+=0.01)    
            {   
                pose.position.x = x;
                pose.position.y = y;
                pose.position.z = z;
                
                tf::poseMsgToEigen(pose, end_effector_state);
                
                bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);    
                if(found_ik)
                    last_x = x;                

                if(found_ik && !has) {
                    fout << y << " " << z << " " << x << " ";
                    has = true;
                }
            }
            if(has)
                fout << last_x << endl;
        }
}


double get_msecs()
{	
	//struct timeval tv;
	//gettimeofday(&tv, NULL);

	ros::Time now = ros::Time::now();

	double millisecondsSinceEpoch =
		//(unsigned long long)(now.toSec()) * 1000 +
		(now.toNSec()) / 1000000;
	return millisecondsSinceEpoch;
}

double get_secs()
{	
	//struct timeval tv;
	//gettimeofday(&tv, NULL);

	ros::Time now = ros::Time::now();

	double millisecondsSinceEpoch =
		//(unsigned long long)(now.toSec()) * 1000 +
		(now.toSec());
	return millisecondsSinceEpoch;
}

typedef vector<double> Gait;

int iteration = 0;

double gx, gy, gz, gr;


double lx = 0 , ly = 0, lt = 0, lv = 0;
double ev2 = 0, ev = 0, edv = 0;
int v_count = 0;
double vel = 0;
double dist = 0;
double real_vel = 0;
double rel;
double meff = 0;

double sz2 = 0, sz = 0;

bool new_vel = false;

bool updated = false;

double rot_sum, rot_sum2;

double work = 0;

int timer = 0;

int e_timer = 0;

ros::Time last_time;
ros::Time ltime;

void effort_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	//e_timer++;
	//if(e_timer%10!=0)
		//return;

	ros::Time time =  msg->header.stamp;	
	ros::Duration dur = time - last_time;
	double dt = dur.toSec();
	
	for(int i = 0;i<msg->effort.size();i++) {
		double eff = abs(msg->effort[i]);
		double vel = abs(msg->velocity[i]);
		
		work += eff * vel * dt;

		//cout << work << endl;
		if(work<0)
		{
			cout << eff << " --- " << vel << " --- " << dt << endl;
			work = 0;
			break;
		}
	}
	last_time = time;
	ltime = last_time;
}



void stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{

	//ltime = last_time;//msg->header.stamp;

	timer++;
	
	if(true || timer%10==0) {

		lx = gx;
		ly = gy;


		gx = msg->pose[1].position.x;
		gy = msg->pose[1].position.y;
		gz = msg->pose[1].position.z;
		gr = msg->pose[1].orientation.z;
		
		//double vx = msg->twist[1].linear.x;
		//double vy = msg->twist[1].linear.y;
		//real_vel = sqrt(vx*vx+vy*vy);

		updated = true;

		v_count++;

		sz += gz;
		sz2 += gz*gz;

		dist += sqrt(abs(lx-gx) * abs(lx-gx) + abs(ly-gy) * abs(ly-gy));

		geometry_msgs::Pose pose = msg->pose[1];

		Eigen::Affine3d eigen;
		tf::poseMsgToEigen(pose, eigen);    

		Eigen::Vector4d axis(0,0,1,0);

		double rot = acos(axis.dot(eigen.matrix() * axis));

		rot_sum += rot;
		rot_sum2 += rot*rot;
	}	 
}




void reset_world()
{

/**/
	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = (std::string) "daedalus";
	modelstate.reference_frame = (std::string) "world";
	modelstate.pose.position.x = 0;
	modelstate.pose.position.y = 0;
	modelstate.pose.position.z = gz+0.05;
	modelstate.pose.orientation.x = 0;
	modelstate.pose.orientation.y = 0;
	modelstate.pose.orientation.z = 0;
	modelstate.pose.orientation.w = 0;

	
	gazebo_msgs::SetModelState setmodelstate;
	setmodelstate.request.model_state = modelstate;
	client.call(setmodelstate);
	/**/
	
  /*
	std_srvs::Empty srv;
  	client.call(srv); 
  	*/
}


struct Result {
	
	int iteration;
	int sample;
		

	double distance;
	double stability;
	double power;
	double efficiency;
	double rel;
	double eff;
	double wand;
	int msecs;
	int code;
};

double lambda = 0.9;

//double W_v = 0.9, W_s = 0.1, W_e = 10;

// double W_v = 0.9, W_s = 0.06, W_e = 10;

// double W_v = 1.2, W_s = 0.06, W_e = 10;

// double W_v = 6, W_s = 0.1, W_e = 5;

//

// stab
//double W_v = 3, W_s = 0.1, W_e = 3;

// vel
double W_v = 6, W_s = 0.06, W_e = 3;

// double W_v = 3, W_s = 0.06, W_e = 8;


// ee
// double W_v = 3, W_s = 0.06, W_e = 6;

double A_e = 1;

/* experiment for stability * /

W_s = 0.1 --> done: workspace worked better a lot, opt is extended -- ws=4.30, js=1.5
W_s = 0.5 --> done: ws worked a lot better, extended - ws = 30, js = 8
W_s = 0.05 --> done: js worked better
W_s = 0.075 --> done: ws worked better
W_s = 0.025 --> done : js worked better



*/

vector <double> inv_kin(geometry_msgs::Pose pose, int leg)
{
	return inverseK(pose);


	string group = "LEG" + std::to_string(leg);
    robot_state::RobotStatePtr state(new robot_state::RobotState(kinematic_model));
   	const robot_state::JointModelGroup* jmg = state->getJointModelGroup(group);
   	
	vector <double> vals;
	Eigen::Affine3d ee_pose;
    tf::poseMsgToEigen(pose, ee_pose);    
    //tf::poseMsgToEigen(to, ee_to);
    bool found_ik = state->setFromIK(jmg, ee_pose, 10, 1);
	if(found_ik)
	  	state->copyJointGroupPositions(jmg, vals);  
	return vals;
}

geometry_msgs::Pose fw_kin(vector <double> joints, int leg)
{
	return forwardK(joints);


	leg = 1;
	string group = "LEG" + std::to_string(leg);
	string ee_link_name = "leg"+std::to_string(leg)+"link5";
    robot_state::RobotStatePtr state(new robot_state::RobotState(kinematic_model));
   	const robot_state::JointModelGroup* jmg = state->getJointModelGroup(group);
	state->setJointGroupPositions(jmg, joints);
	const Eigen::Affine3d &ee_state = state->getGlobalLinkTransform(ee_link_name);
	
	geometry_msgs::Pose pose;
	tf::poseEigenToMsg(ee_state, pose);    
	
	/*f<double>(inv_kin(pose, leg));
	cout << "-----------" << endl;
	cout << pose << endl;*/
	
	return pose;
}



double objective(Result result)
{
	double vel = result.distance/((double)result.msecs/1000);
	//return result.distance - result.stability * 0.5;
	//return result.distance * 1.5 + result.stability * 0 - result.wand * 0.1 - result.rel * 50 + result.eff * 10;
	//return vel * lambda  - result.stability * (1 - lambda);
	
	//return vel * W_v  - result.stability * W_s + W_e / result.efficiency;
	
	return vel * W_v  + result.stability * W_s / 3 + W_e / (result.efficiency * A_e);
	
}

void show_result(Result result)
{
	cout << "Objective value = " << objective(result) << endl;
	cout << "Velocity = " << result.distance/((double)result.msecs/1000) << endl;
	cout << "Height = " << result.eff << endl;
	cout << "Time = " << result.msecs << endl;
	cout << "Z Variance = " << result.rel << endl;
	cout << "Distance = " << result.distance << endl;
	cout << "Move Forward = " << result.stability << endl;
	cout << "Wandering = " << result.wand << endl;
	cout << endl;
}


int steps = 20;

vector <trajectory_msgs::JointTrajectoryPoint> interpolate(int leg, 
															geometry_msgs::Pose from, 
															geometry_msgs::Pose to, 
															double dur = 1, 
															double delay = 0)
{
	string group = "LEG" + std::to_string(leg);

    vector< trajectory_msgs::JointTrajectoryPoint> traj;
    //trajectory_msgs::JointTrajectory traj;
    robot_state::RobotStatePtr state(new robot_state::RobotState(kinematic_model));

    string ee_link_name = "leg"+std::to_string(leg)+"link5";
    //ee_link_name = "leg1link5";
    
    
    const robot_state::LinkModel * ee_link =  state->getLinkModel(ee_link_name);

    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("LEG"+std::to_string(leg));
    
    
    int count = 0;
    for(int i=0;i<steps-1;i++) {
        
        double timestamp = (i+1) * dur / steps + delay;
        
        if(timestamp < 0)
        	continue;
        
        geometry_msgs::Pose pose = from;
        pose.position.x += (to.position.x-from.position.x) * (i+1) / steps;
        pose.position.y += (to.position.y-from.position.y) * (i+1) / steps;
        pose.position.z += (to.position.z-from.position.z) * (i+1) / steps;
        
        /////////////
        
        pose.orientation.x += (to.orientation.x-from.orientation.x) * (i+1) / steps;
        pose.orientation.y += (to.orientation.y-from.orientation.y) * (i+1) / steps;
        pose.orientation.z += (to.orientation.z-from.orientation.z) * (i+1) / steps;
        pose.orientation.w += (to.orientation.w-from.orientation.w) * (i+1) / steps;
        
        
        
       
        /*
        if(i+1<steps) {
		    pose.orientation.x = 0.1;
		    pose.orientation.y = 0;
		    pose.orientation.z = 0;
		    pose.orientation.w = 1;
		    
		}
		*/
		
		double val = sqrt(pose.orientation.x * pose.orientation.x +
						pose.orientation.y * pose.orientation.y +
					 	pose.orientation.z * pose.orientation.z +
					 	pose.orientation.w * pose.orientation.w);
		
		pose.orientation.x /= val;
		pose.orientation.y /= val;
		pose.orientation.z /= val;
		pose.orientation.w /= val;
        
		/////////////
    
        Eigen::Affine3d ee_pose;
        tf::poseMsgToEigen(pose, ee_pose);    
        //tf::poseMsgToEigen(to, ee_to);
        bool found_ik = state->setFromIK(joint_model_group, ee_pose, 10, 1);
        
        //cout << pose << endl;
        
        if(found_ik) 
        {     
            robot_state::RobotStatePtr new_state(new robot_state::RobotState(*state.get()));
            invert(new_state);  
            
            trajectory_msgs::JointTrajectoryPoint point;
            point = getTrajectoryPoint(new_state, group, timestamp);
			traj.push_back(point);

            count++;
        }
    }
    //cout << "percent: " << double(count)*100/double(steps) << endl;

    //if(result<0.7)
      //  traj.clear();
    
    return traj;
}

geometry_msgs::Pose interp(geometry_msgs::Pose from, geometry_msgs::Pose to, double alpha)
{
	geometry_msgs::Pose pose;	
	pose.position.x = alpha * to.position.x + from.position.x * (1-alpha);
    pose.position.y = alpha * to.position.y + from.position.y * (1-alpha);
    pose.position.z = alpha * to.position.z + from.position.z * (1-alpha);
        
        
    pose.orientation.x = alpha * to.orientation.x + from.orientation.x * (1-alpha);
    pose.orientation.y = alpha * to.orientation.y + from.orientation.y * (1-alpha);
    pose.orientation.z = alpha * to.orientation.z + from.orientation.z * (1-alpha);
    pose.orientation.w = alpha * to.orientation.w + from.orientation.w * (1-alpha);

    return normalize(pose);
}


vector <geometry_msgs::Pose> *loop;


vector <trajectory_msgs::JointTrajectoryPoint> alpha_interpolate(int leg, 
																vector<double> gait,
																vector<geometry_msgs::Pose> poses,
																int start,
																int end, 
																double alpha = 0.5,
																double dur = 1, 
																double delay = 0)
{
	string group = "LEG" + std::to_string(leg);

	geometry_msgs::Pose from = poses[start];
	geometry_msgs::Pose to = poses[end];

	vector<double> current = vector<double> (gait.begin()+4+start*5,gait.begin()+9+start*5);
	vector<double> next = vector<double> (gait.begin()+4+end*5,gait.begin()+9+end*5);

    vector< trajectory_msgs::JointTrajectoryPoint> traj;
    

    if(alpha>1)
    	alpha = 1;
    if(alpha<0)
    	alpha = 0;

    


    vector<double> old = vector<double> (current);    

    int count = 0;
    for(int i=0;i<steps-1;i++) {
        
        double timestamp = (i+1) * dur / steps + delay;
        
        if(timestamp < 0)
        	continue;

        
        double beta = double(i+1)/double(steps);
        geometry_msgs::Pose wpose = interp(from, to, beta);


        vector <double> vals = vector<double> (current);

        for(int j = 0;j<5;j++)
        {
        	vals[j] += (next[j]-current[j]) * (i+1) / steps;
        
        }
        

        geometry_msgs::Pose jpose = fw_kin(vals, 1);
        

	    geometry_msgs::Pose pose = interp(jpose, wpose, alpha);		

	    loop->push_back(pose);
	   
    
        Eigen::Affine3d ee_pose;
        tf::poseMsgToEigen(pose, ee_pose);    
        
        vals = inverseK(pose, old);
        bool found_ik = (vals.size() > 0);
        
        
        if(found_ik) 
        {   
        	old = vector<double> (vals);  

            string group = "LEG" + std::to_string(leg);
			string ee_link_name = "leg"+std::to_string(leg)+"link5";
		    robot_state::RobotStatePtr new_state(new robot_state::RobotState(kinematic_model));
		   	const robot_state::JointModelGroup* jmg = new_state->getJointModelGroup(group);
			new_state->setJointGroupPositions(jmg, vals);

            invert(new_state);  
            
            trajectory_msgs::JointTrajectoryPoint point;
            point = getTrajectoryPoint(new_state, group, timestamp);
			traj.push_back(point);

            count++;
        }
        else
        	cout << "NOT FOUND" <<  endl;

        // cout << endl << endl;
    }

    //cout << "percent: " << double(count)*100/double(steps) << endl;

    //if(result<0.7)
      //  traj.clear();
    
    return traj;
}


trajectory_msgs::JointTrajectory cycle( int leg,
										vector<double> gait,
										vector<geometry_msgs::Pose> poses, 
										vector <double> times, 
										int count = 5, 
										double delay = 0,
										double alpha = 0.5 )
{
    string group = "LEG" + std::to_string(leg);
    robot_state::RobotStatePtr state(new robot_state::RobotState(kinematic_model));
    
    
    trajectory_msgs::JointTrajectory traj;
    
    
    const robot_state::JointModelGroup* jmg = state->getJointModelGroup(group);
    const std::vector<std::string> &joint_names = jmg->getJointModelNames();    

    std::vector<std::string> jn;

    for(int i=1;i<joint_names.size();i++)
        jn.push_back(joint_names[i]);

    traj.joint_names = jn;


	double t = 0;
	
	for(int i=0;i<times.size();i++)
		t+=times[i];    
    
    double init = 0;
    
    double timestamp = delay + init/2;    
    
    if(init > 0)
    {
		robot_state::RobotStatePtr state(new robot_state::RobotState(kinematic_model));
		vector <double> vals = {0, 0.78539, 0, 0, 0};	
		state->setJointGroupPositions(group, vals);
		invert(state);             
		
		trajectory_msgs::JointTrajectoryPoint point;
		point = getTrajectoryPoint(state, group, timestamp);
		
		traj.points.push_back(point);
    }
    
    timestamp = delay + init;    

    for(int i = 0;i<=count;i++){
        int size = poses.size();
        loop = new vector<geometry_msgs::Pose>();
        for(int j=0;j<size;j++){
            vector< trajectory_msgs::JointTrajectoryPoint> states;
            double dur;
            int current = j, next;
            
                        
            if(leg == 1 || leg == 2 ){   
            	next = (j+1)%size;
                // states = interpolate(leg, poses[j], poses[(j+1)%size], times[j], timestamp);

                states = alpha_interpolate(leg, 
								gait,
								poses,
								current,
								next, 
								alpha,
								times[j], 
								timestamp);

                timestamp += times[j];
            }
            else {
            
            	current = (size-j)%size;
            	next = (size-j-1)%size;
                // states = interpolate(leg, poses[(size-j)%size], poses[(size-j-1)%size], times[(size-j-1)%size], timestamp); 

                states = alpha_interpolate(leg, 
								gait,
								poses,
								current,
								next, 
								alpha,
								times[(size-j-1)%size], 
								timestamp);

                timestamp += times[(size-j-1)%size];
            }
            
            for(int k=0;k<states.size();k++){             
                if(states[k].time_from_start < ros::Duration(count*t))
                	traj.points.push_back(states[k]);
            }
            
            robot_state::RobotStatePtr state(new robot_state::RobotState(kinematic_model));
            vector <double> vals;	
			vals = vector<double> (gait.begin()+next*5+4,gait.begin()+next*5+9);
			state->setJointGroupPositions(group, vals);
            
            /*
            cout << next << endl;
            for(int i = 0;i<vals.size();i++)
            	cout << vals[i] << " ";
            cout << endl;
            */
            invert(state);             
            
			trajectory_msgs::JointTrajectoryPoint point;
            point = getTrajectoryPoint(state, group, timestamp);
            
            traj.points.push_back(point);
            
           
        }
    }
    
    
    //ros::Duration dur = traj.points[traj.points.size()-1].time_from_start - traj.points[0].time_from_start;
    cout << traj.points.size() << endl;
    
    return traj;
    //follow[leg].publish(traj);
}



template<typename T>
void f(vector<T> s)
{
    for(int i=0;i<s.size();i++)
    	cout << s[i] << ' ';
    cout << endl;
}

double galpha = -1;

void run(vector<double> gait, int count, int leg = 1)
{
	double t =  gait[0];
	double a1 = gait[1];
	double a2 = gait[2];
	double a3 = gait[3];
	double a4 = 1 - a1 - a2 - a3;
	
	double t1 = t * a1;
	double t2 = t * a2;
	double t3 = t * a3;
	double t4 = t * a4;

	double alpha = gait[gait.size()-1];
	alpha = gait[24];
 /*******************************************************************************/

	if(galpha>=0)
		alpha = galpha;

	
	geometry_msgs::Pose pose;
	vector<geometry_msgs::Pose> poses;
	vector <double> times;
	
	vector <double> vals;
	
	vals = vector<double> (gait.begin()+4,gait.begin()+9);
    pose = fw_kin(vals, leg);
    poses.push_back(pose);
    times.push_back(t1);    
    
    vals = vector<double> (gait.begin()+9,gait.begin()+14);
    pose = fw_kin(vals, leg);
    poses.push_back(pose);
    times.push_back(t2);    
    
    vals = vector<double> (gait.begin()+14,gait.begin()+19);
    pose = fw_kin(vals, leg);
    poses.push_back(pose);
    times.push_back(t3); 
    
    vals = vector<double> (gait.begin()+19,gait.begin()+24);
    pose = fw_kin(vals, leg);
    poses.push_back(pose);
    times.push_back(t4);
    
    
    trajectory_msgs::JointTrajectory traj[5];
    
    for(int i=1;i<=4;i++)    
        if(i%2==0)
            traj[i] = cycle(i, gait, poses, times, count, -t/2, alpha);
        else
			traj[i] = cycle(i, gait, poses, times, count, 0, alpha);
	
			
	for(int i=1;i<=4;i++)    
		follow[i].publish(traj[i]);
	//cycle(leg, poses, times, count, (leg%2)*(-t/2));

	cout << "ALPHA = " << alpha << endl;
	
}


Result instant_measure(vector<double> gait, int count)
{   
    run(gait, count);
    
    reset_world();
    
	updated = false;
	while(!updated)
		ros::Duration(0.1).sleep();

	ros::Duration dur(4);
	dur.sleep();
    
    cout << "Start to measure" << endl;
    
    double sx = gx, sy = gy, sr = gr;
	double stability = 0;
	int code = 0;
	unsigned long long st, en;
	
	
	v_count = 0;
	ev = 0;
	ev2 = 0;
	
	sz = 0;
	sz2 = 0;
	
	dist = 0;

	rot_sum = 0;
	rot_sum2 = 0;
	
	work = 0;
	last_time = ros::Time::now();

    
    st = get_secs();
    
    do{
        en = get_secs();
        // ros::spinOnce();
        ros::Duration(0.1).sleep();
        // cout<< en-st << endl;
    }
	//while(en-st<20);
    while(en-st<10);
	//while(en-st<gait[0]*count);
	
	//cerr << v_count << endl;
	
	double rot_dev = sqrt(rot_sum2/v_count);
	double power = work / (en-st);
	
	
	struct Result result; 
	result.distance = sqrt(abs(sx-gx) * abs(sx-gx) + abs(sy-gy) * abs(sy-gy));
	result.msecs = (en - st)*1000;
	
	result.wand = abs(dist - result.distance);//  /8;
	result.stability = 1/(result.wand / result.distance + rot_dev);//dist;
	
	if(result.distance<0.05)
		result.stability = 0;
	
	cerr << "Rotation Deviation >> " << rot_dev << endl;
	cerr << "Translation Total Variation >> " << result.wand << endl;
	cerr << "Power >> " << power << endl;
	cerr << "Energy Efficieny >> " << power/result.distance << endl;
	
	result.power = power;
	result.efficiency = power/result.distance;
	

    //compensate
    result.distance *= 2;

	result.code = code;
	result.rel = rot_dev;
	result.eff = sz/v_count;
    for(int i=0;i<gait.size();i++)
        cout << gait[i] << " ";
    cout << endl;
	show_result(result);
	return result;
}

Result measure(vector<double> gait, int count)
{

    
    
    

    /*run(gait[0], 
        gait[1],
        gait[2],
        gait[3],
        gait[4],
        gait[5],
        gait[6],
        count );*/
    
    
    
    run(gait, count);
    
    reset_world();
    
    updated = false;
    while(!updated)
        ros::spinOnce();

    ros::Duration dur(4);
    dur.sleep();
    
    cout << "Start to measure" << endl;
    
    double sx = gx, sy = gy, sr = gr;
    double stability = 0;
    int code = 0;
    unsigned long long st, en;
    
    
    v_count = 0;
    ev = 0;
    ev2 = 0;
    
    sz = 0;
    sz2 = 0;
    
    dist = 0;

    rot_sum = 0;
    rot_sum2 = 0;
    
    work = 0;
    last_time = ros::Time::now();

    
    st = get_secs();
    
    do{
        en = get_secs();
        ros::spinOnce();
    }
    while(en-st<20);
    // while(en-st<10);
    //while(en-st<gait[0]*count);
    
    //cerr << v_count << endl;
    
    double rot_dev = sqrt(rot_sum2/v_count);
    double power = work / (en-st);
    
    
    struct Result result; 
    result.distance = sqrt(abs(sx-gx) * abs(sx-gx) + abs(sy-gy) * abs(sy-gy));
    result.msecs = (en - st)*1000;
    
    result.wand = abs(dist - result.distance);//  /8;
    result.stability = 1/(result.wand / result.distance + rot_dev);//dist;
    
    if(result.distance<0.05)
        result.stability = 0;
    
    cerr << "Rotation Deviation >> " << rot_dev << endl;
    cerr << "Translation Total Variation >> " << result.wand << endl;
    cerr << "Power >> " << power << endl;
    cerr << "Energy Efficieny >> " << power/result.distance << endl;
    
    result.power = power;
    result.efficiency = power/result.distance;
    
    //compensate
    // result.distance *= 2;

    result.code = code;
    result.rel = rot_dev;
    result.eff = sz/v_count;
    for(int i=0;i<gait.size();i++)
        cout << gait[i] << " ";
    cout << endl;
    show_result(result);
    return result;
}


Result fake_measure(double delay = 0, double period = 20)
{      
	updated = false;
	while(!updated)
		ros::spinOnce();

	ros::Time st, en;
	st = ltime;
    
    do{
        en = ltime;
        //cout << en.toSec() << " --- " << st.toSec() << " ------- " << (en-st).toSec() << endl;
        ros::spinOnce();

    }
	while((en-st).toSec()<delay);

    
    cout << "Start to measure on " << ltime.toSec() << endl;
    
    double sx = gx, sy = gy, sr = gr;
	double stability = 0;
	int code = 0;
	
	
	
	v_count = 0;
	ev = 0;
	ev2 = 0;
	
	sz = 0;
	sz2 = 0;
	
	dist = 0;

	rot_sum = 0;
	rot_sum2 = 0;
	
	work = 0;
	//last_time = ros::Time::now();

    
    st = ltime;
    
    do{
        en = ltime;
        //cout << en.toSec() << " --- " << st.toSec() << " ------- " << (en-st).toSec() << endl;
        ros::spinOnce();

    }
	while((en-st).toSec()<period);
	//while(en-st<gait[0]*count);
	
	//cerr << v_count << endl;
	
	double rot_dev = sqrt(rot_sum2/v_count);
	double power = work / (en-st).toSec();
	
	
	struct Result result; 
	result.distance = sqrt(abs(sx-gx) * abs(sx-gx) + abs(sy-gy) * abs(sy-gy));
	result.msecs = (en-st).toSec()*1000;
	
	result.wand = abs(dist - result.distance);//  /8;
	result.stability = 1/(result.wand / result.distance + rot_dev);//dist;
	
	if(result.distance<0.05)
		result.stability = 0;
	
	cerr << "Rotation Deviation >> " << rot_dev << endl;
	cerr << "Translation Total Variation >> " << result.wand << endl;
	cerr << "Power >> " << power << endl;
	//cerr << "Energy Efficieny >> " << power/result.distance << endl;
	cerr << "Energy Efficieny >> " << work/result.distance << endl;
	
	result.power = power;
	//result.efficiency = power/result.distance;
	result.efficiency = work/result.distance;
	
	result.code = code;
	result.rel = rot_dev;
	result.eff = sz/v_count;
    // for(int i=0;i<gait.size();i++)
    //     cout << gait[i] << " ";
    // cout << endl;
	show_result(result);
	return result;
}


#define SLOWEST sqrt(sqrt(sqrt(sqrt(2))))
#define SLOW sqrt(sqrt(sqrt(2)))
#define MOD sqrt(sqrt(2))
#define FAST sqrt(2)
#define FASTEST 2

#define WIDEST 5
#define WIDE 1
#define MED 0.5
#define NARROW 0.2
#define NARROWEST 0.05




double ar = 0.1;
double tr = 0.2;

double starter[25] = { NARROW, NARROW, NARROW, NARROW, 
                     WIDE, NARROW, NARROW, MED,MED,
                     WIDE, NARROW, NARROW, MED,MED,
                     WIDE, NARROW, NARROW, MED,MED,
                     WIDE, NARROW, NARROW, MED,MED,
                     MED };


double ranges[25] = { NARROW, NARROW, NARROW, NARROW, 
                     WIDE, NARROW, NARROW, MED,MED,
                     WIDE, NARROW, NARROW, MED,MED,
                     WIDE, NARROW, NARROW, MED,MED,
                     WIDE, NARROW, NARROW, MED,MED,
                     MED };
                      
double rate[25] = { SLOWEST, SLOWEST, SLOWEST, SLOWEST,
					SLOW, SLOWEST, SLOWEST, SLOWEST, SLOWEST,
					SLOW, SLOWEST, SLOWEST, SLOWEST, SLOWEST,
					SLOW, SLOWEST, SLOWEST, SLOWEST, SLOWEST,
					SLOW, SLOWEST, SLOWEST, SLOWEST, SLOWEST,
					SLOWEST };
					
void shrink(int times = 1) {
	for(int j = 0;j < times;j++)
		for(int i = 0;i<25;i++)
   			ranges[i] /= rate[i];  	
}

void expand(int times = 1) {
	for(int j = 0;j < times;j++)
		for(int i = 0;i<25;i++)
   			ranges[i] *= rate[i];  	
}

void reset() {
	for(int i = 0;i<25;i++)
   			ranges[i] = starter[i];
}

double changes[25];

vector <double> sample(vector <double> gait, double coef = 1)
{
	std::default_random_engine generator;
	
	//coef = 0.5;

    vector <double> ret;
    for(int i = 0;i<gait.size();i++){
        double rnd = ((double)(rand() % 10000 - 5000)/5000.0) * coef * ranges[i];
        changes[i] = rnd;
        
        //std::normal_distribution<double> distribution(0, coef * ranges[i]);
    	//changes[i] = changes[i] * 0.5 + distribution(generator);
    	
        
        ret.push_back(gait[i]+changes[i]);
    }
    
    double a[5];
    
    a[1] = gait[1];
    a[2] = gait[2];
    a[3] = gait[3];
    a[4] = 1 - a[1] - a[2] - a[3];
    
    double sum = 0;
    
    for(int i=1;i<=4;i++) {
    	double rnd = ((double)(rand() % 10000 - 5000)/5000.0) * coef * ranges[1];
    	a[i] += rnd;
    	a[i] = abs(a[i]);
    	sum += a[i];	
   	} 
    
    ret[0] = abs(ret[0]);
    
    ret[1] = a[1]/sum;
    ret[2] = a[2]/sum;
    ret[3] = a[3]/sum;
    
    if(ret[24]<0)
    	ret[24] = (-ret[24]);

    if(ret[24]>1)
    	ret[24] = abs(2-ret[24]);
    
    /*
    
    int k = rand()%6;
    int s = 0, e = 24;
    
    if(k == 4) {
    	e = 1;
    }
    else if(k == 5) {
    	s = 1;
    	e = 4;
    }
    else if(k<4) {
    	s = 4 + 5*k;
    	e = 9 + 5*k;
    }
    
    
    for(int i=0;i<gait.size();i++)
    	if(i<s || i>=e)
    		ret[i] = gait[i];	
   	*/
    
    return ret;
}


ofstream lout("new.log");
ros::Publisher stat_pub;

void log(Gait gait, Result result)
{
	lout << result.iteration << " ";	// 1: iteration no.
	lout << result.sample << " ";		// 2: sample no.
	
	for(int i=0;i<gait.size();i++)  	// 3 - 26: gait parameters
		lout << gait[i] << " ";
		
	lout << objective(result) << " ";   // 27: objective value
	lout << result.distance/((double)result.msecs/1000) << " "; // 28: velocity
	//lout << result.eff << " "; 		// 29: height
	//lout << result.msecs << " ";		// 30: duration
	
	lout << result.power << " "; 		// 29: power
	lout << result.efficiency << " ";	// 30: energy efficiency
	
	lout << result.rel << " "; 			// 31: Rotation Variance
	lout << result.distance << " ";		// 32: distance
	lout << result.stability << " ";	// 33: stability
	lout << result.wand << " ";			// 34: wandering
	
	for(int i=0;5*i+4<gait.size();i++)	// 35 - 46: end-effector poses
	{
		geometry_msgs::Pose pose;
		vector <double> vals;
	
		vals = vector<double> (gait.begin()+5*i+4,gait.begin()+5*i+9);
    	pose = fw_kin(vals, 1);
    	
    	
    	lout << pose.position.x << " ";
    	lout << pose.position.y << " ";
    	lout << pose.position.z << " ";
   	    	
    }
    
	
	lout << endl;
	
	daedalus_control::Stats m;
	
	m.iteration = result.iteration;
	m.sample = result.sample;
	
	m.distance = result.distance;
	m.stability = result.stability;
	m.power = result.power;
	m.efficiency = result.efficiency;
	m.rel = result.rel;
	m.eff = result.eff;
	m.wand = result.wand;

	m.msecs = result.msecs;
	m.code = result.code;
	
	stat_pub.publish(m);
	
}

Gait improve(Gait gait, double range, int count=10)
{
	cout << " ---------- " << endl;
	bool updated = false;
	struct Result result = measure(gait, 14);
	struct Result mresult = result;
	double max = objective(result);
	Gait mgait = gait;
	
	
	cout << "Starting at: " << max << endl;
	
	for(int i = 0; i<count; i++) {
		if(result.code)
			return mgait;
		
		cout << "sample #" << i+1 << ": " << endl;
		
		
			
		Gait sgait = sample(gait, range);
		result = measure(sgait, 10);
		double obj = objective(result);
		if(obj>max){
			max = obj;
			mgait = sgait;
			updated = true;
			mresult = result;
		}
		ros::spinOnce();
		
		
		// LOG TO FILE
		result.iteration = iteration;
		result.sample = i+1;
					
		log(sgait, result);
		
	}
	double obj = objective(mresult);
	
	mresult.iteration = iteration;
	mresult.sample = 0;
	log(mgait, mresult);
	
	if(!updated){
		//expand();
		//expand();
	}
	cout << "MAX: " << max << endl;
	return mgait;
}



bool accept(double current, double proposal, double temp)
{
	if(proposal > current) {
		return true;
	}
	
	if(temp == 0.0)
		return false;
	
	double prob = exp((proposal-current)/temp);
	double rnd = ((double)(rand() % 10000)/10000.0);
	
	cout << "--------------------- > " << prob << " < -------------------------" << endl;
		
	return rnd < prob;
}



#define COOLING_RATE 0.99;
#define MAX_TEMP 0.18

const int eps = 0.00001;

Gait sa(Gait gait)
{
	double temperature = MAX_TEMP; 
	
	
	Gait mgait = gait;
	Result result = measure(mgait, 10);
	double current = objective(result);

	double iteration = 0; 
	double sample_no = 0;	

	while (temperature > eps) {
		for (int N = 0; N < 100; N++) {
			
			
			
			Gait ngait = sample(mgait, 0.1);
		
			cout << "Sample #" << sample_no << " : " << endl;
			cout << "     TEMP = " << temperature << endl;
		
			result = measure(ngait, 10);
			double proposal = objective(result);
		
			
		
			if(accept(current, proposal, temperature)){
				iteration++;
				
				cout << "-------- Iteration " << iteration+1 << " --------- " << endl;
				
				sample_no = 0;
				
				current = proposal;
				mgait = ngait;		
			}
			
			result.iteration = iteration;
			result.sample = sample_no++;								
			
			log(ngait, result);
			
			
							
			ros::spinOnce();			
			
		}
		
		
		temperature *= COOLING_RATE; 
	}
	
	return mgait;
}



int cnt = -1;

vector <double> gait = {  4, 1./4., 1./4., 1./4., //4, 1./2., 1./2., 0.,
   		   				 /*
   		   				 1, 0.78539, 1, 1, 1,
   						 1, 0.78539, 1, 1, 1,
   						 1, 0.78539, 1, 1, 1,
   						 1, 0.78539, 1, 1, 1 }; 
   */
  /*
3.73627, 0.376042, 0.0316777, 0.184292, -3.03975, 1.23196, 0.70903, 0.279927, -0.638946, 1.51969, 0.913381, -0.358066, -0.433433, -0.134612, 1.69914, 0.47515, -0.62752, 0.836934, 1.06595, 1.57511, 0.0621386, 0.103335, 0.456057, -0.180589 }; 
*/

/* 

	Optimized gait

  4.21121 0.0595915 0.510107 0.417418 1.07496 0.563437 0.0583026 -0.429567 0.696167 1.16593 -0.319323 0.195219 -0.365963 -0.559309 -2.68633 0.886399 -0.443109 1.54016 0.141311 0.164454 0.690649 -9.28554e-05 -0.657426 0.527998
  
  3.73627 0.376042 0.0316777 0.184292 -3.03975 1.23196 0.70903 0.279927 -0.638946 1.51969 0.913381 -0.358066 -0.433433 -0.134612 1.69914 0.47515 -0.62752 0.836934 1.06595 1.57511 0.0621386 0.103335 0.456057 -0.180589
  
  Ball-shaped
  
  WS-interp:  
  
  
  3.86928, 0.112443, 0.191908, 0.0950858, -1.06539, 0.65172, -0.300694, 0.51027, 0.0931607, 0.0215899, 0.707807, -0.344592, 0.358875, 0.828518, 0.811406, 0.674052, -0.208718, 0.363337, 0.356942, 1.2095, 0.724139, 0.209163, 0.511284, -0.6381 };
  
*/

   						 // 0, 0.78539, 0, 0, 0,
   						 // 0, 0.78539, 0, 0, 0,
   						 // 0, 0.78539, 0, 0, 0,
   						 // 0, 0.78539, 0, 0, 0,
   						 // /* alpha = */ 0.5 }; 

  						 // 0, 0, 0, 0, 0,
   						//  0, 0, 0, 0, 0,
   						//  0, 0, 0, 0, 0,
   						//  0, 0, 0, 0, 0, 
   						// /* alpha = */ 0.5 }; 
   						 
   						 -0.5, 0, 0, 0, 0,
   						 -0.5, 0.78539, 0, 0, 0,
   						 0.5, 0.78539, 0, 0, 0,
   						 0.5, 0, 0, 0, 0,
   						 /* alpha = */ 0.5 }; 
   						 
   						 
  /* 						 
	3.96039, 0.0316431, 0.449657, 0.28469, -1.36206, 0.959054, 0.115154, -0.381541, -0.0120294, -0.61009, 0.194842, 0.106541, 0.582143, 1.19968, 1.22678, 0.785207, 0.500366, 1.41853, -1.29914, -0.192465, 1.08704, 0.0200003, -0.797839, -0.702157 };
	*/

   						/*
   						-1.18709, 0.703774, -0.183789, 0.102165, 0,
   						-0.010507, 0.704762, -0.181262, 0.100626, 0,
   						-0.743301, 0.467269, -0.465399, 0.14727, 0 ,
   						-0.743301, 0.467269, -0.465399, 0.14727, 0 };
   						*/
   						
/*
	vector <double> gait = { 2, 1./2., 1./4., 0.01,
   						-1.18709, 0.703774, -0.183789, 0.102165, 0,
   						-0.010507, 0.704762, -0.181262, 0.100626, 0,
   						-0.743301, 0.467269, -0.465399, 0.14727, 0 ,
   						-0.743301, 0.467269, -0.465399, 0.14727, 0 };
*/

Gait start = gait;
   						

Gait read(string file, int index, int sample = 0)
{
	ifstream fin(file);
	string tmp;
	vector <double> gait(25);
	
	while(true) {
		int iteration;
		fin >> iteration;
		//cerr << iteration << endl;
		if(iteration!=index) {
			
			if(!std::getline(fin, tmp))
				break;
			continue;
		}
		
		int number;
		fin >> number;
		if(number!=sample){
			if(!std::getline(fin, tmp))
				break;
			continue;
		}
		
		for (int i = 0;i < gait.size(); i++)
			fin >> gait[i];
		break;
	}
	return gait;
}

void replay(string file)
{
	ifstream fin(file);
	string tmp;
	vector <double> gait(24);
	
	int iteration = 0;
	while(true) {		
		int number;
		fin >> number;
		fin >> number;
		if(number!=0){
			if(!std::getline(fin, tmp))
				break;
			continue;
		}
		
		for (int i = 0;i < gait.size(); i++)
			fin >> gait[i];
		
		std::getline(fin, tmp);
			
		cout << endl << "__________ Iteration #" << iteration << "_________" << endl;				
		Result result = measure(gait, 10);
		result.iteration = iteration;
		
		log(gait, result);
		
		iteration++;
	}
}


void plan_callback(const moveit_msgs::MotionPlanRequest& msg)
{
	if(cnt == -1) {
		cnt++;
		return;
	}
	
	double *vals = new double[5];
	cout << "updating" << cnt+1 << endl;
	for(int i=0;i<5;i++) {
		vals[i] = msg.goal_constraints[0].joint_constraints[i].position;
		gait[cnt*5+4+i] = vals[i];
		cout << vals[i] << " ";
	}
	cout << endl;	
	cnt++;
}

ros::Publisher loop_pub;

void log_loop()
{
	run(gait, 1);
	geometry_msgs::PoseArray array;

	array.poses = *loop;

	loop_pub.publish(array);	
}

void log_all(string file)
{
	for(int i=0;i<800;i++){
		gait = read(file, i);
		log_loop();
	}

}

std::mutex m_;

class mylock
{
    

public:
    mylock()
    {
        cout << "acquiring" << endl;
        m_.lock();
    }
    ~mylock()
    {
        cout << "releasing" << endl;
        m_.unlock();
    }
};

bool is_locked = false;
int scount = 0;

bool handle_eval(daedalus_control::Eval::Request  &req,
         daedalus_control::Eval::Response &res)
{
    mylock lock;


    scount++;
    cout << "new request " << scount  << endl;

    while(is_locked)
        cout << scount << endl;
        ros::Duration(0.5).sleep();

    is_locked = true;

    
    // for(int i=0;i<50;i++){
    //     cout << lcount << endl;
    //     ros::Duration(0.5).sleep();
    // }

    // is_locked = false;
    // return true;


    
    // mylock scopeLock;
    // while(is_locked)
    //     ros::Duration(0.5).sleep();

    // is_locked = true;
    //////////////////////////////////////////////////

    Gait gait;
    for(int i=0;i<req.gait.size();i++){      // 3 - 26: gait parameters
        gait.push_back(req.gait[i]);
        // cout << gait[i] << " ";
    }
    // cout << endl;
    ROS_INFO("Evaluating the gait");
    Result result = instant_measure(gait, req.count);

    ROS_INFO("Evaluating the objective");

    res.objective = objective(result);
    
    res.iteration = result.iteration;
    res.sample = result.sample;
    
    res.distance = result.distance;
    res.stability = result.stability;
    res.power = result.power;
    res.efficiency = result.efficiency;
    res.rel = result.rel;
    res.eff = result.eff;
    res.wand = result.wand;

    res.msecs = result.msecs;
    res.code = result.code;

    ROS_INFO("sending back the result: [%d]", res.objective);

    // ros::Duration(0.5).sleep();

    // mtx.unlock();

    is_locked = false;

    scount--;

    return true;
}


int main(int argc, char **argv)
{
	/*
	Eigen::Affine3d ee_pose;
	cout << ee_pose.matrix() << endl;
	Eigen::Matrix<double, 4, 4>  m = ee_pose.matrix();

	for(int i=0;i<4;i++) {
		for(int j=0;j<4;j++)
			cout << m(i,j) << " " ;
		cout << endl;
	}
	*/

	geometry_msgs::Pose pose;

	double vals[5] = {0,0,0,0,0};

	for (int i=0;i<100;i++){
		// pose = forwardK(vals);
		vector <double> vs = {0,0,0,0,0};

		pose = fw_kin(vs, 1);
		cout << pose << endl << endl;

		vector<double> last;
		for(int i = 0;i<5;i++)
			last.push_back(0);

		vector<double> js = inverseK(pose, last);
		cout << " -------- ";
		for(int i=0;i<js.size();i++)
			cout << js[i] << " ";

		cout << endl;
	}
	

	

    ros::init(argc, argv, "trajectory_executer");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(10);
    spinner.start();

    follow = new ros::Publisher[20];
    
    stat_pub = node_handle.advertise<daedalus_control::Stats>("/daedalus/stats", 1);

    loop_pub = node_handle.advertise<geometry_msgs::PoseArray>("/loop", 1);
    
    last_time = ros::Time::now();
    
    for(int i=1;i<=4;i++)
        follow[i]=node_handle.advertise<trajectory_msgs::JointTrajectory>("/daedalus/LEG" + std::to_string(i) + "_controller/command", 10);

    ros::Subscriber model_sub = node_handle.subscribe("/gazebo/model_states", 1, stateCallback);
    
    ros::Subscriber plan_sub = node_handle.subscribe("/move_group/motion_plan_request", 1, plan_callback);
    
    ros::Subscriber effort_sub = node_handle.subscribe("/daedalus/joint_states", 1, effort_callback);
    ros::Subscriber effort_sub_2 = node_handle.subscribe("/joint_states", 1, effort_callback);

    // ros::Subscriber req_sub = node_handle.subscribe("/request", 1, handle_request);
    // res_pub = node_handle.advertise<geometry_msgs::PoseArray>("/loop", 1);

    
    rml = new robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model = rml->getModel();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("LEG1"); 
    
    
    client = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    //client = node_handle.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    
    //validSpace(1);
    
    //vector <double> gait = {4, 0.12, -0.005, 0.005, 0.12, 0.18, 0.15};
    
    //vector <double> gait = {3.53679, 0.112126, -0.0221317, 0.00784371, 0.0561703, 0.203596, 0.204726};   
    

    //vector <double> gait = {3.9162, 0.105971, 0.0201888, -0.00657353, 0.05955, 0.218181, 0.165312};
    //vector <double> gait = {3.58855, 0.109522, -0.00298116, 0.0235867, 0.0446852, 0.226447, 0.228663 };
    
    ros::ServiceServer service = node_handle.advertiseService("evaluate", handle_eval);
    
    srand(time(0));
    
    /*
    fw_kin(vector<double> (gait.begin()+4,gait.begin()+9), 1);
    
    while(ros::ok())
    {
    	geometry_msgs::Pose pose;
    	
    	cin >> pose.position.x;
		cin >> pose.position.y;
		cin >> pose.position.z;
    	
    	cin >> pose.orientation.x;
		cin >> pose.orientation.y;
		cin >> pose.orientation.z;
		cin >> pose.orientation.w;		
		
		f<double> (inv_kin(pose,1));
    }
	*/
	
	// 0.267127 0.125996 0.131267 0.714072 -0.518984 -0.46614 0.058912
	// 0.267127 0.115996 0.131267 0.714072 -0.518984 -0.46614 0.058912

	
	int time = 10;

    while(ros::ok())
    {
        if (/*kbhit()*/ true)
        {
            string inp;
            
            cin >> inp;
            
            char key=inp[0];//getchar();
            
            if(inp=="shrink") {
            	string arg1;
            	cin >> arg1;
            	shrink(std::stoi(arg1));
            	continue;
            }
            
            if(inp=="steps") {
            	string arg1;
            	cin >> arg1;
            	steps = std::stoi(arg1);
            	continue;
            }

            if(inp=="alpha") {
            	string arg1;
            	cin >> arg1;
            	galpha = std::stof(arg1);
            	continue;
            }
            
            if(inp=="expand") {
            	string arg1;
            	cin >> arg1;
            	expand(std::stoi(arg1));
            	continue;
            }

            if(inp=="reset") {
            	reset();
            	continue;
            }

            if(inp=="loop") {
            	log_loop();
            	continue;
            }

            if(inp=="loopall") {
            	string arg1;
            	cin >> arg1;
            	log_all(arg1);
            	continue;
            }
            	
            
            if(inp=="replay") {
            	string arg1;
            	cin >> arg1;
            	replay(arg1);
            	continue;
            }
            
            if(inp=="read") {
            	string arg1, arg2;
            	cin >> arg1 >> arg2;
            	gait = read(arg1, std::stoi(arg2, NULL));
            	continue;
            }
            
            if(inp=="time") {
            	string arg1;
            	cin >> arg1;
            	time = std::stoi(arg1);
            	continue;
            }

            if(inp=="fake") {
            	double arg1, arg2;
            	cin >> arg1 >> arg2;
            	fake_measure(arg1, arg2);
            	continue;
            }
            
            
            if(key=='j' || key=='J')
            	steps = 1;
            
            if(key=='w' || key=='W')
            	steps = 100;
            
            if(key==' ') {
            	string str;
            	cin >> str;
            	for(int i=0;i<str.size();i++)
            		;
            	cout << " --> " << str;          	            	
            }
            
            
            if(key == 's' || key == 'S'){
            	gait = sa(gait);
            }
            
            if(key == 't' || key == 'T'){
            	run(gait, 1);
            }
            
            
            
            if(key == 'o' || key == 'O'){
            	double SAMPLING_RANGE = 0.5;
	            for(iteration=0;iteration<800;iteration++){
		            cout << "-------- Iteration " << iteration+1 << " --------- " << endl;
		            cout << "SAMPLE RANGE: " << SAMPLING_RANGE << endl;
		            gait=improve(gait, SAMPLING_RANGE);
		            //SAMPLING_RANGE/=sqrt(sqrt(sqrt(2)));
		            shrink();
		            
		            // if(iteration>=40 && iteration%20==0)
		            // 	expand(10);

		            if(iteration>=35 && iteration%20==19)
		            	expand(19);
		            	
	            }
	            
            }
            
            if(key >= '1' && key <= '4'){
            	cout << "start playing" << endl;
            	v_count = 0;
            	run(gait, time * int(key-'0'));
            	
            	//play_gait(10);
                //playAll();
			}
            
            if(key == 'p' || key == 'P'){
            	cout << "start playing" << endl;
            	v_count = 0;
            	Result result = measure(gait, 10);
            	cout << result.distance << "m in " << (double)result.msecs/1000 << "secs" << endl;
            	cout << "Velocity: " << result.distance/((double)result.msecs/1000) << endl;
            	//play_gait(10);
                //playAll();
			}
			
			if(key == 'i' || key == 'I'){
            	v_count = 0;
	          	measure(gait, 10);            	
	          	cout << "speed: " << ev << endl;
            	Result result = measure(gait, 10);
            	cout << result.distance << "m in " << (double)result.msecs/1000 << "secs" << endl;
            	cout << "Velocity: " << result.distance/((double)result.msecs/1000) << endl;
            	//play_gait(10);
                //playAll();
			}
		
        }  

        ros::getGlobalCallbackQueue()->callOne(ros::WallDuration(0.5));
        // ros::spinOnce();
    }
    

    ros::shutdown();  
    return 0;
}

