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

#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/JointState.h"

#include "daedalus_control/Stats.h"

#include <eigen_conversions/eigen_msg.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>


using namespace std;


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

double gx, gy, gz, gr;


double lx = 0 , ly = 0, lt = 0, lv = 0;
double ev2 = 0, ev = 0, edv = 0;
int v_count = 0;
double vel = 0;
double real_vel = 0;
double rel;
double meff = 0;

bool new_vel = false;

void stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	gx = msg->pose[1].position.x;
	gy = msg->pose[1].position.y;
	gz = msg->pose[1].position.z;
	gr = msg->pose[1].orientation.z;
	double vx = msg->twist[1].linear.x;
	double vy = msg->twist[1].linear.y;
	real_vel = sqrt(vx*vx+vy*vy);
	
	/*
	daedalus_control::Stats m;
	m.velocity = real_vel;
	m.velocity_variation = rel;
	m.leg_x = leg_x - gx ;
	m.leg_y = leg_y - gy ;
	m.leg_z = leg_z - gz ;
	//if(v>0.001)
		stat_pub.publish(m);
	*/
}

struct Result {
	double distance;
	double racc;
	double rel;
	double eff;
	int msecs;
	int code;
};

double objective(Result result)
{
	double vel = result.distance/((double)result.msecs/1000);
	//return result.distance - result.racc * 0.5;
	return vel * 10 + result.distance * 0.25 - result.racc * 0.1 - result.rel * 1.5 - result.eff * 0.01;
}

void show_result(Result result)
{
	cout << "Objective value = " << objective(result) << endl;
	cout << "Velocity = " << result.distance/((double)result.msecs/1000) << endl;
	cout << "Effort = " << result.eff << endl;
	cout << "Distance = " << result.distance << endl;
	cout << "Time = " << result.msecs << endl;
	cout << "Velocity Variation = " << result.rel << endl;
	cout << "Rotation Error = " << result.racc << endl;
	cout << endl;
}


const int steps = 10;

vector <trajectory_msgs::JointTrajectoryPoint> interpolate(int leg, geometry_msgs::Pose from, geometry_msgs::Pose to, double dur = 1, double delay = 0)
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
    for(int i=0;i<steps;i++) {
        
        double timestamp = (i+1) * dur / steps + delay;
        
        //if(timestamp < 0)
        	//continue;
        
        geometry_msgs::Pose pose = from;
        pose.position.x += (to.position.x-from.position.x) * (i+1) / steps;
        pose.position.y += (to.position.y-from.position.y) * (i+1) / steps;
        pose.position.z += (to.position.z-from.position.z) * (i+1) / steps;
        
        /////////////
        
        //pose.orientation.x += (to.orientation.x-from.orientation.x) * (i+1) / steps;
        //pose.orientation.y += (to.orientation.y-from.orientation.y) * (i+1) / steps;
        //pose.orientation.z += (to.orientation.z-from.orientation.z) * (i+1) / steps;
        //pose.orientation.w += (to.orientation.w-from.orientation.w) * (i+1) / steps;
        
        
        
        if(i+1<steps) {
		    pose.orientation.x = 0;
		    pose.orientation.y = 0;
		    pose.orientation.z = 0;
		    pose.orientation.w = 0;
		}
        
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
    cout << "percent: " << double(count)*100/double(steps) << endl;

    //if(result<0.7)
      //  traj.clear();
    
    return traj;
}

trajectory_msgs::JointTrajectory cycle(int leg, vector<geometry_msgs::Pose> poses, vector <double> times, int count = 5, double delay = 0)
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
    
    double timestamp = delay;    

    for(int i = 0;i<=count;i++){
        int size = poses.size();
        
        for(int j=0;j<size;j++){
            vector< trajectory_msgs::JointTrajectoryPoint> states;
            double dur;
            int current, next;
            
                        
            if(leg == 1 || leg == 2 ){            
                states = interpolate(leg, poses[j], poses[(j+1)%size], times[j], timestamp);
                timestamp += times[j];
            }
            else {
                states = interpolate(leg, poses[(size-j)%size], poses[(size-j-1)%size], times[(size-j-1)%size], timestamp); 
                timestamp += times[(size-j-1)%size];
            }
            
            for(int k=0;k<states.size();k++){             
                if(states[k].time_from_start < ros::Duration(count*t))
                	traj.points.push_back(states[k]);
            }
        }
    }
    
    
    ros::Duration dur = traj.points[traj.points.size()-1].time_from_start - traj.points[0].time_from_start;
    
    
    
    /*
    for(int i=0;i<traj.points.size();i++){
    	
    	trajectory_msgs::JointTrajectoryPoint point = trajectory_msgs::JointTrajectoryPoint(traj.points[i]);    
    	
    	//cout << point.time_from_start << endl;	  	
    	
    	if(point.time_from_start<ros::Duration(0)){
    		cout << point.time_from_start << endl;
    		point.time_from_start += dur;
    		
    		traj.points.push_back(point);
    		
    	}
    	//else
    		//break;    	
    	
    }*/
    
    
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


vector <double> inv_kin(geometry_msgs::Pose pose, int leg)
{
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
	leg = 1;
	string group = "LEG" + std::to_string(leg);
	string ee_link_name = "leg"+std::to_string(leg)+"link5";
    robot_state::RobotStatePtr state(new robot_state::RobotState(kinematic_model));
   	const robot_state::JointModelGroup* jmg = state->getJointModelGroup(group);
	state->setJointGroupPositions(jmg, joints);
	const Eigen::Affine3d &ee_state = state->getGlobalLinkTransform(ee_link_name);
	
	geometry_msgs::Pose pose;
	tf::poseEigenToMsg(ee_state, pose);    
	
	f<double>(inv_kin(pose, leg));
	cout << "-----------" << endl;
	cout << pose << endl;
	
	return pose;
}




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
	
	//cout << t1 << endl << t2  << endl << t3 << endl << t4 << endl;
	
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
            traj[i] = cycle(i, poses, times, count, -t/2);
        else
			traj[i] = cycle(i, poses, times, count, 0);
	
			
	for(int i=1;i<=4;i++)    
		follow[i].publish(traj[i]);
	//cycle(leg, poses, times, count, (leg%2)*(-t/2));
	
}


Result measure(vector<double> gait, int count)
{
    double sx = gx, sy = gy, sr = gr;
	double racc = 0;
	int code = 0;
	unsigned long long st, en;
	st = get_secs();
	
	v_count = 0;
	ev = 0;
	ev2 = 0;

    /*run(gait[0], 
        gait[1],
        gait[2],
        gait[3],
        gait[4],
        gait[5],
        gait[6],
        count );*/
    run(gait, count);

    
    do{
        en = get_secs();
        ros::spinOnce();
    }
	//while(en-st<20);
	while(en-st<gait[0]*count);
	struct Result result; 
	result.distance = sqrt(abs(sx-gx) * abs(sx-gx) + abs(sy-gy) * abs(sy-gy));
	result.msecs = (en - st)*1000;
	result.racc = racc;
	result.code = code;
	result.rel = rel;
	result.eff = meff;
    for(int i=0;i<gait.size();i++)
        cout << gait[i] << " ";
    cout << endl;
	show_result(result);
	return result;
}



double ar = 0.1;
double ranges[24] = { 0.1, 0.1, 0.1, 0.1, 
                     ar,ar,ar,ar,ar,
                     ar,ar,ar,ar,ar,
                     ar,ar,ar,ar,ar,
                     ar,ar,ar,ar,ar
                      };

vector <double> sample(vector <double> gait, double coef)
{
    vector <double> ret;
    for(int i = 0;i<gait.size();i++){
        double rnd = ((double)(rand() % 10000 - 5000)/5000.0) * coef * ranges[i];
        ret.push_back(gait[i]+rnd);
    }
    
    double a[5];
    
    a[1] = gait[1];
    a[2] = gait[2];
    a[3] = gait[3];
    a[4] = 1 - a[1] - a[2] - a[3];
    
    double sum;
    
    for(int i=1;i<=4;i++) {
    	double rnd = ((double)(rand() % 10000 - 5000)/5000.0) * coef * ranges[i-1];
    	a[i] += rnd;
    	a[i] = abs(a[i]);
    	sum += a[i];	
   	} 
    
    ret[0] = abs(ret[0]);
    
    ret[1] = a[1]/sum;
    ret[2] = a[2]/sum;
    ret[3] = a[3]/sum;
    
    return ret;
}


ofstream lout("gait.log");

Gait improve(Gait gait, double range, int count=10)
{
	cout << " ---------- " << endl;
	bool updated = false;
	struct Result result = measure(gait, 10);
	struct Result mresult = result;
	double max = objective(result);
	Gait mgait = gait;
	
	
	cout << "Starting at: " << max << endl;
	
	for(int i = 0; i<count; i++) {
		if(result.code)
			return mgait;
		
		cout << "ROUND " << i+1 << ": " << endl;
			
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
		
		lout << i+1 << " ";
		lout << obj << " ";
		lout << result.distance/((double)result.msecs/1000) << " ";
		lout << result.distance << " ";
		lout << result.msecs << " ";
		lout << result.racc << " ";
		lout << endl;
		
	}
	double obj = objective(mresult);
	lout << 0 << " ";
	lout << obj << " ";
	lout << mresult.distance/((double)mresult.msecs/1000) << " ";
	lout << mresult.distance << " ";
	lout << mresult.msecs << " ";
	lout << mresult.racc << " ";
	lout << endl;
	
	//if(!updated)
		//SAMPLING_RANGE*=2;
	
	cout << "MAX: " << max << endl;
	return mgait;
}


int cnt = -1;



vector <double> gait = { 3, 1./2., 1./4., 0.05,
   						-1.18709, 0.703774, -0.183789, 0.102165, 0,
   						-0.010507, 0.704762, -0.181262, 0.100626, 0,
   						-0.743301, 0.467269, -0.465399, 0.14727, 0 ,
   						-0.743301, 0.467269, -0.465399, 0.14727, 0 };
   						
/*
	vector <double> gait = { 2, 1./2., 1./4., 0.01,
   						-1.18709, 0.703774, -0.183789, 0.102165, 0,
   						-0.010507, 0.704762, -0.181262, 0.100626, 0,
   						-0.743301, 0.467269, -0.465399, 0.14727, 0 ,
   						-0.743301, 0.467269, -0.465399, 0.14727, 0 };
*/
   						

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_executer");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(4);
    spinner.start();

    follow = new ros::Publisher[20];
    
    for(int i=1;i<=4;i++)
        follow[i]=node_handle.advertise<trajectory_msgs::JointTrajectory>("/daedalus/LEG" + std::to_string(i) + "_controller/command", 10);

    ros::Subscriber model_sub = node_handle.subscribe("/gazebo/model_states", 1, stateCallback);
    
    ros::Subscriber plan_sub = node_handle.subscribe("/move_group/motion_plan_request", 1, plan_callback);

    rml = new robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model = rml->getModel();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("LEG1"); 
    
    
    //validSpace(1);
    
    //vector <double> gait = {4, 0.12, -0.005, 0.005, 0.12, 0.18, 0.15};
    
    //vector <double> gait = {3.53679, 0.112126, -0.0221317, 0.00784371, 0.0561703, 0.203596, 0.204726};   
    

    //vector <double> gait = {3.9162, 0.105971, 0.0201888, -0.00657353, 0.05955, 0.218181, 0.165312};
    //vector <double> gait = {3.58855, 0.109522, -0.00298116, 0.0235867, 0.0446852, 0.226447, 0.228663 };
    
    
    
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
	

    while(ros::ok())
    {
        if (kbhit())
        {
            char key=getchar();
            
            if(key == 'o' || key == 'O'){
            	double SAMPLING_RANGE = 8;
	            for(int i=0;i<20;i++){
		            cout << "-------- Iteration " << i+1 << " --------- " << endl;
		            cout << "SAMPLE RANGE: " << SAMPLING_RANGE << endl;
		            gait=improve(gait, SAMPLING_RANGE);
		            SAMPLING_RANGE/=sqrt(sqrt(2));
	            }
            }
            
            if(key >= '1' && key <= '4'){
            	cout << "start playing" << endl;
            	v_count = 0;
            	run(gait, 5, key-'0');
            	
            	//play_gait(10);
                //playAll();
			}
            
            if(key == 'p' || key == 'P'){
            	cout << "start playing" << endl;
            	v_count = 0;
            	Result result = measure(gait, 1);
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
        ros::spinOnce();
    }
    

    ros::shutdown();  
    return 0;
}

