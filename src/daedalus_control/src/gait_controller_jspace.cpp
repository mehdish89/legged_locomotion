#include <iostream>
#include <fstream>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

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

void moveLegTo(robot_state::RobotStatePtr state, string group, double dur = 2)
{
    int leg = (int)(group[3]-'0');
    const robot_state::JointModelGroup* jmg = state->getJointModelGroup(group);
    const std::vector<std::string> &joint_names = jmg->getJointModelNames();    

    std::vector<std::string> jn;

    for(int i=1;i<joint_names.size();i++)
        jn.push_back(joint_names[i]);

    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = jn;
    traj.points.push_back(getTrajectoryPoint(state, group, dur));
    
    follow[leg].publish(traj);
}

void moveTo(robot_state::RobotStatePtr state, double dur = 2)
{
    for(int i=1;i<=4;i++) {
        string group = "LEG" + std::to_string(i);
        moveLegTo(state, group, 2);
    }
}

void move(vector <robot_state::RobotStatePtr> states, string group, double dur = 2, int count = 1)
{
    int leg = (int)(group[3]-'0');
    trajectory_msgs::JointTrajectory traj;

    for(int j=0;j<count;j++){
        for(int i=0;i<states.size();i++){
            const robot_state::JointModelGroup* jmg = states[i]->getJointModelGroup(group);
            const std::vector<std::string> &joint_names = jmg->getJointModelNames();    

            std::vector<std::string> jn;

            for(int i=1;i<joint_names.size();i++)
                jn.push_back(joint_names[i]);

            
            traj.joint_names = jn;
            traj.points.push_back(getTrajectoryPoint(states[i], group, j*dur+dur*(i+1)/states.size()));
        }    
    }
    follow[leg].publish(traj);
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


const int steps = 1;

vector <robot_state::RobotStatePtr> interpolate(int leg, geometry_msgs::Pose from, geometry_msgs::Pose to)
{
    vector< robot_state::RobotStatePtr> traj;
    robot_state::RobotStatePtr state(new robot_state::RobotState(kinematic_model));

    string ee_link_name = "leg"+std::to_string(leg)+"link5";
    const robot_state::LinkModel * ee_link =  state->getLinkModel(ee_link_name);

    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("LEG"+std::to_string(leg));
    
    
    int count = 0;
    for(int i=0;i<steps;i++) {
        
        geometry_msgs::Pose pose = from;
        pose.position.x += (to.position.x-from.position.x) * (i+1) / steps;
        pose.position.y += (to.position.y-from.position.y) * (i+1) / steps;
        pose.position.z += (to.position.z-from.position.z) * (i+1) / steps;
    
        Eigen::Affine3d ee_pose;
        tf::poseMsgToEigen(pose, ee_pose);    
        //tf::poseMsgToEigen(to, ee_to);
        bool found_ik = state->setFromIK(joint_model_group, ee_pose, 10, 0.1);
        
        if(found_ik) {     
            robot_state::RobotStatePtr new_state(new robot_state::RobotState(*state.get()));
            invert(new_state);            
            traj.push_back(new_state); 
            count++;
        }
    }
    //cout << "percent: " << double(count)*100/double(steps) << endl;

    //if(result<0.7)
      //  traj.clear();
    
    return traj;
}

void cycle(int leg, vector<geometry_msgs::Pose> poses, vector <double> times, int count = 1, double delay = 0)
{
    string group = "LEG" + std::to_string(leg);
    trajectory_msgs::JointTrajectory traj;
    double timestamp = delay;    

    for(int i = 0;i<count;i++){
        int size = poses.size();
        
        for(int j=0;j<size;j++){
            vector< robot_state::RobotStatePtr> states;
            double dur;
            int current, next;
            
                        
            if(leg == 1 || leg == 2){            
                states = interpolate(leg, poses[j], poses[(j+1)%size]);
                dur = times[j]/states.size();
            }
            else {
                states = interpolate(leg, poses[(size-j)%size], poses[(size-j-1)%size]); 
                dur = times[(size-j-1)%size]/states.size();
            }

            
        
            for(int k=0;k<states.size();k++){
                timestamp += dur;
                const robot_state::JointModelGroup* jmg = states[k]->getJointModelGroup(group);
                const std::vector<std::string> &joint_names = jmg->getJointModelNames();    

                std::vector<std::string> jn;

                for(int i=1;i<joint_names.size();i++)
                    jn.push_back(joint_names[i]);

                traj.joint_names = jn;
                if(timestamp>0)                
                    traj.points.push_back(getTrajectoryPoint(states[k], group, timestamp));
            }
        }
    }
        
    follow[leg].publish(traj);
}




geometry_msgs::Pose fw_kin(vector <double> joints)
{
	string group = "LEG1";
    robot_state::RobotStatePtr state(new robot_state::RobotState(kinematic_model));
   	const robot_state::JointModelGroup* jmg = state->getJointModelGroup(group);
	state->setJointGroupPositions(jmg, joints);
	const Eigen::Affine3d &ee_state = state->getGlobalLinkTransform("leg1link5");
	
	geometry_msgs::Pose pose;
	tf::poseEigenToMsg(ee_state, pose);    
	return pose;
}

vector <double> inv_kin(geometry_msgs::Pose pose)
{
	string group = "LEG1";
    robot_state::RobotStatePtr state(new robot_state::RobotState(kinematic_model));
   	const robot_state::JointModelGroup* jmg = state->getJointModelGroup(group);
   	
	vector <double> vals;
	Eigen::Affine3d ee_pose;
    tf::poseMsgToEigen(pose, ee_pose);    
    //tf::poseMsgToEigen(to, ee_to);
    bool found_ik = state->setFromIK(jmg, ee_pose, 10, 0.1);
	if(found_ik)
	  	state->copyJointGroupPositions(jmg, vals);  
	return vals;
}


void run(double t, double y, double z1, double z2, double x1, double x2, double x3, int count)
{
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1; 
    
    pose.position.x = x1;
    pose.position.y = y;
    pose.position.z = z1;

    vector<geometry_msgs::Pose> poses;
    vector <double> times;
    
    poses.push_back(pose);
    times.push_back(t/2);    
    
    pose.position.x = x2;
    //pose.position.z = z1;

    poses.push_back(pose);
    times.push_back(t/4);    
    
    pose.position.x = x3;
    pose.position.z = z2;

    poses.push_back(pose);
    times.push_back(t/4); 


    for(int i=1;i<=4;i++)    
        if(i%2==0)
            cycle(i, poses, times, count, -2);
        else
            cycle(i, poses, times, count, 0);
}



void run(vector<double> gait, int count)
{
	double t =  gait[0];
	geometry_msgs::Pose pose;
	vector<geometry_msgs::Pose> poses;
	vector <double> times;
	
	vector <double> vals;
	
	vals = vector<double> (gait.begin()+1,gait.begin()+6);
    pose = fw_kin(vals);
    poses.push_back(pose);
    times.push_back(t/2);    
    
    vals = vector<double> (gait.begin()+6,gait.begin()+11);
    pose = fw_kin(vals);
    poses.push_back(pose);
    times.push_back(t/4);    
    
    vals = vector<double> (gait.begin()+11,gait.begin()+16);
    pose = fw_kin(vals);
    poses.push_back(pose);
    times.push_back(t/4); 
    
    for(int i=1;i<=4;i++)    
        if(i%2==0)
            cycle(i, poses, times, count, -2);
        else
            cycle(i, poses, times, count, 0);
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
	while(en-st<20);
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



double ar = 0.05;
double ranges[16] = { 0.1, 
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

    rml = new robot_model_loader::RobotModelLoader("robot_description");
    kinematic_model = rml->getModel();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("LEG1"); 
    
    
    //validSpace(1);
    
    //vector <double> gait = {4, 0.12, -0.005, 0.005, 0.12, 0.18, 0.15};
    
    //vector <double> gait = {3.53679, 0.112126, -0.0221317, 0.00784371, 0.0561703, 0.203596, 0.204726};   
    

    //vector <double> gait = {3.9162, 0.105971, 0.0201888, -0.00657353, 0.05955, 0.218181, 0.165312};
    //vector <double> gait = {3.58855, 0.109522, -0.00298116, 0.0235867, 0.0446852, 0.226447, 0.228663 };
    
    vector <double> gait = {4,
    						-1.18709, 0.703774, -0.183789, 0.102165, 0,
    						-0.010507, 0.704762, -0.181262, 0.100626, 0,
    						-0.743301, 0.467269, -0.465399, 0.14727, 0 };
    
    srand(time(0));
    //measure(sample(gait, 2), 10);


    while(ros::ok())
    {
        //cout << "looping" << endl;
        /*
        double x,y,z;
        
        cin >> x >> y >> z;
        geometry_msgs::Pose pose;
		pose.orientation.x = 0;
		pose.orientation.y = 0;
		pose.orientation.z = 0;
		pose.orientation.w = 1; 
		
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;
		
		vector <double> vals = inv_kin(pose);
		for(int i=0;i<vals.size();i++)
			cout << vals[i] << " ";
		cout << endl;
		
		continue;
		*/
		    
        if (kbhit())
        {
            //cout << "kbhit" << endl;
            //render_menu();
	        
            char key=getchar();
            /*            
            if(key == ' ')
                capture();
            if(key == 'r' || key == 'R')
                restart();
            */
            if(key == 'o' || key == 'O'){
            	double SAMPLING_RANGE = 4;
	            for(int i=0;i<8;i++){
		            cout << "-------- Iteration " << i+1 << " --------- " << endl;
		            cout << "SAMPLE RANGE: " << SAMPLING_RANGE << endl;
		            gait=improve(gait, SAMPLING_RANGE);
		            SAMPLING_RANGE/=sqrt(2);
	            }
            }
            /*
            if(key == 'm' || key == 'M'){
            	if(mode == ROBOT)
            		mode = SIMULATION;
            	else
            		mode = ROBOT;
            		
            	cout << " Mode changed to: " << (char)mode << endl;
            }
            
                
            if(key == 'a' || key == 'A')
                apply_gait(poses);
            
            if(key=='-')
                stop();

            if(key=='0')
                play(-1);

            if(key<='4' && key>='1')
                toggle(key-'0');         */

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
			

            // END-EFFECTOR = 3.63236 0.113067 -0.010082 0.00162227 0.0499788 0.231563 0.216521
            //     JOINT    = 3.64143 0.109732 -0.0163256 0.0193083 0.0659698 0.222688 0.176315 

            //                3.53679 0.112126 -0.0221317 0.00784371 0.0561703 0.203596 0.204726
            //  BEST EE     = 3.54248 0.110877 -0.010282  0.0152681  0.0681082 0.235961 0.226099
            
            //  BEST JS     = 3.9162  0.105971  0.0201888 -0.00657353 0.05955  0.218181 0.165312
            // EE 1.62      = 3.58855 0.109522 -0.00298116 0.0235867 0.0446852 0.226447 0.228663 
            /*
            if(key == 'g' || key == 'G')
                play(curser);

            if(key == 's' || key == 'S')
                showing = !showing;          

            if(key == 'q' || key == 'Q')
                break;
            
            */            

            /*
            if(key == 'l' || key == 'L')
                load(context);

            if(key == 'w' || key == 'W')
                save();
            
            
        
            if(key == 'f' || key == 'F'){
                string fname;
                cout << "Please ENTER the file name: ";
                cin >> fname;
                context = fname.c_str();
                cout << "Context has been changed to \" " << fname << " \" " << endl;
            }

            if(key == ',' || key == '.')
                change(key);
            
            if(key == 'd' || key == 'D')
                remove(curser);
            */
        }  
        ros::spinOnce();
    }
    

    ros::shutdown();  
    return 0;
}

