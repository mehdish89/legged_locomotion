#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include <cmath>

#include <functional>
#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "dynamixel_msgs/JointState.h"
#include "control_msgs/JointControllerState.h"
#include "dynamixel_controllers/TorqueEnable.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#include "sensor_msgs/JointState.h"

#include "daedalus_control/Stats.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>

using namespace std;

typedef vector<double> Pose;
typedef vector <Pose> Gait;

double SAMPLING_RANGE = 0.4;

ros::Publisher *pub;
ros::Publisher *spub;
ros::Publisher stat_pub;
ros::ServiceClient *srv;


int curser = 0;
int fcount = 0;

double vals[20];

double gx, gy, gz, gr;

Gait poses;

bool recording = false;
bool showing = false;

bool state[4] = {false, false, false, false};

const char* context = ".";

const string topics[] = {
  
    "top_left_1",
    "top_left_2",
    "top_left_3",
    "top_left_4",    
    "top_left_5",    

    "top_right_1",
    "top_right_2",
    "top_right_3",
    "top_right_4",
    "top_right_5",

    "bottom_left_1",
    "bottom_left_2",
    "bottom_left_3",
    "bottom_left_4",
    "bottom_left_5",

    "bottom_right_1",
    "bottom_right_2",
    "bottom_right_3",
    "bottom_right_4",
    "bottom_right_5"

};

const string sim_topics[] = {
  
    "leg2joint1",
    "leg2joint2",
    "leg2joint3",
    "leg2joint4",
    "leg2joint5",

    "leg4joint1",
    "leg4joint2",
    "leg4joint3",
    "leg4joint4",
    "leg4joint5",
    
    "leg1joint1",
    "leg1joint2",
    "leg1joint3",
    "leg1joint4",    
    "leg1joint5",    

    "leg3joint1",
    "leg3joint2",
    "leg3joint3",
    "leg3joint4",
    "leg3joint5"    

};


enum Mode {
	ROBOT = 'R',
	SIMULATION = 'S'
};

Mode mode = SIMULATION;

int char_count = 0;

void print(const char* str)
{
	string s(str);
	cout << s;
	char_count+=s.size();
}


void flush()
{
	for(int i = 0;i<char_count;i++)
		cout << "\b";
	for(int i = 0;i<char_count;i++)
		cout << " ";
	for(int i = 0;i<char_count;i++)
		cout << "\b";
	char_count = 0;
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

int indexOf(string topic)
{
    for(int i=0;i<20;i++)
        if(topic == topics[i])
            return i;
    return 0;
}

double eff = 0;

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	double effort = 0;
	for(int i=0;i<20;i++)
		effort += abs(msg->effort[i]);
	effort/=20;
	eff = effort;
	//cout << effort << endl;
}


double leg_x = 0, leg_y = 0, leg_z = 0;

void linkCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
	leg_x = msg->pose[6].position.x;
	leg_y = msg->pose[6].position.y;
	leg_z = msg->pose[6].position.z;
	
}


double lx = 0 , ly = 0, lt = 0, lv = 0;
double ev2 = 0, ev = 0, edv = 0;
int v_count = 0;
double vel = 0;
double real_vel = 0;
double rel;
double meff = 0;

bool new_vel = false;

void timerCallback(const ros::TimerEvent&)
{
	if(lx == 0) {
		lx = gx;
		ly = gy;
	}
	double x = gx, y = gy, t = get_msecs();
	double d = sqrt((x-lx)*(x-lx)+(y-ly)*(y-ly));
	double dt = t - lt;
	double v = d*1000/dt;
	new_vel = true;
	
	vel = v;
	
	ev = (ev * v_count + v)/(v_count+1);
	ev2 = (ev2 * v_count + v*v)/(v_count+1);
	edv = (edv * v_count + abs(v-lv))/(v_count+1);
	
	//cout << eff << endl;
	meff = (meff * v_count + eff)/(v_count+1);
	
	double var = ev2 - ev*ev;
	//rel = edv/ev;//
	rel = sqrt(var)/ev;
	/*
	daedalus_control::Stats msg;
	msg.effort = eff;
	msg.velocity = v;
	msg.velocity_variation = rel;
	msg.leg_x = leg_x - gx ;
	msg.leg_y = leg_y - gy ;
	msg.leg_z = leg_z - gz ;
	if(v>0.001)
		stat_pub.publish(msg);
		*/
	//	cout << "Instant Velocity: " << v << "  MEAN: " << ev << "   VAR: " << sqrt(var) << " rel: " << rel << endl;
		
	v_count++;
	lx = gx;
	ly = gy;
	lt = t;
	lv = v;
}




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

void dynamixelCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%d]", msg->header.seq);
    if(mode == ROBOT){
		int index = indexOf(msg->name);
		vals[index] = msg->current_pos;		
		std_msgs::Float64 send_msg; 
		send_msg.data = msg->current_pos;
	    spub[index].publish(send_msg); 
	    //cout << " SEEMS WORKING " << endl;
    }
}

void motorCallback(const control_msgs::JointControllerState::ConstPtr& msg, int i)
{
    //ROS_INFO("I heard: [%d]", msg->header.seq);
    //int index = indexOf(msg->name);
    if(mode == SIMULATION){
    	vals[i] = msg->process_value;
    }
}



void restart()
{
    poses.clear();
    recording = true;
}

void capture()
{
    vector <double> joints;
    for(int i=0;i<20;i++)
        joints.push_back(vals[i]);
    if(curser>=poses.size()){
        poses.push_back(joints);
        curser = poses.size();
    } 
    else
        poses[curser] = joints;
}

void stop()
{
    
    for(int i=0;i<20;i++){
        //cout << topics[i] << ": "  << "stopped" << endl;
        dynamixel_controllers::TorqueEnable serv;
        serv.request.torque_enable = false;
        srv[i].call(serv); 
    }
}

bool same(Pose pose){
    if(pose.size()!=20)
        return false;
    double diff = 0;
    for(int i=0;i<20;i++)
        diff += pow(abs(vals[i] - pose[i]), 2);
    diff = sqrt(diff/20);
    //cerr << " Diff is  " << diff << endl;
    return diff < 0.1;
}


void append(Pose &first, Pose &second, int sign = 1)
{
	for(int i=0;i<second.size();i++)
		first.push_back(sign*second[i]);
}

void play(Pose pose)
{
	for(int i=0;i<20;i++){
        
        //if(i%5==0)
          //  cout << "------------------------ " << endl;
        std_msgs::Float64 msg; 
 
        msg.data = pose[i];
        //cout << topics[i] <<  ": " << msg.data << endl;
        //if(i<10 || i>=15)
        if(mode == ROBOT)
	        pub[i].publish(msg); 
	    else
	        spub[i].publish(msg);     
    }
}

Gait one_to_all(Gait poses)
{
	Gait gait;
	
	for(int i=0;i<poses.size();i++)
		gait.push_back(poses[i]);
	
	append(gait[0], poses[2], -1);
	append(gait[0], poses[2], -1);
	append(gait[0], poses[0]);
	
	append(gait[1], poses[3], -1);
	append(gait[1], poses[3], -1);
	append(gait[1], poses[1]);
	
	append(gait[2], poses[0], -1);
	append(gait[2], poses[0], -1);
	append(gait[2], poses[2]);
	
	append(gait[3], poses[1], -1);
	append(gait[3], poses[1], -1);
	append(gait[3], poses[3]);
	
	for(int i=0;i<gait.size();i++){
		gait[i][5]*=-1;
		//gait[i][7]*=-1;
		gait[i][15]*=-1;
		//gait[i][17]*=-1;
	}
	
	return gait;
}

/* Vector Operators */
vector<double>& operator+(vector <double> a, vector <double> b)
{
	int size = b.size();
	vector<double> *c = new vector<double>();
	if(a.size()<size)
		size = a.size();
	for(int i=0;i<size;i++)
		c->push_back(b[i]+a[i]);
	return *c;
}

vector<double>& operator-(vector <double> a, vector <double> b)
{
	int size = b.size();
	vector<double> *c = new vector<double>();
	if(a.size()<size)
		size = a.size();
	for(int i=0;i<size;i++)
		c->push_back(a[i]-b[i]);
	return *c;
}

vector<double>& operator*(double a, vector <double> b)
{
	vector<double> *c = new vector<double>();
	for(int i=0;i<b.size();i++)
		c->push_back(b[i]*a);
	return *c;
}
/**********************************************************/

Pose& get_pose()
{
	Pose *p = new Pose();
	for(int i=0;i<20;i++)
		p->push_back(vals[i]);
	return *p;
}


int apply_gait(Gait poses, double v = 0)
{
	Gait gait = one_to_all(poses);
	
	for(int i=0;i<gait.size();i++)
	{
	    
	    //cout << " Pos " << i+1 << ": " << endl;
	    //cout << " -------------------- " << endl;
	    play(gait[i]);
	    //cout << " -------------------- " << endl;
	    while(!same(gait[i]))
	    {
	        if(kbhit()){
	            stop();
	            cerr << "_______________ Cancelled _______________" << endl;
	            return 1;
	        }
	        /*
        	if(v>0 && new_vel)
        	{
        		double coef = v/vel;
        		Pose next; 
        		
        		if(coef>2)
        			coef=2;
        		if(coef<0.5)
        			coef = 0.5;
        		
        		new_vel = false;
        		
        		if(coef<0.9)
        			next = 0.9*(gait[i]-get_pose()) + get_pose();
        		else if(coef>1.1)
        			next = 1.1*(gait[i]-get_pose()) + get_pose();
        		else
        			next = gait[i];
        		
        			
        		next = coef*(gait[i]-get_pose()) + get_pose();	
        		//cout << "CHECK" << endl;
        		play(next);
        			
        	}*/
	        //usleep(10000);
	        //sleep(1);
	        //std::this_thread::sleep_for (std::chrono::seconds(1));
	        ros::spinOnce();
	    }
	}
	return 0;	
}

void play_gait(int count)
{
	for(int i=0;i<count;i++) {
		apply_gait(poses);
	}
}


void play(int index=-1)
{
    //cerr << "err: index = " << index << ", size = " << poses.size() << endl;
    if(index>0 && index>=poses.size())
        return;
    
    Gait gait;
    
    if(index>=0)
    	gait = one_to_all(poses);
        
    for(int i=0;i<20;i++){
        
        //if(i%5==0)
          //  cout << "------------------------ " << endl;
        std_msgs::Float64 msg; 
 

        if(index==-1)
            msg.data = double(0);
        else
            msg.data = gait[index][i];
        //cout << topics[i] <<  ": " << msg.data << endl;
        //if(i<10 || i>=15)
        if(mode == ROBOT)
	        pub[i].publish(msg); 
	    else
	        spub[i].publish(msg);     
    }
}



void show()
{
    for(int i=0;i<20;i++)
        cout << topics[i] << ": "  << vals[i] << endl;
}


void change(char c){
    if(c==',' && curser>0)
        curser--;
    if(c=='.' && curser<poses.size())
        curser++;
    cout << "Curser on: " << curser << endl;
}

void remove(int index){
    if(index>=poses.size()){
        cout << "Curser out of bound" << endl;
        return;
    }

    poses.erase (poses.begin()+index);
}

void playAll()
{
    //while(ros::ok()) {
    //
    for(int j=0;j<15;j++) {
    
		for(int i=0;i<poses.size();i++)
		{
		    
		    cout << " Pos " << i+1 << ": " << endl;
		    cout << " -------------------- " << endl;
		    play(i);
		    cout << " -------------------- " << endl;
		    while(!same(poses[i])){
		        if(kbhit()){
		            stop();
		            cerr << "_______________ Cancelled _______________" << endl;
		            return;
		        }
		        //usleep(10000);
		        //sleep(1);
		        //std::this_thread::sleep_for (std::chrono::seconds(1));
		        ros::spinOnce();
		    }
		}
    }

   // play(-1);
}


int toggle(int motor){
    state[motor-1] = !state[motor-1];
    for(int i=(motor-1)*5;i<motor*5;i++){
        //cout << topics[i] << ": "  << "stopped" << endl;
        dynamixel_controllers::TorqueEnable serv;
        serv.request.torque_enable = state[motor-1];
        srv[i].call(serv); 
    }
}

void load(const char* dname = ".") {
	char * fname;	
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (dname)) != NULL) {
		for (int i = 0; i<=fcount; ) {
			if((ent = readdir (dir)) == NULL){
				print("end of files");
				fcount = 0;	
				closedir (dir);		
				return;
			}
			fname = ent->d_name;
			string fn(fname);
			if(fn.substr(fn.find_last_of(".") + 1) == "gait")
				i++;
			//cout <<  ent->d_name << endl;
		}
		closedir (dir);
	} else {
	  /* could not open directory */
	  print("could not open directory ");
	  print(dname);
	  return;
	}

	fcount++;
	
	print(fname);
	print(" loaded!! ");
	
    ifstream fin(fname);
    poses.clear();
    curser = 0;

    while(!fin.eof()) {
        Pose pose;
        for(int i=0;i<20 && !fin.eof();i++){
            double val;
            fin >> val;
            //if(i>=15)
            if(i<5)
         		pose.push_back(val);
         	//cout << val << endl;
        }
        poses.push_back(pose);
    }
    poses.pop_back();
    fin.close();
    
}

/*
	1) sampling
	2) regression
	3) estimating gradient
	
	data:
		th1 .... th20 --> dis
	
	samples:	
		[ (1)  th1 ... th20  --> dis(1)
		  (2)  ...
		       ...
		  (100) th1 ... th20 --> dis(2) ]   
		  
	
*/




void save(const char* fname = "default1.gait"){
    ofstream fout(fname);
    
    cout << "saving on : " << fname << endl;
    
    poses = one_to_all(poses);

    for(int i=0; i<poses.size();i++) {
        vector <double> pose;
        for(int j=0;j<20;j++){
            fout << poses[i][j] << " ";
        }
        fout << endl;
    }

    fout.close();
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

struct Result measure(Gait poses, int count = 10, double v = 0)
{
	double sx = gx, sy = gy, sr = gr;
	double racc = 0;
	int code = 0;
	unsigned long long st, en;
	st = get_msecs();
	
	v_count = 0;
	ev = 0;
	ev2 = 0;
	
	for(int i=0;i<count;i++) {
		code = apply_gait(poses, v);
		if(code)
			break;
		//play(-1);
		ros::spinOnce();
		double dr = abs(gr-sr);
		if(dr>180)
			dr = 360 - dr;
		racc+=dr;
		sr = gr;
		//cout << i << " <> " << "X: " << gx << " --- " << "Y: " << gy << endl;
	}
	
	en = get_msecs();
	
	struct Result result; 
	result.distance = sqrt(abs(sx-gx) * abs(sx-gx) + abs(sy-gy) * abs(sy-gy));
	result.msecs = en - st;
	result.racc = racc;
	result.code = code;
	result.rel = rel;
	result.eff = meff;
	show_result(result);
	return result;
}

Gait sample(Gait poses)
{
	Gait gait;
	for(int i=0;i<poses.size();i++){
		Pose pose;
		for(int j=0;j<poses[i].size();j++){
			double rnd = ((double)(rand() % 1000 - 500)/500.0) * SAMPLING_RANGE;
			//cout << rnd << endl;
			pose.push_back(poses[i][j] + rnd);			
		}
		gait.push_back(pose);
	}
	return gait;
}



ofstream fout("gait.log");

Gait improve(Gait gait, int count=10)
{
	cout << " ---------- " << endl;
	bool updated = false;
	struct Result result = measure(gait);
	struct Result mresult = result;
	double max = objective(result);
	Gait mgait = gait;
	
	
	cout << "Starting at: " << max << endl;
	
	for(int i = 0; i<count; i++) {
		if(result.code)
			return mgait;
		
		cout << "ROUND " << i+1 << ": " << endl;
			
		Gait sgait = sample(gait);
		result = measure(sgait);
		double obj = objective(result);
		if(obj>max){
			max = obj;
			mgait = sgait;
			updated = true;
			mresult = result;
		}
		ros::spinOnce();
		
		// LOG TO FILE
		
		fout << i+1 << " ";
		fout << obj << " ";
		fout << result.distance/((double)result.msecs/1000) << " ";
		fout << result.distance << " ";
		fout << result.msecs << " ";
		fout << result.racc << " ";
		fout << endl;
		
	}
	double obj = objective(mresult);
	fout << 0 << " ";
	fout << obj << " ";
	fout << mresult.distance/((double)mresult.msecs/1000) << " ";
	fout << mresult.distance << " ";
	fout << mresult.msecs << " ";
	fout << mresult.racc << " ";
	fout << endl;
	
	//if(!updated)
		//SAMPLING_RANGE*=2;
	
	cout << "MAX: " << max << endl;
	return mgait;
}


void render_menu()
{
	flush();
	for(int i=0;i<100;i++)
		cout << endl;
    print("[N] new ");
    print("[L] load ");
    print("[W] save ");
    print("[O] optimize ");
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber *sub;
    ros::Subscriber *dyn_sub;    
    ros::Subscriber model_sub;
    ros::Subscriber link_sub;
    
    ros::Subscriber joints_sub;
    
    ros::Timer vel_timer = n.createTimer(ros::Duration(0.1), timerCallback);

	dyn_sub = new ros::Subscriber[20];
    sub = new ros::Subscriber[20];
    pub = new ros::Publisher[20];
    spub = new ros::Publisher[20];
    srv = new ros::ServiceClient[20];
    
	model_sub = n.subscribe("/gazebo/model_states", 1, stateCallback);
	link_sub = n.subscribe("/gazebo/link_states", 1, linkCallback);
	joints_sub = n.subscribe("/daedalus/joint_states", 1, jointCallback);
	
	stat_pub = n.advertise<daedalus_control::Stats>("/daedalus/stats", 10);

    for(int i=0;i<20;i++){
        dyn_sub[i] = n.subscribe("/" + topics[i] + "/state", 10, dynamixelCallback);        
        sub[i] = n.subscribe<control_msgs::JointControllerState>("/daedalus/" + sim_topics[i] + "_position_controller/state", 10, boost::bind(motorCallback, _1,i));
        pub[i] = n.advertise<std_msgs::Float64>("/" + topics[i] + "/command", 10);
        spub[i] = n.advertise<std_msgs::Float64>("/daedalus/" + sim_topics[i] + "_position_controller/command", 10);
        srv[i] = n.serviceClient<dynamixel_controllers::TorqueEnable>("/" + topics[i] + "/torque_enable");
    }
    //ros::spin();
    
    int count = 0;
    
    render_menu();

    while(ros::ok())
    {
        //cout << "looping" << endl;
        
        
        if (kbhit())
        {
            //cout << "kbhit" << endl;
            render_menu();
	        
            char key=getchar();
            if(key == ' ')
                capture();
            if(key == 'r' || key == 'R')
                restart();
            
            if(key == 'o' || key == 'O'){
            	SAMPLING_RANGE = 0.2;
            	for(int i=0;i<8;i++){
            		cout << "-------- Iteration " << i+1 << " --------- " << endl;
            		cout << "SAMPLE RANGE: " << SAMPLING_RANGE << endl;
            		poses=improve(poses);
            		SAMPLING_RANGE/=sqrt(2);
            	}
                //cout << measure(poses) <<  endl;   
            }
            
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
                toggle(key-'0');         

            if(key == 'p' || key == 'P'){
            	v_count = 0;
            	Result result = measure(poses);
            	cout << result.distance << "m in " << (double)result.msecs/1000 << "secs" << endl;
            	cout << "Velocity: " << result.distance/((double)result.msecs/1000) << endl;
            	//play_gait(10);
                //playAll();
			}
			
			if(key == 'i' || key == 'I'){
            	v_count = 0;
	          	measure(poses, 5, 0);            	
	          	cout << "speed: " << ev << endl;
            	Result result = measure(poses, 10, ev);
            	cout << result.distance << "m in " << (double)result.msecs/1000 << "secs" << endl;
            	cout << "Velocity: " << result.distance/((double)result.msecs/1000) << endl;
            	//play_gait(10);
                //playAll();
			}
			
            if(key == 'g' || key == 'G')
                play(curser);

            if(key == 's' || key == 'S')
                showing = !showing;          

            if(key == 'q' || key == 'Q')
                break;
            
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
        }  

        if(showing && ++count%1000==0)
            show();
        ros::spinOnce();
    }

    return 0;
}


