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


#define PI 3.14159
#define N_JOINTS 5

class Gait {
	
	public:		
	const double lb_p[5] = {-PI, -PI, -PI, -PI, -PI};
	const double ub_p[5] = {+PI, +PI, +PI, +PI, +PI};
	
	const double lb_t = 0;
	const double ub_t = 1;
	
	const double lb_ph = 0;
	const double ub_ph = 1;
	
	double step_dur;
	vector<double> p1, p2;
	vector<double> t1, t2;

	double leg_phases[4] = {0, 0.5, 0.25, 0.75};
	
	double random(double range){
		return ((double)(rand() % 10000 - 5000)/5000.0) * range;
	}
	
	void swap(double &a, double &b){
		double c = a;
		a = b;
		b = c;
	}
	
	void validate()
	{
		for(int i=0;i<N_JOINTS;i++) {
			if(p1[i]<lb_p[i])
				p1[i]=lb_p[i];
			if(p2[i]<lb_p[i])
				p2[i]=lb_p[i];
				
			if(p1[i]>ub_p[i])
				p1[i]=ub_p[i];	
			if(p2[i]>ub_p[i])
				p2[i]=ub_p[i];	
	
			if(t1[i]<lb_t)
				t1[i]=lb_t;
			if(t2[i]<lb_t)
				t2[i]=lb_t;
				
			if(t1[i]>ub_t)
				t1[i]=ub_t;
			if(t2[i]>ub_t)
				t2[i]=ub_t;
				
			if(t1[i]>t2[i])
				swap(t1[i],t2[i]);
		}
		
		for(int i=0;i<4;i++){
			if(leg_phases[i]<lb_ph)
				leg_phases[i]=lb_ph;
			
			if(leg_phases[i]>ub_ph)
				leg_phases[i]=ub_ph;
		}
	}
	
	void order(int i)
	{
		// legs 2 4 1 3
		double p[3][4] = {{0.5, 0.25, 0, 0.75},
						{0.5, 0, 0, 0.5},
						{0.75, 0.5, 0, 0.25}};
		for(int j = 0;j<4;j++)
			leg_phases[j] = p[i][j];
	}
	
	void sample(double range) 
	{
		
		for(int i=0;i<N_JOINTS;i++) {
			p1[i] = p1[i]+random(range*PI);
			p2[i] = p2[i]+random(range*PI);
			t1[i] = t1[i]+random(range);
			t2[i] = t2[i]+random(range);
		}
		
		validate();
		//for(int i=0;i<4;i++)
			//leg_phases[i] = leg_phases[i];
		
		//return gait;
	}
	
	Gait resample(double range)
	{
		Gait gait(this->get_chroms());	
		gait.sample(range);
		return gait;
	}
		
	vector<double> get_chroms() const
	{
		vector<double> chroms;
		chroms.push_back(step_dur);
		for(int i=0;i<N_JOINTS;i++){
			chroms.push_back(p1[i]);
			chroms.push_back(t1[i]);
			chroms.push_back(p2[i]);
			chroms.push_back(t2[i]);
		}
		for(int i=0;i<4;i++)
			chroms.push_back(leg_phases[i]);
		return chroms;
	}
	
	void put_chroms (const vector<double>& chroms)
	{
		step_dur = chroms[0];
		for(int i=0;i<N_JOINTS;i++) {
			p1[i] = chroms[i*4+1];
			t1[i] = chroms[i*4+2];
			p2[i] = chroms[i*4+3];
			t2[i] = chroms[i*4+4];
		}
		for(int i=0;i<4;i++)
			leg_phases[i] = chroms[i+21];
	}
	
	Gait (vector<double> chroms) : Gait()
	{
		step_dur = chroms[0];
		for(int i=0;i<N_JOINTS;i++) {
			p1[i] = chroms[i*4+1];
			t1[i] = chroms[i*4+2];
			p2[i] = chroms[i*4+3];
			t2[i] = chroms[i*4+4];
		}
		for(int i=0;i<4;i++)
			leg_phases[i] = chroms[i+21];
	}		
	
	
	Gait(const Gait& gait) : Gait()
	{
		put_chroms(gait.get_chroms());
		
	}
	
	Gait& operator=(const Gait& gait)
	{
		put_chroms(gait.get_chroms());
	}
	
	
	Gait()
	{
		step_dur = 4;
		for(int i=0;i<5;i++){
			p1.push_back(0);
			p2.push_back(0);
			t1.push_back(0.25);
			t2.push_back(0.75);
		}
		p1[1] = p2[1] = -1;
	}
};


typedef vector<double> Pose;
//typedef vector <Pose> Gait;

double SAMPLING_RANGE = 0.4;

ros::Publisher *pub;
ros::Publisher *spub;
ros::Publisher stat_pub;
ros::ServiceClient *srv;


int curser = 0;
int fcount = 0;

double vals[20];

double gx, gy, gz, gr;

//Gait poses;

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
	struct timeval tv;
	gettimeofday(&tv, NULL);

	ros::Time now = ros::Time::now();

	double millisecondsSinceEpoch = //tv.tv_sec * 1000 + tv.tv_usec /1000;
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
	
	daedalus_control::Stats msg;
	msg.effort = eff;
	msg.velocity = v;
	msg.velocity_variation = rel;
	msg.leg_x = leg_x - gx ;
	msg.leg_y = leg_y - gy ;
	msg.leg_z = leg_z - gz ;
	if(v>0.001)
		stat_pub.publish(msg);
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
	
	//cout << gx << endl;
	
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



void put(double th, int joint)
{
	std_msgs::Float64 msg;
	msg.data = th;
	if(mode == ROBOT)
        pub[joint].publish(msg); 
    else
        spub[joint].publish(msg); 
    //cout << joint << " --- " << th << endl;
}

double mod(double a, double b)
{
	int c = a/b;
	return a-b*c;
}

void run_gait(Gait gait, int count)
{
	double sd = gait.step_dur*1000;
	double t0 = get_msecs();
	double time = 0;
	
	int counter = 0;
	
	
	ros::Rate r(100);
	
	while(time/sd<count && ros::ok()) {
		//cout << time/sd << endl;
		time = get_msecs()-t0;
		for(int i=0;i<20;i++){	
			int leg = i/5;
			int joint = i%5;
			double phase = gait.leg_phases[leg];
			double t = mod((time + phase*sd), sd)/sd;
			double t1 = gait.t1[joint];
			double t2 = gait.t2[joint];
			
			//cout << t << endl;
			ros::Time now = ros::Time::now();
			//if(counter++%1000==0)
			//ROS_INFO("%f ---- %f", now.toSec(), t);
			
			double value; 
			
			if(t<t1 || t>=t2)
				value = gait.p2[joint];
			else
				value = gait.p1[joint];
			
			if(leg>0 && leg<3)
				value*=-1;
			
			if(i==5 || i==15 || i==9 || i==19 )
				value*=-1;	
			
			put(value, i);
		}
		r.sleep();
		ros::spinOnce();
	}
	
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
	return vel * 10 + result.distance * 0.25 - result.racc * 0.1 - result.rel * 1.5;// - result.eff * 0.01;
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
	
	run_gait(poses, count);
	
	/*
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
	}*/
	
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


ofstream fout("gait.log");

Gait improve(Gait gait, int count=10, double sample_range = 0.25)
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
			
		Gait sgait = gait.resample(sample_range);
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


Gait load(const char* dname = ".") {
	Gait gait;
	char * fname;	
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (dname)) != NULL) {
		for (int i = 0; i<=fcount; ) {
			if((ent = readdir (dir)) == NULL){
				print("end of files");
				fcount = 0;	
				closedir (dir);		
				return gait;
			}
			fname = ent->d_name;
			string fn(fname);
			if(fn.substr(fn.find_last_of(".") + 1) == "chroms")
				i++;
			//cout <<  ent->d_name << endl;
		}
		closedir (dir);
	} else {
	  /* could not open directory */
	  print("could not open directory ");
	  print(dname);
	  return gait;
	}

	fcount++;
	
	print(fname);
	print(" loaded!! ");
	
    ifstream fin(fname);
    curser = 0;
	Pose chroms;

    while(!fin.eof()) {
        double val;
        fin >> val;
 		chroms.push_back(val);
    }
    
    fin.close();
    
	gait = Gait(chroms);
	return gait;
}

void save(const char* fname, vector<double> gait){
    ofstream fout(fname);
    
    cout << "saving on : " << fname << endl;

    for(int i=0; i<gait.size();i++) {
        
		fout << gait[i] << endl;
    }

    fout.close();
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
	
	stat_pub = n.advertise<daedalus_control::Stats>("/daedalus/stats", 1);

    for(int i=0;i<20;i++){
        dyn_sub[i] = n.subscribe("/" + topics[i] + "/state", 10, dynamixelCallback);        
        sub[i] = n.subscribe<control_msgs::JointControllerState>("/daedalus/" + sim_topics[i] + "_position_controller/state", 10, boost::bind(motorCallback, _1,i));
        pub[i] = n.advertise<std_msgs::Float64>("/" + topics[i] + "/command", 10);
        spub[i] = n.advertise<std_msgs::Float64>("/daedalus/" + sim_topics[i] + "_position_controller/command", 10);
        srv[i] = n.serviceClient<dynamixel_controllers::TorqueEnable>("/" + topics[i] + "/torque_enable");
    }
       
    Gait gait;
    
    gait.step_dur = 2.5;
    //gait.sample(0.1);
    double a[5] = {-0.540022, -1.38859, -0.833004, 0.173653, 0.0551636};
    double b[5] = {0.363155, -0.982566, -0.429265, -0.0268031, -0.223628};
    
    gait.p1 = vector<double> (a,a+sizeof(a)/sizeof(a[0]));
    gait.p2 = vector<double> (b,b+sizeof(b)/sizeof(b[0]));
    /*
    vector<double> c = gait.get_chroms();
    for(int i=0;i<c.size();i++)
	    cout << c[i] << " ";
    cout << endl;
    */
 
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
                        
            if(key == 'o' || key == 'O'){
            	SAMPLING_RANGE = 0.05;
            	for(int i=0;i<8;i++){
            		cout << "-------- Iteration " << i+1 << " --------- " << endl;
            		cout << "SAMPLE RANGE: " << SAMPLING_RANGE << endl;
            		gait=improve(gait, 10, SAMPLING_RANGE);
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
            

            //if(key=='0')
              //  play(-1);

            if(key<='4' && key>='1')
                gait.order(key-'0'-1);         

            if(key == 'p' || key == 'P'){
            	v_count = 0;
            	Result result = measure(gait);
            	cout << result.distance << "m in " << (double)result.msecs/1000 << "secs" << endl;
            	cout << "Velocity: " << result.distance/((double)result.msecs/1000) << endl;
            	//play_gait(10);
                //playAll();
			}
				
         
            if(key == 'q' || key == 'Q')
                break;
            
            if(key == 'l' || key == 'L'){
                gait = load();
             	//gait.step_dur = 2;                  
			}
            if(key == 'w' || key == 'W')
                save("default.chroms", gait.get_chroms());

        }  

      
        ros::spinOnce();
    }

    return 0;
}


