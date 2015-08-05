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
#include "dynamixel_msgs/JointState.h"
#include "control_msgs/JointControllerState.h"
#include "dynamixel_controllers/TorqueEnable.h"
#include "gazebo_msgs/ModelStates.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>

using namespace std;

typedef vector<float> Pose;
typedef vector <Pose> Gait;

float SAMPLING_RANGE = 0.4;

ros::Publisher *pub;
ros::Publisher *spub;
ros::ServiceClient *srv;


int curser = 0;
int fcount = 0;

float vals[20];

float gx, gy, gr;

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

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

int indexOf(string topic)
{
    for(int i=0;i<20;i++)
        if(topic == topics[i])
            return i;
    return 0;
}

void stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	gx = msg->pose[1].position.x;
	gy = msg->pose[1].position.y;
	gr = msg->pose[1].orientation.z;
}

void motorCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%d]", msg->header.seq);
    int index = indexOf(msg->name);
    vals[index] = msg->current_pos;
}

void restart()
{
    poses.clear();
    recording = true;
}

void capture()
{
    vector <float> joints;
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
    float diff = 0;
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
        pub[i].publish(msg); 
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

void apply_gait(Gait poses)
{
	Gait gait = one_to_all(poses);
	
	for(int i=0;i<gait.size();i++)
	{
	    
	    //cout << " Pos " << i+1 << ": " << endl;
	    //cout << " -------------------- " << endl;
	    play(gait[i]);
	    //cout << " -------------------- " << endl;
	    while(!same(gait[i])){
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
    
        
    for(int i=0;i<20;i++){
        
        //if(i%5==0)
          //  cout << "------------------------ " << endl;
        std_msgs::Float64 msg; 
 

        if(index==-1)
            msg.data = float(0);
        else
            msg.data = poses[index][i];
        //cout << topics[i] <<  ": " << msg.data << endl;
        //if(i<10 || i>=15)
        pub[i].publish(msg); 
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
				cerr << "end of files" << endl;		
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
	  cerr << "could not open directory" << dname << endl;
	  return;
	}

	fcount++;
	
	cout << fname << " loaded!! " << endl;
	
    ifstream fin(fname);
    poses.clear();
    curser = 0;

    while(!fin.eof()) {
        Pose pose;
        for(int i=0;i<20 && !fin.eof();i++){
            float val;
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
        vector <float> pose;
        for(int j=0;j<20;j++){
            fout << poses[i][j] << " ";
        }
        fout << endl;
    }

    fout.close();
}

struct Result {
	float distance;
	float racc;
};

struct Result measure(Gait poses, int count = 10)
{
	float sx = gx, sy = gy, sr = gr;
	float racc = 0;
	for(int i=0;i<count;i++) {
		apply_gait(poses);
		play(-1);
		ros::spinOnce();
		float dr = abs(gr-sr);
		if(dr>180)
			dr = 360 - dr;
		racc+=dr;
		//cout << i << " <> " << "X: " << gx << " --- " << "Y: " << gy << endl;
	}
	cout << "Err(r) = " << racc << endl;
	struct Result result; 
	result.distance = sqrt(abs(sx-gx) * abs(sx-gx) + abs(sy-gy) * abs(sy-gy));
	result.racc = racc;

	return result;
}

Gait sample(Gait poses)
{
	Gait gait;
	for(int i=0;i<poses.size();i++){
		Pose pose;
		for(int j=0;j<poses[i].size();j++){
			float rnd = ((float)(rand() % 1000 - 500)/500.0) * SAMPLING_RANGE;
			//cout << rnd << endl;
			pose.push_back(poses[i][j] + rnd);			
		}
		gait.push_back(pose);
	}
	return gait;
}

float objective(Result result)
{
	return result.distance - result.racc * 0.5;
}

Gait improve(Gait gait, int count=10)
{
	cout << " ---------- " << endl;
	bool updated = false;
	struct Result result = measure(gait);
	float max = objective(result);
	Gait mgait = gait;
	
	
	cout << "Starting at: " << max << endl;
	
	for(int i = 0; i<count; i++) {
		Gait sgait = sample(gait);
		result = measure(sgait);
		float obj = objective(result);
		if(obj>max){
			max = obj;
			mgait = sgait;
			updated = true;
		}
		ros::spinOnce();
		cout << "ROUND " << i+1 << ": " << endl;
		cout << "Objective value = " << obj << endl;
		cout << "Distance = " << result.distance << endl;
		cout << "Rotation Error = " << result.racc << endl;
		cout << endl;
	}
	
	if(!updated)
		SAMPLING_RANGE*=2;
	
	cout << "MAX: " << max << endl;
	return mgait;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber *sub;    
    ros::Subscriber model_sub;

    sub = new ros::Subscriber[20];
    pub = new ros::Publisher[20];
    spub = new ros::Publisher[20];
    srv = new ros::ServiceClient[20];
    
    

	model_sub = n.subscribe("/gazebo/model_states", 1, stateCallback);

    for(int i=0;i<20;i++){
        sub[i] = n.subscribe("/" + topics[i] + "/state", 10, motorCallback);        
        //sub[i] = n.subscribe<control_msgs::JointControllerState>("/daedalus/" + sim_topics[i] + "_position_controller/state", 10, boost::bind(motorCallback, _1,i));
        pub[i] = n.advertise<std_msgs::Float64>("/" + topics[i] + "/command", 10);
        spub[i] = n.advertise<std_msgs::Float64>("/daedalus/" + sim_topics[i] + "_position_controller/command", 10);
        srv[i] = n.serviceClient<dynamixel_controllers::TorqueEnable>("/" + topics[i] + "/torque_enable");
    }
    //ros::spin();
    
    int count = 0;

    while(ros::ok())
    {
        //cout << "looping" << endl;
        if (kbhit())
        {
            //cout << "kbhit" << endl;
            char key=getchar();
            if(key == ' ')
                capture();
            if(key == 'r' || key == 'R')
                restart();
            
            if(key == 'm' || key == 'M'){
            	SAMPLING_RANGE = 0.2;
            	for(int i=0;i<10;i++){
            		cout << "SAMPLE RANGE: " << SAMPLING_RANGE << endl;
            		poses=improve(poses);
            		SAMPLING_RANGE/=2;
            	}
                //cout << measure(poses) <<  endl;   
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
            	Result result = measure(poses);
            	cout << result.distance << endl;
            	//play_gait(10);
                //playAll();
			}
            if(key == 'o' || key == 'O')
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


