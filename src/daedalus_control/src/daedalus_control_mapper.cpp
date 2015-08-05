#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include <cmath>

#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "dynamixel_msgs/JointState.h"
#include "dynamixel_controllers/TorqueEnable.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

ros::Publisher *pub;
ros::ServiceClient *srv;


int curser = 0;

float vals[20];

vector <vector <float> > poses;

bool recording = false;
bool showing = false;

bool state[4] = {false, false, false, false};

const char* context = "default.gait";

const string topics[] = {

	"bottom_left_1",
    "bottom_left_2",
    "bottom_left_3",
    "bottom_left_4",
    "bottom_left_5",
  
    "top_left_1",
    "top_left_2",
    "top_left_3",
    "top_left_4",    
    "top_left_5",    

    "bottom_right_1",
    "bottom_right_2",
    "bottom_right_3",
    "bottom_right_4",
    "bottom_right_5",
    
    "top_right_1",
    "top_right_2",
    "top_right_3",
    "top_right_4",
    "top_right_5",

};

const string sim_topics[] = {
  
    "leg1joint1",
    "leg1joint2",
    "leg1joint3",
    "leg1joint4",    
    "leg1joint5",    

    "leg2joint1",
    "leg2joint2",
    "leg2joint3",
    "leg2joint4",
    "leg2joint5",

    "leg3joint1",
    "leg3joint2",
    "leg3joint3",
    "leg3joint4",
    "leg3joint5",

    "leg4joint1",
    "leg4joint2",
    "leg4joint3",
    "leg4joint4",
    "leg4joint5"

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

void motorCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%d]", msg->header.seq);
    int index = indexOf(msg->name);
    vals[index] = msg->current_pos;
    cout << "Receive from " << index << ": " << msg->current_pos << endl;
	std_msgs::Float64 send_msg; 
	send_msg.data = msg->current_pos;
    pub[index].publish(send_msg); 
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

void play(int index=-1)
{
    cerr << "err: index = " << index << ", size = " << poses.size() << endl;
    if(index>0 && index>=poses.size())
        return;
    for(int i=0;i<20;i++){
        
        if(i%5==0)
            cout << "------------------------ " << endl;

        std_msgs::Float64 msg; 
 
        if(index==-1)
            msg.data = float(0);
        else
            msg.data = poses[index][i];
        cout << topics[i] <<  ": " << msg.data << endl;
        //if(i<10 || i>=15)
            pub[i].publish(msg); 
    }
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

void show()
{
    for(int i=0;i<20;i++)
        cout << topics[i] << ": "  << vals[i] << endl;
}

bool same(vector <float> pose){
    if(pose.size()!=20)
        return false;
    float diff = 0;
    for(int i=0;i<20;i++)
        diff += pow(abs(vals[i] - pose[i]), 2);
    diff = sqrt(diff/20);
    //cerr << " Diff is  " << diff << endl;
    return diff < 0.1;
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


int toggle(int motor){
    state[motor-1] = !state[motor-1];
    for(int i=(motor-1)*5;i<motor*5;i++){
        //cout << topics[i] << ": "  << "stopped" << endl;
        dynamixel_controllers::TorqueEnable serv;
        serv.request.torque_enable = state[motor-1];
        srv[i].call(serv); 
    }
}

void load(const char* fname = "default.gait") {
    ifstream fin(fname);
    poses.clear();

    while(!fin.eof()) {
        vector <float> pose;
        for(int i=0;i<20 && !fin.eof();i++){
            float val;
            fin >> val;
            pose.push_back(val);
        }
        poses.push_back(pose);
    }
    fin.close();
}

void save(const char* fname = "default1.gait"){
    ofstream fout(fname);
    
    cout << "saving on : " << fname << endl;

    for(int i=0; i<poses.size();i++) {
        vector <float> pose;
        for(int j=0;j<20;j++){
            fout << poses[i][j] << " ";
        }
        fout << endl;
    }

    fout.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_mapper");
    ros::NodeHandle n;
    ros::Subscriber *sub;    

    sub = new ros::Subscriber[20];
    pub = new ros::Publisher[20];
    srv = new ros::ServiceClient[20];

    for(int i=0;i<20;i++){
        sub[i] = n.subscribe("/" + topics[i] + "/state", 10, motorCallback);
        //pub[i] = n.advertise<std_msgs::Float64>("/" + topics[i] + "/command", 10);
        pub[i] = n.advertise<std_msgs::Float64>("/daedalus/" + sim_topics[i] + "_position_controller/command", 10);
        srv[i] = n.serviceClient<dynamixel_controllers::TorqueEnable>("/" + topics[i] + "/torque_enable");
    }
    ros::spin();
    
    int count = 0;

    while(ros::ok())
    {
        //cout << "looping" << endl;
        /*
        if (kbhit())
        {
            //cout << "kbhit" << endl;
            char key=getchar();
            if(key == ' ')
                capture();
            if(key == 'r' || key == 'R')
                restart();
            
            if(key=='-')
                stop();

            if(key=='0')
                play(-1);

            if(key<='4' && key>='1')
                toggle(key-'0');         

            if(key == 'p' || key == 'P')
                playAll();

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
        */    
        ros::spinOnce();
    }

    return 0;
}


