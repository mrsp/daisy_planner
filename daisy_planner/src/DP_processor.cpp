



#include <string.h>
#include <ctime>
#include <iostream>
#include "DP_processor.h"
#include "fuzzy_calc.h"
#include "FuzzyTimeTBX.h"
#include "CoutColors.h"
//#include "gnuplot-iostream/gnuplot-iostream.h"

#include <functional>
#include <iostream>
#include <string>

#define CONSTRAINT_HANDLING 1   //0: Select petal based on the less number of constraints
                                //1: Select petal based on the faster resolved constraints

//#define LocalTesting //Do not define it here. Visit CoutColors.h  
                                
#define MAXTIME 100000

#define UrgencyUPDATE 0.9 //1.1: the petal will be executed soon, 0.9: the petal will be executed with low priority

using namespace std;


extern double humanArousal;
extern double arousalToTimeEffect;

extern double human2target;
extern double robot2target;

extern commandlineParameters cp;

int moveon=0;
int localT=0;
float max_urgency=1.0; // this is not static but increases linearly by 1, as time goes by.....



/*This implements the processing of the DaisyPlanner to produce the currently implented graph.*/
float MixingCycleDuration=2.5;

  DP_processor *myself;

const char* MenuItems[] = {"no", "APPLE", "BANANA", "GRAPE", "WATERMELON", "ORANGE", "STRAWBERRIES", "CHERRIES"};



/*--------------------------------------------------------------------*/

void NAOchatterCallback(const timestorm_msg::Robot::ConstPtr& msg)
{
  ROS_INFO("------------------ NAOchatter heard: aid:%d compl:%d\n\n\n", msg->a_id, msg->a_completed);
  if (msg->a_completed==1){
	myself->setCurrentActionComplete(NAO_ID);
  }
  
        myself->sendDaisyState();

}


/*--------------------------------------------------------------------*/

void JACOchatterCallback(const timestorm_msg::Robot::ConstPtr& msg)
{
  ROS_INFO("################# JACOchatter heard: aid:%d compl:%d", msg->a_id, msg->a_completed);
  if (msg->a_completed==1){
	myself->setCurrentActionComplete(JACO_ID);
  }
  
  if (msg->a_param>0){ //number of executed mixing-cycles
     //na antistoixithoyn ta cycles se xrono
  }
  
        myself->sendDaisyState();

}

/*--------------------------------------------------------------------*/

void HUMANchatterCallback(const timestorm_msg::Robot::ConstPtr& msg)
{
  ROS_INFO("()()()()()()()() HUMANchatter heard: aid:%d compl:%d", msg->a_id, msg->a_completed);
  if (msg->a_completed==1){
	myself->setCurrentActionComplete(HUMAN_ID);
  }
  
        myself->sendDaisyState();

}

/*--------------------------------------------------------------------*/

void HUMAN_Perc_chatterCallback(const timestorm_msg::Time::ConstPtr& msg)
{
  ROS_INFO("()()()()()()()()() HUMAN_Perc_chatter heard dur:%d, eff:%f", (int)msg->duration, (float)msg->efficiency );
  
  if ((msg->duration>=0) &&(msg->duration<1000)){
    myself->setRemainingTimeForAction(HUMAN_ID, myself->agentMainWorkingPetal[HUMAN_ID], myself->agentMainWorkingAction[HUMAN_ID], msg->duration);
  }
  
  if ((msg->efficiency>=0.0) &&(msg->efficiency<1000.0)){
    arousalToTimeEffect=sqrt(1.0+(msg->efficiency/100.0)-0.75);
  }
  
  if (((int)msg->efficiency==-1) &&((int)msg->efficiency==-1)){
    myself->petal[myself->agentMainWorkingPetal[HUMAN_ID]].action[myself->agentMainWorkingAction[HUMAN_ID]].exp_deftime[HUMAN_ID]+=200;
    int p=myself->agentMainWorkingPetal[(int)NAO_ID];
    	      if ( (p==0) &&(myself->agentMainWorkingAction[NAO_ID]==2)){
		
		
		cout <<"inteRVining..."<<endl;
		//myself->intervene(p, NAO_ID);
	      }

    cout << "\n\nHuman postponed action \n\n";
  }

        myself->sendDaisyState();

}
/*--------------------------------------------------------------------*/

//this function updates the Menu, based on the menus provided by the memeory-based inference engine
void MENUchatterCallback(const timestorm_msg::Fruit_id::ConstPtr& msg)
{
//  makeMenuPlan(msg.Fruit_id[0], msg.Fruit_id[1], msg.Fruit_id[2], msg.Fruit_id[3]);
  
  
  ROS_INFO("MENUchatter heard: %d %d %d %d ",msg->fruit_id[0], msg->fruit_id[1], msg->fruit_id[2], msg->fruit_id[3]);
  
//  petal[2].action[2].name<<"GRASP_%"<< ingredientNames[ingr_id];
//  petal[2].action[2].name<<"GRASP_%"<< ingredientNames[ingr_id];
  
  
  //replace the occurences of the "Fruit1" string with the name of the first selected fruit
  string s2remove="Fruit1";
  int c=0;
  string s2put=MenuItems[msg->fruit_id[c]];
  int pet=2;
  int act=0;
//  cout<<myself->petal[pet].action[act].name;
  if (myself->petal[pet].action[act].name.find(s2remove)!=std::string::npos){
//    std::size_t p0 =myself->petal[pet].action[act].name.find(s2remove); //a new petal start is found

    myself->petal[pet].action[act].name.replace(myself->petal[pet].action[act].name.find(s2remove),s2remove.length(),s2put);
//    cout<<myself->petal[pet].action[act].name;
  
    act=1;
    myself->petal[pet].action[act].name.replace(myself->petal[pet].action[act].name.find(s2remove),s2remove.length(),s2put);
    act=2;
    myself->petal[pet].action[act].name.replace(myself->petal[pet].action[act].name.find(s2remove),s2remove.length(),s2put);
  }
  //replace the occurences of the "Fruit2" string  with the name of the second selected fruit
  s2remove="Fruit2";
  c=1;
  s2put=MenuItems[msg->fruit_id[c]];
  pet=3;
  act=0;

  if (myself->petal[pet].action[act].name.find(s2remove)!=std::string::npos){
    myself->petal[pet].action[act].name.replace(myself->petal[pet].action[act].name.find(s2remove),s2remove.length(),s2put);
    act=1;
    myself->petal[pet].action[act].name.replace(myself->petal[pet].action[act].name.find(s2remove),s2remove.length(),s2put);
    act=2;
    myself->petal[pet].action[act].name.replace(myself->petal[pet].action[act].name.find(s2remove),s2remove.length(),s2put);
  }
  //replace the occurences of the "Fruit3" string  with the name of the third selected fruit
  s2remove="Fruit3";
  c=2;
  s2put=MenuItems[msg->fruit_id[c]];
  pet=4;
  act=0;
  if (myself->petal[pet].action[act].name.find(s2remove)!=std::string::npos){
    myself->petal[pet].action[act].name.replace(myself->petal[pet].action[act].name.find(s2remove),s2remove.length(),s2put);
    act=1;
    myself->petal[pet].action[act].name.replace(myself->petal[pet].action[act].name.find(s2remove),s2remove.length(),s2put);
    act=2;
    myself->petal[pet].action[act].name.replace(myself->petal[pet].action[act].name.find(s2remove),s2remove.length(),s2put);
  }
  
        myself->sendDaisyState();

}


/*--------------------------------------------------------------------*/
//this will support the visualization of the DP
/*void DPchatterCallback(const nao_com::Time::ConstPtr& msg)
{
  ROS_INFO("DPchatter heard: ");
}
*/
/*--------------------------------------------------------------------*/
/*
void DP_processor::DPchatterCallback(const nao_com::Time::ConstPtr& msg)
{
  ROS_INFO("I heard: ");
}
*/
/*--------------------------------------------------------------------*/
DP_processor::DP_processor(){

   init();
   myself=this;
       
   ros::init(cp.nargc, cp.nargv, "daisy_planner");
   
   n=new ros::NodeHandle();

    cout<<"sending menu"<<endl;

   
   NAO_pub = new ros::Publisher(n->advertise<timestorm_msg::Daisy>("/daisy/to_nao_chatter", 10) );
   NAO_sub = new ros::Subscriber(n->subscribe("/daisy/from_nao_chatter", 10, NAOchatterCallback));

   JACO_pub = new ros::Publisher(n->advertise<timestorm_msg::Daisy>("/daisy/to_jaco_chatter", 10) );
   JACO_sub = new ros::Subscriber(n->subscribe("/daisy/from_jaco_chatter", 10, JACOchatterCallback));

   HUMAN_pub = new ros::Publisher(n->advertise<timestorm_msg::Daisy>("/daisy/to_human_chatter", 10) );
//   HUMAN_sub = new ros::Subscriber(n->subscribe("/daisy/from_human_chatter", 10, JACOchatterCallback));
   
   //This is used to hear the Human-action related estimates
   HUMAN_Perc_pub = new ros::Publisher(n->advertise<timestorm_msg::Time>("/nao_com/time", 10) );
   HUMAN_Perc_sub = new ros::Subscriber(n->subscribe("/nao_com/time", 10, HUMAN_Perc_chatterCallback));
   
   MENU_pub = new ros::Publisher(n->advertise<timestorm_msg::Fruit_id>("/daisy/menu_chatter", 10) );
   MENU_sub = new ros::Subscriber(n->subscribe("/daisy/menu_chatter", 10, MENUchatterCallback));

   GRAPH_pub = new ros::Publisher(n->advertise<timestorm_msg::Daisy_graph>("/daisy/graph_chatter", 10) );
//   GRAPH_sub = new ros::Subscriber(n->subscribe("/daisy/graph_chatter", 10, GRAPHchatterCallback));

//   GRAPH_sub = n.subscribe("/daisy/graph_chatter", 10, GRAPHchatterCallback);

   
//   MemQuery_pub = new ros::Publisher(n->advertise<timestorm_msg::Fruit_id>("MemQuery_chatter", 10) );
//   MemQuery_sub = new ros::Subscriber(n->subscribe("MemQuery_chatter", 10, MemQuerychatterCallback));
   
   

/*   
   DPvis_pub = n.advertise<timestorm_msg::Time>("DP_chatter", 10);
   DPvis_sub = n.subscribe("DP_chatter", 10, DPchatterCallback);
*/


 cout<<"sending menu"<<endl;

  sendMenu();

}





/*--------------------------------------------------------------------*/

void DP_processor::sendAction2ROS(int ag_id){
  
      timestorm_msg::Daisy msg;
      msg.p_name="PetalName";
      msg.p_id=agentMainWorkingPetal[ag_id];
      
      msg.a_name=petal[agentMainWorkingPetal[ag_id]].action[agentMainWorkingAction[ag_id]].name;
      msg.a_id=agentMainWorkingAction[ag_id];
      
      
      msg.a_start_time=petal[agentMainWorkingPetal[ag_id]].action[agentMainWorkingAction[ag_id]].start_time;
//      msg.a_completed=-1;
      
      if (petal[agentMainWorkingPetal[ag_id]].action[agentMainWorkingAction[ag_id]].done>=0){
	msg.a_end_time=petal[agentMainWorkingPetal[ag_id]].action[agentMainWorkingAction[ag_id]].implementation_time;
      }
      else{
	msg.a_end_time=-1;
      }
     
      if (ag_id==JACO_ID){	
	msg.a_param=-1;
	string s(petal[agentMainWorkingPetal[ag_id]].action[agentMainWorkingAction[ag_id]].name); //make a string with the name of the action

	if (agentMainWorkingPetal[ag_id]==5){
	//	if it is the mix-salad action, then estimate the number of repetitive cycles for mixing
	  if (s.find("Mix_Salad")!=std::string::npos){
	    msg.a_param =(int)(petal[agentMainWorkingPetal[ag_id]].action[agentMainWorkingAction[ag_id]].exp_deftime[ag_id]/MixingCycleDuration);
	  }
	  if (agentMainWorkingAction[ag_id]!=3)
	    msg.a_id=agentMainWorkingAction[ag_id]+100;
	}
	else{
	  if (s.find(MenuItems[0])!=std::string::npos)
	    msg.a_param=0;
	  else if (s.find(MenuItems[1])!=std::string::npos)
	    msg.a_param=1;
	  else if (s.find(MenuItems[2])!=std::string::npos)
	    msg.a_param=2;
	  else if (s.find(MenuItems[3])!=std::string::npos)
	    msg.a_param=3;
	  else if (s.find(MenuItems[4])!=std::string::npos)
	    msg.a_param=4;
	  else if (s.find(MenuItems[5])!=std::string::npos)
	    msg.a_param=5;
	}
//	ROS_INFO("send to JACO: p:%d aid:%d ap:%d as:%d ae:%d ", msg.p_id, msg.a_id, msg.a_param, msg.a_start_time, msg.a_end_time);    
//	JACO_pub->publish(msg);
      }
      else if (ag_id==NAO_ID){
	string s(petal[agentMainWorkingPetal[ag_id]].action[agentMainWorkingAction[ag_id]].name); //make a string with the name of the action

	if (s.find("Move2FruitShelf")!=std::string::npos)
	    msg.a_id=0;
	else if (s.find("Wait_Filling")!=std::string::npos)
	    msg.a_id=1;
	else if (s.find("Move2Table")!=std::string::npos)
	    msg.a_id=2;
	else if (s.find("DeliverBreakfast")!=std::string::npos)
	    msg.a_id=3;
	else if (s.find("GoHome")!=std::string::npos){
	    msg.a_id=4;
	    //the last action tha NAO handles is a_id=3
	    //therefore we just complete action locally, without sending to NAO
	    setCurrentActionComplete(NAO_ID);
            sendDaisyState();

	    return;
	}
	
//	    ROS_INFO("send to NAO: p:%d aid:%d ap:%d as:%d ae:%d ", msg.p_id, msg.a_id, msg.a_param, msg.a_start_time, msg.a_end_time);
//	    NAO_pub->publish(msg);
      }
      else if (ag_id==HUMAN_ID){
//	    HUMAN_pub->publish(msg);
      }
      
            sendDaisyState();

}


/*--------------------------------------------------------------------*/

void DP_processor::sendCancel2JACO(int ag_id){
  
  if (ag_id==JACO_ID){
      timestorm_msg::Daisy msg;
      msg.p_name="PetalName";
      msg.p_id=agentMainWorkingPetal[ag_id];
      
      msg.a_name=petal[agentMainWorkingPetal[ag_id]].action[agentMainWorkingAction[ag_id]].name;
      msg.a_id=-1; ///agentMainWorkingAction[ag_id];
      
      msg.a_start_time=petal[agentMainWorkingPetal[ag_id]].action[agentMainWorkingAction[ag_id]].start_time;

      msg.a_end_time=-1;
      msg.a_param=-1;


//      ROS_INFO("send to JACO: p:%d aid:%d ap:%d as:%d ae:%d ", msg.p_id, msg.a_id, msg.a_param, msg.a_start_time, msg.a_end_time);    
//      JACO_pub->publish(msg);
  }
  
     sendDaisyState();

}





/*--------------------------------------------------------------------*/

void DP_processor::sendDaisyState(){
  
    timestorm_msg::Daisy_graph msg;
      
    msg.p_num=petalNumber;
    msg.pet.resize(petalNumber);

    for (int i=0; i<petalNumber; i++){
      msg.pet[i].p_id=petal[i].id;	  
      msg.pet[i].p_assigned=petal[i].assigned;
      msg.pet[i].p_completed=petal[i].completed;
      msg.pet[i].p_interupted=petal[i].interupted;
      msg.pet[i].p_inplan=petal[i].inplan;
      
      msg.pet[i].a_num=petal[i].length;
      msg.pet[i].act.resize(petal[i].length);

        for (int j=0; j<petal[i].length; j++){
	   msg.pet[i].act[j].a_id=petal[i].action[j].id;
	   msg.pet[i].act[j].a_done=petal[i].action[j].done;
	   msg.pet[i].act[j].a_assigned=petal[i].action[j].assigned;
	   msg.pet[i].act[j].a_blocked=petal[i].action[j].blocked;
	   msg.pet[i].act[j].a_interupted=petal[i].action[j].interupted;
	   msg.pet[i].act[j].a_name=petal[i].action[j].name;
	   msg.pet[i].act[j].a_param=petal[i].action[j].param;
        }      
    }
    
    GRAPH_pub->publish(msg);
}



/*--------------------------------------------------------------------*/

void DP_processor::sendMenu(){
  
      timestorm_msg::Fruit_id msg;
      
      msg.fruit_id.resize(6);

   //   prepei na ginei init to megethos toy pinaka
      msg.fruit_id[0]=1;
      msg.fruit_id[1]=2;
      msg.fruit_id[2]=3;
      msg.fruit_id[3]=4;
      msg.fruit_id[4]=5;
      msg.fruit_id[5]=6;

      MENU_pub->publish(msg);
}





/*--------------------------------------------------------------------*/
void DP_processor::init(){
    //PetalID=id;
    //VertexNum=0;

    localT=0;
    for (int i=0; i<MaxPetals; i++){
      initializePetal(i);
    }
    petalNumber=0;   
    constrNum=0;
    
//    n_petalNumber=0;
//    n_constrNum=0;

    //all agents are not busy and they do not work on any petal
    for (int a=0; a<MAX_AGENTS; a++){
        agentBusyState[a]=0;
        agentMainWorkingPetal[a]=-1;
        agentMainWorkingAction[a]=-1;
        agentUrgentWorkingPetal[a]=-1;
        agentUrgentWorkingAction[a]=-1;
    }

    cout<<"to start reading"<<endl;
    readDaisy();

}



/*--------------------------------------------------------------------*/
void DP_processor::sendMessage(int ag_id, const char* mes){
  
    int t=0;
/*
    while (t<6){
      sleep(1);
      cout<<"+"<<endl;
      t++;
    }
    cout <<"connecting";
*/

    char s[300];
    bzero(s, 300);
    if (ag_id==NAO_ID){
	sprintf(s, "-PtoN-%s", mes);
	cout<< "sending to NAO"<<s<<endl;
//	sendOverTCPIP((const char*)hostNAO, portNAO, &listenNAO, &svrAddNAO, &serverNAO,s);
    }
    else if (ag_id==JACO_ID){
	sprintf(s, "-PtoJ-%s", mes);
	cout<< "sending to JACO"<<s<<endl;
//	sendOverTCPIP((const char*)hostJACO, portJACO, &listenJACO, &svrAddJACO, &serverJACO,s);
    }

}


/*--------------------------------------------------------------------*/
DP_processor::~DP_processor(){

}

/*--------------------------------------------------------------------*/

void DP_processor::initializePetal(int pid){

  
  if (pid<MaxPetals){
        petal[pid].length=0;
        petal[pid].to_cNum=0;
        petal[pid].from_cNum=0;
        petal[pid].assigned=-1; // not assigned
        petal[pid].completed=-1; // not completed
        petal[pid].inplan=0; //not considered for plan design/implementation
        petal[pid].urgency=1.0;
	petal[pid].interupted=0;
	
	for (int i=0; i<MAX_PETAL_LENGTH;i++){
	  petal[pid].action[i].assigned=-1;
	  petal[pid].action[i].blocked=0; 
  	  petal[pid].action[i].interupted=0;
  	  petal[pid].action[i].done=-1;
	  petal[pid].action[i].start_time=0;
	  petal[pid].action[i].implementation_time=0;

	  for (int a=0; a<MAX_AGENTS; a++){// expected times are initialized elsewhere, based on fuzzy times. This initialization is practically useless.
	    petal[pid].action[i].exp_deftime[a]=0; //set expected time elapsed time to zero for all agent/action pairs.
	  }

	}
  }
  else{
	cout<<"DP_processor::initializePetal() petal ID is more than allowed."<<endl;
	exit(1);
  }
}


/*--------------------------------------------------------------------*/

void DP_processor::readDaisy(){
  
  cout<<"reading file"<<endl;
  
  std::stringstream ss;
  ss << ros::package::getPath("daisy_planner").c_str() <<"/resources/DaisyPlan.txt";
  std::string filename = ss.str();  
  
//  cout<< s<<endl;
  
//  std::string filename ("DaisyPlan.txt");
//  std::ifstream infile("DaisyPlan.txt");
  std::ifstream infile(filename);
  if (!infile)
  {
    cout<<"DP_processor::readDaisy() cannot find file: "<<filename << endl;
    exit(0);
//    throw std::runtime_error("..");
  }
//  FILE* infile;
//  infile=fopen("DaisyPlan.txt", "r");
//  if (!infile){
//    cout<< "file to read was not found"<< endl;
//    exit(1);
//  }

  std::string line;
  int c=0;

  while (std::getline(infile, line))
  {
    std::istringstream iss(line);
    cout <<c<< line<<endl;

    if (line.length()==0) //The empty line declares the end of file
	    break;
    if (line.find("#")!=std::string::npos) //any line containing an # is considered comment
	    continue;

    std::size_t found = line.find("<P>"); //a new petal start is found
    if (found!=std::string::npos)
    {
	std::string s_id ("ID=");
	std::size_t id_start = line.find_first_of(s_id)+s_id.length(); //a new petal start is found
	std::size_t id_end = line.find_first_of(";"); //a new petal start is found
	cout << "id1:"<< line.substr (id_start, id_end-id_start)<<endl;
	
	initializePetal(c);

	petal[c].id=std::stoi (line.substr (id_start, id_end-id_start),nullptr); //string to integer

	int ac=0;
        while (std::getline(infile, line))//read an action line
	{
	  std::istringstream iss(line);
	  cout <<"--"<< line<<line.length()<<endl;
	  
	  if (line.length()==0)
	    break;

	  std::size_t found1 = line.find("<Act>"); //a new action start is found
	  std::size_t found2 = line.find("</Act>"); //an action end is found
	  cout <<found1<<",,"<<found2<<endl;
	  if ((found1!=std::string::npos) && (found2!=std::string::npos)){ //read action features
	    
	   petal[c].action[ac].id=ac;
	     
	    std::string s_name ("name=");
	    std::size_t name_start = line.find_first_of(s_name)+s_name.length(); //a new petal start is found
	    std::size_t name_end = line.find_first_of(";"); //a new petal start is found
	    petal[c].action[ac].name=line.substr (name_start, name_end-name_start);
	    cout << "part1:"<< line.substr (name_start, name_end-name_start)<<endl;
	    
	    line=line.substr (name_end+1);
	      
	    std::string s_obj ("obj=");
	    std::size_t obj_start = line.find_first_of(s_obj)+s_obj.length(); //a new petal start is found
	    std::size_t obj_end = line.find_first_of(";"); //a new petal start is found
	    petal[c].action[ac].obj=line.substr (name_start, name_end-name_start);
	    cout << "part2:"<< line.substr (obj_start, obj_end-obj_start)<<endl;

	    line=line.substr (obj_end+1);

	    std::string s_type ("type=");
	    std::size_t type_start = line.find_first_of(s_type)+s_type.length(); //a new petal start is found
	    std::size_t type_end = line.find_first_of(";"); //a new petal start is found
//	    cout<<obj_start <<" "<<obj_end<<endl;
	    cout << "part3:"<< line.substr (type_start, type_end-type_start)<<endl;

	    std::string tmps=line.substr (type_start, type_end-type_start);
	    if (tmps.find("A_Pre")!=std::string::npos){
	      petal[c].action[ac].type=A_Pre;
	    }
	    if (tmps.find("A_MainIntAble")!=std::string::npos){
	      petal[c].action[ac].type=A_MainIntAble;
	    }
	    if (tmps.find("A_MainImCompl")!=std::string::npos){
	      petal[c].action[ac].type=A_MainImCompl;
	    }
	    if (tmps.find("A_Post")!=std::string::npos){
	      petal[c].action[ac].type=A_Post;
	    }

	    line=line.substr (type_end+1);
	    cout <<line <<endl;
	    
	    while (line.find("<Ag>")!=std::string::npos){
	      std::string s_iid ("id=");
	      std::size_t iid_start = line.find_first_of(s_iid)+s_iid.length(); //a new petal start is found
	      std::size_t iid_end = line.find_first_of(";"); 
//	    	cout<<obj_start <<" "<<obj_end<<endl;
//	      cout << "part4:"<< line.substr (iid_start, iid_end-iid_start)<<endl;
	      int agent_id=std::stoi (line.substr (iid_start, iid_end-iid_start),nullptr); //string to integer
	      cout << "part4:"<< agent_id <<endl;


	      line=line.substr (iid_end+1);
      
      	      std::string s_time ("time=(");
	      std::size_t time_start = line.find_first_of(s_time)+s_time.length(); //a new petal start is found
	      std::size_t time_end = line.find_first_of(");"); 
//	    	cout<<obj_start <<" "<<obj_end<<endl;
	      cout << "part5:"<< line.substr (time_start, time_end-time_start)<<endl;

	      std::stringstream ss(line.substr (time_start, time_end-time_start));
	      ss >> petal[c].action[ac].expected_ftime[agent_id].s >>  petal[c].action[ac].expected_ftime[agent_id].m1 >> petal[c].action[ac].expected_ftime[agent_id].m2 >>petal[c].action[ac].expected_ftime[agent_id].e;	      
	      
	      line=line.substr (time_end+2);

	      std::string s_rob("robust=");
	      std::size_t rob_start = line.find_first_of(s_rob)+s_rob.length(); //a new petal start is found
	      std::size_t rob_end = line.find_first_of(");"); 
	      std::stringstream stmp(line.substr (rob_start, rob_end-rob_start));
	      stmp >> petal[c].action[ac].robust[agent_id] ; 
	      cout << "part6:"<< petal[c].action[ac].robust[agent_id]<<endl;

	      line=line.substr (rob_end+1);
	      
	    }
	        
	    petal[c].action[ac].assigned=-1;
	    petal[c].action[ac].done=-1;
	    petal[c].action[ac].blocked=0;
	    petal[c].action[ac].interupted=0;
	    
	    ac++;
	    petal[c].length=ac;
	  }
	  else if (line.find_first_of("</P>")!=std::string::npos){ //this is petal closure
	    cout<< "break"<<endl;
	    break;
	  }

	  else{
	    cout<<"DP_processor::readDaisy(), Error in action definition in line: "<<line<<endl;
	  }
	  
	}
    
      c++;
      petalNumber=c;
    }
    else if (line.find_first_of("<C>")!=std::string::npos){ //this is constraint

	    std::string s_fromP ("fromP=");
	    std::size_t fromP_start = line.find_first_of(s_fromP)+s_fromP.length(); //a new petal start is found
	    std::size_t fromP_end = line.find_first_of(";"); //a new petal start is found
//	    n_petal[c].action[ac].obj=line.substr (name_start, name_end-name_start);
//	    cout << "c1:"<< line.substr (fromP_start, fromP_end-fromP_start)<<endl;
	    
	    constr[constrNum].fpetal=std::stoi (line.substr (fromP_start, fromP_end-fromP_start),nullptr);

	    line=line.substr (fromP_end+1);
	    
	    std::string s_fromAct ("fromAct=");
	    std::size_t fromAct_start = line.find_first_of(s_fromAct)+s_fromAct.length(); //a new petal start is found
	    std::size_t fromAct_end = line.find_first_of(";"); //a new petal start is found
//	    cout << "c2:"<< line.substr (fromAct_start, fromAct_end-fromAct_start)<<endl;
	    constr[constrNum].fvertex=std::stoi (line.substr (fromAct_start, fromAct_end-fromAct_start),nullptr);

	    line=line.substr (fromAct_end+1);
	    
	    std::string s_toP ("toP=");
	    std::size_t toP_start = line.find_first_of(s_toP)+s_toP.length(); //a new petal start is found
	    std::size_t toP_end = line.find_first_of(";"); //a new petal start is found
//	    cout << "c3:"<< line.substr (toP_start, toP_end-toP_start)<<endl;
	    constr[constrNum].tpetal=std::stoi (line.substr (toP_start, toP_end-toP_start),nullptr);

	    line=line.substr (toP_end+1);
	    
	    std::string s_toAct ("toAct=");
	    std::size_t toAct_start = line.find_first_of(s_toAct)+s_toAct.length(); //a new petal start is found
	    std::size_t toAct_end = line.find_first_of(";"); //a new petal start is found
//	    cout << "c4:"<< line.substr (toAct_start, toAct_end-toAct_start)<<endl;
	    constr[constrNum].tvertex=std::stoi (line.substr (toAct_start, toAct_end-toAct_start),nullptr);

	    line=line.substr (toAct_end+1);
	    
	    std::string s_time ("time=(");
	    std::size_t time_start = line.find_first_of(s_time)+s_time.length(); //a new petal start is found
	    std::size_t time_end = line.find_first_of(");"); 
//	    	cout<<obj_start <<" "<<obj_end<<endl;
//	    cout << "c5:"<< line.substr (time_start, time_end-time_start)<<endl;

	    std::stringstream ss(line.substr (time_start, time_end-time_start));
	    ss >>  constr[constrNum].time.s >>  constr[constrNum].time.m1 >> constr[constrNum].time.m2 >>constr[constrNum].time.e;	      
	      
	    line=line.substr (time_end+2);
	    
	    std::string s_state ("state=");
	    std::size_t state_start = line.find_first_of(s_state)+s_state.length(); //a new petal start is found
	    std::size_t state_end = line.find_first_of(";"); //a new petal start is found
//	    cout << "c6:"<< line.substr (state_start, state_end-state_start)<<endl;
	    constr[constrNum].stateOn=std::stoi (line.substr (state_start, state_end-state_start),nullptr);

	    line=line.substr (state_end+1);

    	    constr[constrNum].prevents=-1; //currently (in execution time) no action is prevented by the constraint

/*	      std::string s_state ("state=");
	      std::size_t state_start = line.find_first_of(s_state)+s_state.length(); //a new petal start is found
	      std::size_t state_end = line.find_first_of(";"); //a new petal start is found
	      cout << "c6:"<< line.substr (state_start, state_end-state_start)<<endl;

	      line=line.substr (state_end+1);
	      */

	    linkConstraint2Petals(constrNum);
	    
	    constrNum++;
	    if (constrNum>=MAX_CONSTRAINTS){
		cout<<"DP_processor::readDaisy() Not enough space for Constriants. Error!";
		exit(0);
	    }
    }

  }
  cout<<"showing"<< endl;  
  showPetals(); 
}


/*--------------------------------------------------------------------*/
void DP_processor::linkConstraint2Petals(int constrNum){
  
    constr[constrNum].prevents=-1;
    int k=constr[constrNum].fpetal;
    petal[k].pcToL[petal[k].to_cNum]=constrNum;
	    
//    cout<<"petal"<<k<<"  "<< petal[k].to_cNum<<"-th constraint. Mapped on "<< petal[k].pcToL[petal[k].to_cNum]<<"-th GLconstraint."<<endl;            
//    cout<<petal[k].pcToL[petal[k].to_cNum]<<":("<<constr[petal[k].pcToL[petal[k].to_cNum]].fpetal<<"." <<constr[petal[k].pcToL[petal[k].to_cNum]].fvertex <<"->"<< constr[petal[k].pcToL[petal[k].to_cNum]].tpetal<<"." << constr[petal[k].pcToL[petal[k].to_cNum]].tvertex <<")      ";

    petal[k].to_cNum++;

    k=constr[constrNum].tpetal;
    petal[k].pcFromL[petal[k].from_cNum]=constrNum;
    petal[k].from_cNum++;
}


/*--------------------------------------------------------------------*/
int DP_processor::addNewPetal(int petLen){

  int npid; // the id of the new petal to be added.

  if (petalNumber+1<MaxPetals){
    if (petLen<=MAX_PETAL_LENGTH){
      npid=petalNumber; 

      initializePetal(npid);
      
      petalNumber++; //increase the total number of petals
    }
    else{
	cout<<"DP_processor::addNewPetal(). The requested length for new petal is more than the available MAX_PETAL_LENGTH!"<<endl;
	exit(1);
    }
  }
  else{
	cout<<"DP_processor::addNewPetal() no space for more petals. MaxPetals already reached."<<endl;
	exit(1);
  }
  return npid;
}




/*--------------------------------------------------------------------*/
//Find the best petal for the agent aid to work next

int DP_processor::toDoPetal(int aid)
{
    FuzzyTimeTBX ftbx(1);

    float bestfit=1000000; 
    int bestid=-1;
    for (int i=0; i<petalNumber; i++){
//     cout<< "petal:"<<i<<" urg"<<petal[i].urgency<<" inplan"<<petal[i].inplan<<" ass"<<petal[i].assigned<<" compl"<<petal[i].completed<<endl;
//        if ((petal[i].assigned<0) && (petal[i].completed<0)){ //if the petal is not assigned to an agent and not already completed
        if ((petal[i].inplan>0) && (petal[i].assigned<0) && ((petal[i].completed<0)/* || (petal[i].completed==UnknownAgent)*/ )){ //if the petal is not assigned to an agent and not already completed
//        if ((petal[i].assigned<0) ){ //if the petal is not assigned to an agent and not already completed
            //if (getActiveConstrOnPetal(i)<1){ //if there are no constraints on this petal
//	  cout<<"p"<<i<<" inp:"<<petal[i].inplan<<" ass:"<<petal[i].assigned<<" comp:"<<petal[i].completed<<endl;
//	  cout<< "not assigned, not completed found for robot "<<aid<<endl;
                float fit=ftbx.defuz(petal[i].petal_time[aid])/(1+petal[i].petal_robustness[aid]); //currently does not consider expected_time
//		cout <<"p"<<i<<" -time:"<<ftbx.defuz(petal[i].petal_time[aid])<<" rob:"<<petal[i].petal_robustness[aid];
		if (MAX_AGENTS==2){
		  if (aid==0){
		    fit= (petal[i].urgency/max_urgency)*ftbx.defuz(petal[i].petal_time[aid])/(1+petal[i].petal_robustness[aid]) - ftbx.defuz(petal[i].petal_time[1])/(1+petal[i].petal_robustness[1]);
//		  	cout<<"petal:"<<i<<" fit0 :"<<fit<<endl;
		  }
		  if (aid==1){
		    fit= (petal[i].urgency/max_urgency)*ftbx.defuz(petal[i].petal_time[aid])/(1+petal[i].petal_robustness[aid]) - ftbx.defuz(petal[i].petal_time[0])/(1+petal[i].petal_robustness[0]);
//		  	cout<<"petal:"<<i<<" fit1 :"<<fit<<endl;
		  }
		}
		if (MAX_AGENTS==3){
		  if (aid==0){
		    float f1,f2;
		    f1= ftbx.defuz(petal[i].petal_time[aid])/(1+petal[i].petal_robustness[aid]) - ftbx.defuz(petal[i].petal_time[1])/(1+petal[i].petal_robustness[1]);
//		    cout<<"def:"<< ftbx.defuz(petal[i].petal_time[aid])<<" r:"<<petal[i].petal_robustness[aid]<<"   -   def:"<<ftbx.defuz(petal[i].petal_time[1])<< " r:"<<petal[i].petal_robustness[1]<< "  u:"<<petal[i].urgency << endl;
		    f2= ftbx.defuz(petal[i].petal_time[aid])/(1+petal[i].petal_robustness[aid]) - ftbx.defuz(petal[i].petal_time[2])/(1+petal[i].petal_robustness[2]);
//		    cout<<"def:"<< ftbx.defuz(petal[i].petal_time[aid])<<" r:"<<petal[i].petal_robustness[aid]<<"   -   def:"<<ftbx.defuz(petal[i].petal_time[2])<< " r:"<<petal[i].petal_robustness[2]<< "  u:"<<petal[i].urgency <<endl;
		    if (f1<f2)
//		      fit=f1;
		      fit=(petal[i].urgency/max_urgency)*f1;
		    else
//		      fit=f2;
		      fit=(petal[i].urgency/max_urgency)*f2;
		    
//		    cout<<"a["<<aid<<"] p["<<i<<"]:"<<f1<<","<<f2<<"   -"<<fit<<endl;
		  }
		  if (aid==1){
		    float f0,f2;
		    f0= ftbx.defuz(petal[i].petal_time[aid])/(1+petal[i].petal_robustness[aid]) - ftbx.defuz(petal[i].petal_time[0])/(1+petal[i].petal_robustness[0]);
		    f2= ftbx.defuz(petal[i].petal_time[aid])/(1+petal[i].petal_robustness[aid]) - ftbx.defuz(petal[i].petal_time[2])/(1+petal[i].petal_robustness[2]);
		    if (f0<f2)
		      fit=(petal[i].urgency/max_urgency)*f0;
		    else
		      fit=(petal[i].urgency/max_urgency)*f2;
		  }
		  if (aid==2){
		    float f0,f1;
		    f0= ftbx.defuz(petal[i].petal_time[aid])/(1+petal[i].petal_robustness[aid]) - ftbx.defuz(petal[i].petal_time[0])/(1+petal[i].petal_robustness[0]);
		    f1= ftbx.defuz(petal[i].petal_time[aid])/(1+petal[i].petal_robustness[aid]) - ftbx.defuz(petal[i].petal_time[1])/(1+petal[i].petal_robustness[1]);
//		    cout<< "f0 :"<<f0<<"  u:"<<petal[i].urgency ;
//		    cout<< "f1 :"<<f1<<"  u:"<<petal[i].urgency ;
		    if (f0<f1)
		      fit=(petal[i].urgency/max_urgency)*f0;
		    else
		      fit=(petal[i].urgency/max_urgency)*f1;
		  }
		}
		
//		cout << "petal"<<i<<" fit: "<<fit<< "\n\n"<<endl;
                if (fit<bestfit){
                    bestfit=fit;   //  *************************8  einai lathos oi times twn xronwn kai robustness    *****************
                    bestid=i;
//                    cout << "better petal found: (" << bestid<<","<<bestfit<<")   ";
                }
//                    cout <<"["<<petal[i].petal_time[aid].s <<","<<petal[i].petal_time[aid].m1 <<","<<petal[i].petal_time[aid].m2 <<","<<petal[i].petal_time[aid].e <<"\n";
            //}

        }
    }
//    cout << "Best petal : " << bestid <<"\n";
    if (bestid>-1) { //a petal completely free of constraints has been found
        return bestid;
    }
    else{ //find the petal with the less number of constraints
        for (int i=0; i<petalNumber; i++){
            int lessConst=100000;
            if ((petal[i].inplan>0) && (petal[i].assigned<0) && (petal[i].completed<0)){
                 int curConst;
                if (CONSTRAINT_HANDLING==0){
                    curConst=getActiveConstrOnPetal(i);
//		    cout << "Active constraints on petal[" <<i<<"]:"<< curConst<<endl;
		}
                else if (CONSTRAINT_HANDLING==1){
		  curConst=getConstrTimeOnPetal(aid,i);
//		  cout << "found constraint on petal["<< i <<"] with time cost:" << curConst<<endl;
                }
//		getchar();
                if (curConst<lessConst){
		  lessConst=curConst;
		  bestid=i;		  
		}
            }
        }
        return bestid;
    }

}


/*--------------------------------------------------------------------*/
//Resets expected implementation time for all agents
void DP_processor::resetAllAgentTime(int pid)
{
    for (int i=0; i<MAX_AGENTS; i++){
        resetAgentTime(pid, i);
    }
}

/*--------------------------------------------------------------------*/
//Resets expected implementation features for all agents
void DP_processor::resetAllAgentFeatures(int pid)
{
    for (int i=0; i<MAX_AGENTS; i++){
        petal[pid].petal_robustness[i]=0;
    }
}

/*--------------------------------------------------------------------*/
//Resets expected implementation time for all agents
void DP_processor::resetAgentTime(int pid, int aid)
{
        petal[pid].petal_time[aid].s=0;
        petal[pid].petal_time[aid].m1=0;
        petal[pid].petal_time[aid].m2=0;
        petal[pid].petal_time[aid].e=0;
}

/*--------------------------------------------------------------------*/
//Resets expected implementation features for all agents
void DP_processor::resetAgentFeatures(int pid, int aid)
{
        petal[pid].petal_robustness[aid]=1000000;
}


/*--------------------------------------------------------------------*/
//this function collects information about the petal.
//Information regards the total time of execution and the minimum robustness
void DP_processor::updatePetalInfo(int pid, int aid)
{

    resetAgentTime(pid,aid);
    resetAgentFeatures(pid,aid);

    FuzzyTimeTBX ftbx(1);
    struct FuzzyNumber r;
//    cout <<"starting petal:"<<pid <<"\n";
    int eid;
    for (int i=0; i<petal[pid].length; i++){
//        eid=petal[pid].action[i].time[aid].s;
//        r=ftbx.get_fuzzy_time(cg, eid,aid);

        petal[pid].petal_time[aid].s+=petal[pid].action[i].expected_ftime[aid].s;
        petal[pid].petal_time[aid].m1+=petal[pid].action[i].expected_ftime[aid].m1;
        petal[pid].petal_time[aid].m2+=petal[pid].action[i].expected_ftime[aid].m2;
        petal[pid].petal_time[aid].e+=petal[pid].action[i].expected_ftime[aid].e;

//	cout <<" petal["<<pid<<"][act"<<i<<"[ag"<<aid<<"]="<<petal[pid].action[i].time[aid].m1;
    
/*        char stuckstr[30];
        sprintf(stuckstr, "rU%d", aid);
        int atribv=EAN(cg, stuckstr, eid);
*/
	int atribv=petal[pid].action[i].robust[aid];
        if (atribv<petal[pid].petal_robustness[aid])
            petal[pid].petal_robustness[aid]=atribv;
    }
//    cout << "for petal["<<pid << "] time" <<petal[pid].petal_time[aid].s <<",...,"<< petal[pid].petal_time[aid].e <<")\n";
//    cout << "for petal["<<pid << "] robustness:" <<petal[pid].robustness[aid] <<"\n";
//cout<<endl;
//    cout <<"petal["<<pid<<"][a"<<aid<<"]="<<ftbx.defuz(petal[pid].petal_time[aid]);
}

/*--------------------------------------------------------------------*/
//this function collects information about the petal.
//Information regards the total time of execution and the minimum robustness
void DP_processor::updateAllPetalInfo()
{

    for (int i=0; i<petalNumber; i++){
        for (int j=0; j<MAX_AGENTS; j++){
            updatePetalInfo(i, j);
        }
    }
}


/*--------------------------------------------------------------------*/
//this function collects information about the petal.
//Information regards the total time of execution and the minimum robustness
void DP_processor::n_updateExpectedTimes()
{
    FuzzyTimeTBX ftbx(1);

    for (int i=0; i<petalNumber; i++){
        for (int j=0; j<petal[i].length; j++){
                for (int a=0; a<MAX_AGENTS; a++){
                    int eid=j;
                    petal[i].action[j].exp_deftime[a]=ftbx.defuz(petal[i].action[j].expected_ftime[a]);		    
            }
        }
    }
    showExpectedTimes();
//    getchar();
}

/*--------------------------------------------------------------------*/

void DP_processor::adjustExpectedTimes(int pid, int acid, float percent)
{
    FuzzyTimeTBX ftbx(1);

    for (int a=0; a<MAX_AGENTS; a++){
        petal[pid].action[acid].exp_deftime[a]=petal[pid].action[acid].exp_deftime[a]*percent;
    }
}


/*--------------------------------------------------------------------*/

int  DP_processor::showPetals(){

    for (int i=0; i<petalNumber; i++){
        cout<<"\n\n ----petal[" << i <<"], length:" << petal[i].length;
        cout<<"  assigned:" << petal[i].assigned<<"  inPlan:" << petal[i].inplan;

	cout<<"\nedges:";
        for (int j=0; j<petal[i].length; j++){
//            cout<<petal[i].edge[j] << "  ";
	    
	    cout<<petal[i].action[j].name << "  ";
        }
        
        cout<<"\nconstrains vertices:";
        for (int j=0; j<petal[i].to_cNum; j++){
            cout<<petal[i].pcToL[j]<<":("<<constr[petal[i].pcToL[j]].fvertex  <<"->"<< constr[petal[i].pcToL[j]].tvertex <<")      ";
        }
        cout<<"\nconstrained by vertices:";
        for (int j=0; j<petal[i].from_cNum; j++){
            cout<<petal[i].pcFromL[j]<<":("<<constr[petal[i].pcFromL[j]].fvertex <<"->"<< constr[petal[i].pcFromL[j]].tvertex <<")    ";
        }
    }
}

/*--------------------------------------------------------------------*/

void DP_processor::setPetalsInPlan(){

  	petal[0].inplan=1;
  	petal[1].inplan=1;
  	petal[2].inplan=1;
  	petal[3].inplan=1;
  	petal[4].inplan=1;
  	petal[5].inplan=1;
/*
    for (int i=0; i<petalNumber; i++){
      if (i==0){ // This is "go-get-fruits" attributed to NAO
	petal[0].inplan=1;
	petal[0].i_type=PT_Intervene;
	for (int j=0; j<petal[i].length; j++){
	  petal[i].a_type[j]=A_MainImCompl;
	}
      }
      if (i==1){
	petal[1].inplan=1;
	petal[1].i_type=PT_Interupt;
	petal[1].a_type[0]=A_MainImCompl; //SayPleaseClean
	petal[1].a_type[1]=A_MainIntAble; //CleanTable
	petal[1].a_type[2]=A_MainImCompl; //SayThankYou
	petal[1].a_type[3]=A_Post; //GoHome / Rest	
      }
      if (i==2){
	petal[2].inplan=1;
	petal[2].i_type=PT_Interupt;
	petal[2].a_type[0]=A_Pre; //Grasp
	petal[2].a_type[1]=A_Pre; //Pick
	petal[2].a_type[2]=A_MainImCompl; //Place
	petal[2].a_type[3]=A_Post; //GoHome / Rest
      }
      if (i==3){
	petal[3].inplan=1;
	petal[3].i_type=PT_Interupt;
	petal[3].a_type[0]=A_Pre; //Grasp
	petal[3].a_type[1]=A_Pre; //Pick
	petal[3].a_type[2]=A_MainImCompl; //Place
	petal[3].a_type[3]=A_Post; //GoHome / Rest
      }
      if (i==4){
	petal[4].inplan=1;
	petal[4].i_type=PT_Interupt;
	petal[4].a_type[0]=A_Pre; //Grasp
	petal[4].a_type[1]=A_Pre; //Pick
	petal[4].a_type[2]=A_MainImCompl; //Place
	petal[4].a_type[3]=A_Post; //GoHome / Rest
      }
      if (i==5){
	petal[5].inplan=0;
	petal[5].i_type=PT_Interupt;
	petal[5].a_type[0]=A_Pre; //Grasp
	petal[5].a_type[1]=A_Pre; //Pick
	petal[5].a_type[2]=A_MainImCompl; //Place
	petal[5].a_type[3]=A_Post; //GoHome / Rest
      }
    }
    */
}


/*--------------------------------------------------------------------*/

void DP_processor::showExpectedTimes(){

    for (int i=0; i<petalNumber; i++){
        if (petal[i].assigned<0){
            cout<<"\n\n ----ExpectedTimes petal[" << i <<"], length:" << petal[i].length;
            for (int a=0; a<MAX_AGENTS; a++){
                cout<<"\nAgent["<<a<<"] ";
                for (int j=0; j<petal[i].length; j++){  //+1
                    cout<<petal[i].action[j].exp_deftime[a] << "  ";
                }
            }
        }
    }
}

/*--------------------------------------------------------------------*/

int DP_processor::getPetalExpectedTime(int agid, double *implemented, double* expected){

  *implemented=0.0;
  *expected=0.0;
  int pid;
  
  if (agentUrgentWorkingPetal[agid]<0) //if there is no emergency job
    pid=agentMainWorkingPetal[agid];  //the agent works on the main petal
  else
    pid=agentUrgentWorkingPetal[agid]; //else, it works on the Emergency etal

  //  cout<<"ag:"<<agid<<" works on:"<<pid <<" that is assigned to:"<< petal[pid].assigned<<endl;
  
   if (petal[pid].assigned>=0){
     for (int j=0; j<petal[pid].length; j++){  //+1	  
//       cout<<"     ["<<pid<<"]["<<j<<"]:("<<petal[pid].implementation_time[j]<<","<<petal[pid].expected_time[agid][j]<<")";
       if (petal[pid].action[j].done>=0)	    
	 *implemented+=petal[pid].action[j].implementation_time;	  
       else	    
	 *expected+=petal[pid].action[j].exp_deftime[agid];
     }
   }
   else{
	  *implemented=0.0;
	  *expected=0.0;
	  return -1;
   }
  return 1;
}

/*--------------------------------------------------------------------*/

int DP_processor::getActionRemainingTime(int pid, int actid, int agid, float* remt){

  *remt=0.0;

//       cout<<"     ["<<pid<<"]["<<j<<"]:("<<petal[pid].implementation_time[j]<<","<<petal[pid].expected_time[agid][j]<<")";
       if (petal[pid].action[actid].done>=0){	//fully completed    
	 *remt=0;
       }
       else if ((petal[pid].action[actid].done<0) && (petal[pid].action[actid].assigned>=0)){ //assigned but not yet completed
	 *remt=petal[pid].action[actid].exp_deftime[agid]-petal[pid].action[actid].implementation_time;	  
       }
       else{ //not yet assigned
	 *remt=petal[pid].action[actid].exp_deftime[agid];
       }
  return 1;
}

/*--------------------------------------------------------------------*/
void DP_processor::setRemainingTimeForAction(int agentid, int pid, int actid, int remT){

  if ((remT<0) || (remT>1000)){
    cout << "DP_processor::setRemainingTimeForAction(), negative remaing time. Possible error."<<endl; 
  }
  else{
    cout << "expected time before: " <<petal[pid].action[actid].exp_deftime[agentid]<<endl;
    cout << "time0: " <<time(0)<<" start:" <<petal[pid].action[actid].start_time<< " rem:"<<remT;//
    petal[pid].action[actid].exp_deftime[agentid]=time(0)-petal[pid].action[actid].start_time+remT;  
    petal[pid].action[actid].exp_deftime[agentid]=time(0)-petal[pid].action[actid].start_time+remT;  
    cout << "expected time updated to: " <<petal[pid].action[actid].exp_deftime[agentid]<<endl;
  }
}

/*--------------------------------------------------------------------*/
//Return the expected time from "now", that an action will be completed
float DP_processor::getExpectedRemainingTimeForAction(int agentid, int pid, int actid){
  
//  cout <<"getting ExpectedRemainingTimeForAction() agentid:"<<agentid<< " petal:"<< pid<<" actid:"<< actid;
  
  if (pid<petalNumber){
    if (actid<petal[pid].length){
      if (petal[pid].action[actid].done>=0){ //the action is already implemented
	return 0.0;
      }
      if (petal[pid].assigned>=0){
	if (petal[pid].assigned!=agentid){ //the petal is assigned to another agent
	  return -1;
	}
	else{ //the petal is assigned to agent agentid
	  float rt=0.0;
//      	  for (int j=0; j<petal[pid].length; j++){
//	    cout<<"   ["<<j<<"]:"<<petal[pid].start_time[j];
//	  }
	  
      	  for (int j=0; j<petal[pid].length; j++){
//	    cout <<"j:"<<j<<"  rt:"<<rt << "               ";
	    if (petal[pid].action[j].done>=0){
//	      cout << "action:("<<pid<<","<<j<<")-done"<<endl;
	      continue;
	    }
    	    if (petal[pid].action[j].assigned<0) { //action not assigned
	      rt+= petal[pid].action[j].exp_deftime[agentid];
//	      cout<<"*"<<j<<" t:"<< petal[pid].action[j].exp_deftime[agentid]<<"|";
	    }
	    else if ((petal[pid].action[j].assigned>=0) && (petal[pid].action[j].done<0)){ //action assigned but not fully completed
	      int temp=petal[pid].action[j].start_time-time(0)+petal[pid].action[j].exp_deftime[agentid];
//	      cout<<"temp:"<<temp<<"  ";
	      rt+= (float)temp;//petal[pid].start_time[j]+(float)petal[pid].expected_time[agentid][j]-(float)time(0);
//	      cout<<"#"<<j<<" tt:" << petal[pid].action[j].start_time<<" - "<< petal[pid].action[j].exp_deftime[agentid]<< " - "<< time(0) <<"|";
	    }
//	    cout <<"j:"<<j<< " rt:"<<rt<<"     \n";
            if (j==actid)      
	      break;
	  } 
	 return rt; 
	}
      }
      else{
	return MAXTIME;
      }      
    }
    else{
      cout<<"getExpectedRemainingTime() wrong action-id"<<endl;
      exit(1);
    }
  }
  else{
    cout<<"getExpectedRemainingTime() wrong petal-id"<<endl;
    exit(1);
  }
  return -1;
}

/*--------------------------------------------------------------------*/
void DP_processor::releaseConstraintsFromUnassignedgedPetals(int pid, int verid){

    for (int m=0; m<petal[pid].from_cNum; m++){
      if ((constr[petal[pid].pcFromL[m]].tvertex == verid) &&(constr[petal[pid].pcFromL[m]].stateOn==1)){  // an active constraint on this vertex has been found
        int cid=petal[pid].pcFromL[m]; //the id of the constraint
        int depp=constr[cid].fpetal; //get the departing petal of the constraint
        int c_aid=petal[depp].assigned; //agent assigned to implement petal
        if (c_aid<0){
	  constr[cid].stateOn=0;	  
	  constr[cid].prevents=-1;	  
  	  cout<<FMAG("released Rconstraint") <<cid<< FMAG("from petal")<<depp<<endl;
	  
//	  petal[depp].inplan=0; //set the petal out of the plan.....
	  petal[depp].completed=UnknownAgent;// Petal completion is assigned on purpose to an unknown agent, because the current team is no longer interested in it

	}
      }
    }    
}

/*--------------------------------------------------------------------*/
void DP_processor::releaseConstraintsFromHumanPetal(int pid, int verid){

    for (int m=0; m<petal[pid].from_cNum; m++){
      if ((constr[petal[pid].pcFromL[m]].tvertex == verid) &&(constr[petal[pid].pcFromL[m]].stateOn==1)){  // an active constraint on this vertex has been found
        int cid=petal[pid].pcFromL[m]; //the id of the constraint
        int depp=constr[cid].fpetal; //get the departing petal of the constraint
        int c_aid=petal[depp].assigned; //agent assigned to implement petal
        if (c_aid==HUMAN_ID){
	  constr[cid].stateOn=0;	  
	  constr[cid].prevents=-1;	  
  	  cout<<FMAG("released Hconstraint") <<cid<< FMAG("from petal")<<depp<<endl;
	}
      }
    }    
}


/*--------------------------------------------------------------------*/
//it returns the id of an "action" (it is action not edge) before a given vertex in a petal
int DP_processor::getActionBeforeVertex(int pid, int verid){

  int action_id=-1;
  for (int i=0; i<petal[pid].length; i++){
    if (petal[pid].action[i].id==verid){
      action_id=i-1;
      break;  
    }
  }
//  cout <<"gets pid:"<<pid<<" verid:"<<verid<< "--> returns:"<<action_id;
  return action_id;
  
}

/*--------------------------------------------------------------------*/
//Returns the expected time that a constraint on pid on vertex verid, from a petal that is currently assigned to agent other_aid will be resolved 
float DP_processor::getAgentConstrTimeOnPetal(int pid, int verid, int other_aid){

    float rt=0.0; //time for the constraint to be resolved
    int ttoc=0; //time for the agent to face the constraint
    int maxdt=-MAXTIME;

//    cout << "checking constraint time on petal"<<pid<<" vertex:"<<verid<<"by agent:"<<other_aid<< "....."<<endl; 
    
    for (int m=0; m<petal[pid].from_cNum; m++){
      if ((constr[petal[pid].pcFromL[m]].tvertex == verid) &&(constr[petal[pid].pcFromL[m]].stateOn==1) ){  // an active constraint on this vertex has been found
        int cid=petal[pid].pcFromL[m]; //the id of the constraint
        int depp=constr[cid].fpetal; //get the departing petal of the constraint
        int c_aid=petal[depp].assigned; //agent assigned to implement petal
        
//        cout <<"petal["<<depp<<"].assigned="<<petal[depp].assigned<<"  cid:"<<cid<<endl;
	if (c_aid==other_aid){
//	  int remote_action_id=getActionBeforeVertex(depp,constr[petal[pid].pcFromL[m]].fvertex);
	  int remote_action_id=constr[petal[pid].pcFromL[m]].fvertex;
//	  cout<< "remoteA "<<depp<<"."<<remote_action_id<<endl;
	  rt=getExpectedRemainingTimeForAction(other_aid, depp, remote_action_id);
//	  cout  <<"rt: "<<rt<<endl;
//	  break;
	}
	else{
//	  cout<<"oxi  "<<c_aid<< "  "<<other_aid<<"   xxx ";
	}
      }
    }    
    return rt;
}

/*--------------------------------------------------------------------*/
//Check whether human is expected to wait after considering the expected times.
//In the case that our predictions say that human will wait, we wait the current petal to complete but we release all constraints departing from non-assigned petals.
void DP_processor::checkHumanForcedConstraintRelease(int agid, int pid, int verid){

    float rt=0.0; //time for the constraint to be resolved
    float tafterc=0; //time to complete petal after the constraint
    float h_constrT=0;

//    cout << "checkHumanForcedConstraintRelease - agid:"<<agid<<" pid:"<<pid<<" verid:"<<verid<<endl;
//    getchar();
    int foundHumanConstr=0;
    
    for (int m=0; m<petal[pid].from_cNum; m++){
//      cout<<"m:"<<m<<" vert:"<<constr[petal[pid].pcFromL[m]].tvertex<<"  state:"<<constr[petal[pid].pcFromL[m]].stateOn<<endl;
      if ((constr[petal[pid].pcFromL[m]].tvertex == verid) &&(constr[petal[pid].pcFromL[m]].stateOn==1)){  // an active constraint on this vertex has been found
        int cid=petal[pid].pcFromL[m]; //the id of the constraint

        int depp=constr[cid].fpetal; //get the departing petal of the constraint
        int c_aid=petal[depp].assigned; //agent assigned to implement petalthe constraining petal


//        cout<<"active constr:"<<cid<<" from petal"<<depp<<" assigned to ag:"<<c_aid<<endl;
	if (c_aid==HUMAN_ID){
	    h_constrT=getAgentConstrTimeOnPetal(pid, constr[petal[pid].pcFromL[m]].tvertex, HUMAN_ID);
//	    cout <<"human will complete action (release constraint) in "<< h_constrT<<endl;
	    foundHumanConstr=1;
	}
	
//	float r_constrT=MAXTIME;
//	if (c_aid==JACO_ID){
//	  r_constrT = getAgentConstrTimeOnPetal(pid, constr[petal[pid].pcFromL[m]].tvertex, JACO_ID);
//	  cout <<"JACO will release constraint in "<< r_constrT<<endl;
//	}
//	if (c_aid==NAO_ID){
//	  r_constrT = getAgentConstrTimeOnPetal(pid,constr[petal[pid].pcFromL[m]].tvertex, NAO_ID);
//	  cout <<"NAO will release constraint in "<< r_constrT<<endl;	  
//	}

      }
    }

    if (foundHumanConstr==1){
      tafterc=0;
//      int action_id=getActionBeforeVertex(pid,verid);    
      int action_id=verid;    
      for (int j=action_id; j<petal[pid].length; j++){    
	if (petal[pid].action[j].done<0){
//	    cout << "time["<<pid<<"]["<<j<<"].expT:" << petal[pid].action[j].exp_deftime[agid];
	  tafterc+=petal[pid].action[j].exp_deftime[agid];
	}
      }
//      cout<< "robot will complete task "<<pid<<" in "<< tafterc << endl;

      robot2target=tafterc;
      human2target=h_constrT;
    
      if (arousalToTimeEffect*h_constrT<tafterc){ //then human is expected to wait
	cout << FMAG("H-Arous:")<<arousalToTimeEffect<< FMAG("  H-Time:")<< (int) h_constrT<<endl;
	cout<<FMAG("human will need ")<<arousalToTimeEffect*h_constrT<<FMAG(" more time but robot implementation time is: ")<<tafterc<<endl;
	//release constrants from all petals that are not assigned yet..... but wait assigned petals to finish 
	releaseConstraintsFromUnassignedgedPetals(pid, verid);
	releaseConstraintsFromHumanPetal(pid, verid);
      }
      cout << FMAG("H-Arous:")<<arousalToTimeEffect<< FMAG("  H-Time:")<< (int) h_constrT<<endl;
      cout<<FMAG("human will need aprox ")<<arousalToTimeEffect*h_constrT<<FMAG(" more time but robot implementation time is: ")<<tafterc<<endl;
//      if (moveon==0){
//	cout<<"give 0 for break, 1 for moveon: ";
//	cin>>moveon;
//      }
      
    }
    else{
      robot2target=0.0;
      human2target=0.0;
    }
}


/*--------------------------------------------------------------------*/
//Return the maximum time that constraints on petal "pid" are expected to be resolved
int DP_processor::getConstrTimeOnPetal(int my_aid, int pid){

    int rt=0; //time for the constraint to be resolved
    int ttoc=0; //time for the agent to face the constraint
    int maxdt=-MAXTIME;

//    showExpectedTimes();

//    cout<< "check constraints time for petal:"<<pid<<endl;
//    getchar();

    for (int m=0; m<petal[pid].from_cNum; m++){
        int cid=petal[pid].pcFromL[m]; //the id of the constraint
        cout << "\nc:" <<cid <", ";
        if (constr[cid].stateOn == 1){
            rt=0;
            ttoc=0;
            for (int j=0; j<petal[pid].length; j++){
                if (petal[pid].action[j].done<0)
                    ttoc+=petal[pid].action[j].exp_deftime[my_aid];
//                cout<<"action:"<<j<< " ttoc:"<<ttoc<< ",   ";
            }
//            cout<<"facing constraint in:"<<ttoc<<endl;


//            cout << "p["<<depp<<"]:";
            int depp=constr[cid].fpetal; //get the departing petal of the constraint
            int aid=petal[depp].assigned; //agent assigned to implement petal
            if ((petal[depp].inplan>0) && (aid<0)) //no agent is currently working on the petal
                return MAXTIME;
                //run departing petal to estimate the expected time that constraint will be resolved
            for (int j=0; j<petal[depp].length; j++){
                if (petal[depp].action[j].done<0){ //action is not already implemented/completed
                    rt+=petal[depp].action[j].exp_deftime[aid];
                }
//                cout << petal[depp].vertex[j] << "?" << constr[cid].fvertex << " ct:"<<ct <<"     ";
//                if (petal[depp].vertex[j]==constr[cid].fvertex)
                if (petal[depp].action[j].id==constr[cid].fvertex)
                    break;
            }
            cout << "constraint from petal "<< depp<< ", cid:" <<cid << " rt:"<<rt <<"ttoc:" <<ttoc;
        }
        if (maxdt<rt-ttoc)
            maxdt=rt-ttoc;
    }
//    cout<<"petal["<<pid<<"]-result:"<<maxdt<<endl;
    return maxdt;
}


/*--------------------------------------------------------------------*/
//Return the number of active constraints on petal "pid"
//it also checks if agent aid is located there

int DP_processor::getActiveConstrOnVertex(int pid, int vid, int aid){

    int on_number=0;

    for (int m=0; m<petal[pid].from_cNum; m++){       
//	    cout << "active constraint to:" << constr[petal[pid].pcFromL[m]].tvertex << endl;  ///m
        if ((constr[petal[pid].pcFromL[m]].stateOn == 1) && (constr[petal[pid].pcFromL[m]].tvertex==vid)) {
//	    cout << "active constraint to:" << constr[petal[pid].pcFromL[m]].tvertex << " from:"<< constr[petal[pid].pcFromL[m]].fvertex <<"    ";  ///m
            on_number++;
	    int pp=getAgentWorkingPetal(aid);
	    int act1=getAgentWorkingAction(aid);

	    int act2=getActionBeforeVertex(pid, vid);
	    if (act1==act2+1){
	      constr[petal[pid].pcFromL[m]].prevents=1;
	    }
	      
//	    cout << "action1:"<<act1 <<"  action2:"<<act2<<endl;
	    
        }
    }
    return on_number;
}


/*--------------------------------------------------------------------*/
//Return the number of active constraints on petal "pid"
int DP_processor::getActiveConstrOnPetal(int pid){

    int on_number=0;

    for (int m=0; m<petal[pid].from_cNum; m++){ //+1
        if (constr[petal[pid].pcFromL[m]].stateOn == 1){
            on_number++;
        }
    }
    return on_number;
}


/*--------------------------------------------------------------------*/
//Return the number of active constraints that petal "pid" sets to other petals.
int DP_processor::getActiveConstrFromPetal(int pid){

    int on_number=0;
    for (int m=0; m<petal[pid].to_cNum; m++){ //+1
        if (constr[petal[pid].pcToL[m]].stateOn == 1){
            on_number++;
        }
    }
    return on_number;
}


/*--------------------------------------------------------------------*/
//Sets inactive the constraint arivinng in petal "to_pid" sets to other petals.
int DP_processor::setInactiveConstraintToPetal(int pid, int tov){

    for (int m=0; m<petal[pid].from_cNum; m++){ //+1
        if (constr[petal[pid].pcFromL[m]].tvertex == tov){
            constr[petal[pid].pcFromL[m]].stateOn =0;
        }
    }
    return 1;
}


/*--------------------------------------------------------------------*/
int DP_processor::getAgentWorkingPetal(int agent_id){

  if ((agent_id==HUMAN_ID) || (agent_id==NAO_ID) || (agent_id==JACO_ID))
  {
    if (agentUrgentWorkingPetal[agent_id]<0)
      return agentMainWorkingPetal[agent_id];
    else
      return agentUrgentWorkingPetal[agent_id];      
  }
  else {
    cout <<"DP_processor::getAgentWorkingPetal() wrong agent id"<<endl;
    exit(-1);
  }
}

/*--------------------------------------------------------------------*/
int DP_processor::getAgentWorkingAction(int agent_id){

  if ((agent_id==HUMAN_ID) || (agent_id==NAO_ID) || (agent_id==JACO_ID))
    return agentMainWorkingAction[agent_id];
  else {
    cout <<"DP_processor::getAgentWorkingPetal() wrong agent id"<<endl;
    exit(-1);
  }
}



/*--------------------------------------------------------------------*/
int DP_processor::simulateRun(){

    time_t gstartT = time(0);
    time_t gcurrentT = time(0);

    static int human_work_assigned=0;
//    cout <<"human_work_assigned: "<<human_work_assigned<<endl;
#ifdef LocalTesting    
//    cout << "Run planner without ICE communicaion"<<endl;
#else
    // initialize the ICE-based communication with armarX
//    icePlanCommunication();
#endif
    
    std::cout <<"Planning is ready to start." << endl;
    std::cout << "Press:  1" <<endl;
    getchar();
//    cout <<"\n\n\n\n\n\n\n\n\n";
    
    cout <<"petals:"<<petalNumber<<"  ";
    ros::Rate loop_rate(5);
    
    while (ros::ok()){
//    while (1){

      gcurrentT = time(0);
      int d=gcurrentT-gstartT;
      if (d>=localT+1){ // assume discrete time, increased every second
         localT=d;
         cout << localT << "  ";
         cout.flush();
	 

	 max_urgency+=1.0;
	 for (int p=0; p<petalNumber; p++){
	   if ((petal[p].assigned<0) && (petal[p].completed<0))
	    petal[p].urgency+=1.9;
	 }

//	 cout <<"U[4]=" << petal[4].urgency;
//	 cout <<"   U[5]=" << petal[5].urgency<< endl;

/*
	 if (petal[5].inplan==0){
	   if (localT>15){
	     petal[5].inplan=1;
	    petal[5].urgency=max_urgency;
	   }
	 }
*/	 
//	 cout<< "done[1][1]="<<petal[1].a_done[1]<<"expected time:"<< petal[1].expected_time[0][1] <<"  implementation time:"<<petal[1].implementation_time[1]<<endl;
        //all agents are assigned a task/petal
         for (int a=0; a<MAX_AGENTS; a++){
//	    if (a==NAO_ID){          
//		cout << "agent[" << a<<"].busy:"<<agentBusyState[a]<<endl;
//	    }
            if (agentBusyState[a]==0){
                int pid=-1;//toDoPetal(a);
		if (a==HUMAN_ID+7777){
		  if (!human_work_assigned) {
		    human_work_assigned=1;
		    pid=toDoPetal(a);
		    cout << FRED("task") << pid << FRED("assigned to agent ") << a<<"\n";
		  }
		}
		else if (a==NAO_ID) {         
                  pid=toDoPetal(a);
		  cout << FGRN("task") << pid << FGRN("assigned to agent ") << a<<"\n";
		}
		else if (a==JACO_ID){          
		  pid=toDoPetal(a);
		  cout << FCYN("task") << pid << FCYN("assigned to agent ") << a<<"\n";
		}
		else{          
		  pid=toDoPetal(a);
		  cout << FRED("task") << pid << FRED("assigned to agent ") << a<<"\n";
		}
		
                if (pid>=0){
//		    cout << "before.. .. task" << pid << ".interupted=" << petal[pid].interupted<<"\n"; 
                    petal[pid].assigned=a;
		    petal[pid].interupted=0;

//		    cout << "task" << pid << ".interupted=" << petal[pid].interupted<<"\n";

		    
		    for (int i=0; i<MAX_PETAL_LENGTH;i++){
		      petal[pid].action[i].assigned=-1;
		      petal[pid].action[i].blocked=0;
		      petal[pid].action[i].interupted=0;
		      petal[pid].action[i].done=-1;
		      petal[pid].action[i].start_time=0;
		      petal[pid].action[i].implementation_time=0;
		    }
		    
                    agentMainWorkingPetal[a]=pid;
                    agentBusyState[a]=1;
//                    cout << "to select agent action....-- \n";
//                    getchar();

		    new_agent_action(a);
		}
                else {
		  agentBusyState[a]=0;
                  agentMainWorkingPetal[a]=-1;
		  if (a==HUMAN_ID) 
		    cout << FRED("no work assigned to agent ") << a << FRED("--IDLE-- ")<<"\n";
		  else if (a==NAO_ID) 
		    cout << FGRN("no petal available for agent ") << a << FGRN("--IDLE-- ")<<"\n";
		  else
		    cout << FCYN("no petal available for agent ") << a << FCYN("--IDLE-- ")<<"\n";                    
                }
	    }
            if (agentBusyState[a]==1){ //implement a previously or just assigned petal
//	    if (a==NAO_ID){          
//                cout << "agent " << a << " is busy on action "<< agentMainWorkingAction[a]<< "    ";
//	    }
		if (agentMainWorkingAction[a]<0){
		  cout<<"looking for action....";
                  new_agent_action(a);
		}
                make_agent_action(a); //updates elapsed time
                if (checkActionCompletion(a)==1){
                    if (petal_completed(a)==1){ //the completion of action may result to the completion of petal
                        agentBusyState[a]=0;
                        agentMainWorkingPetal[a]=-1;
                    }
                    else{
//                    cout << "--agent[" << a<<"].busy:"<<agentBusyState[a]<< "\n";
                    agentMainWorkingAction[a]=-1;
                    new_agent_action(a);
//                    cout << "agent[" << a<<"].busy:"<<agentBusyState[a]<< "\n";
                    }
                }
	    }
	 }

      }
//      sendDaisyState();
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 1;
}

//prepei na einai entaxei.
//Yparxoyn themata sto debugging giai kanoyme polles fores intervene kai interupt, alla fainetai okay.


/*--------------------------------------------------------------------*/
//
//This implements the interuption of the execution of a petal, to be completed by any agent.
//
//In short, following the current scenario, a new petal is inserted into the plan with high priority
//The execution of the current petal is interupted and the agent switches to the new task
//The urgency of the first task is adjusted to specify how soon it will be executed 

int DP_processor::interupt(int pid, int agid){

  int ci;
  float rem_perc=0.0;
  if (petal[pid].assigned!=agid){
    cout <<"Error in DP_processor::interupt(). Different agent ids!"<<endl;
    exit(0);
  }
  
//  petal[pid].action[1].interupted=1;
  if (petal[pid].action[agentMainWorkingAction[agid]].interupted==1){
    return 1;
  }
    
//  cout << "\n\n\n\n now interupting petal:"<<pid<<"\n\n\n\n";
//  if (petal[pid].i_type==PT_Interupt)  { //the petal is interuptable and can be continued by anyone
    for (int i=0; i<petal[pid].length; i++){ //find the action that is currently worked out by the agent 
      if (petal[pid].action[i].done>=0) // some agent has already complete it
	continue;

      ci=i;//this is the first action of the petal that is not done yet

//      cout<< "action:"<<pid<<"."<<ci<<".assigned="<<petal[pid].action[ci].assigned<<endl;
      float t1=getExpectedRemainingTimeForAction(agid, pid, ci);      
      float basict=petal[pid].action[ci].exp_deftime[agid]; // this is the total time of the action

//      cout<<"basict:"<<basict<<" rem:"<<t1<<endl;

      if (t1>0){
//       cout<<"basict:"<<basict<<" rem:"<<t1<<" rem_compl_percent: "<< (t1)/basict <<endl;
       rem_perc=t1/basict;
      }

      break; //exit the for-loop
    }
    make_action_completion(agid); //complete the current action to let the agent continue with the post-actions of the petal 
    adjustExpectedTimes(pid, ci, rem_perc); //adjust the percentage of the action that remains to be completed by any agent in the future
    petal[pid].action[ci].interupted=1;

    petal[pid].interupted=1;
    //Note: after the current action, post-actions will be executed to enable the agent move to another task
    //Note: that since the petal is marked as intrupted, at the completion of the petal, its urgency is updated by UrgencyUPDATE   
//  }
  
    return 1;
}



/*--------------------------------------------------------------------*/
//
// This shifts the actions of a petal on step later in time. It starts from action pos and shifts all the actions following pos. 
//

int DP_processor::shift_actions(int pid, int pos){

  if (petal[pid].length+1<MAX_PETAL_LENGTH){
    for (int i=petal[pid].length-1; i>=pos; i--){
      int pr_pos=i;
      int n_pos=i+1;
    
      petal[pid].action[n_pos].name=petal[pid].action[pr_pos].name;
      petal[pid].action[n_pos].obj=petal[pid].action[pr_pos].obj;
      petal[pid].action[n_pos].type=petal[pid].action[pr_pos].type;
      for (int ag=0; ag<MAX_AGENTS; ag++){
	petal[pid].action[n_pos].expected_ftime[ag].s=petal[pid].action[pr_pos].expected_ftime[ag].s;
	petal[pid].action[n_pos].expected_ftime[ag].m1=petal[pid].action[pr_pos].expected_ftime[ag].m1;
	petal[pid].action[n_pos].expected_ftime[ag].m2=petal[pid].action[pr_pos].expected_ftime[ag].m2;
	petal[pid].action[n_pos].expected_ftime[ag].e=petal[pid].action[pr_pos].expected_ftime[ag].e;
	petal[pid].action[n_pos].exp_deftime[ag]=petal[pid].action[pr_pos].exp_deftime[ag];
	
	petal[pid].action[n_pos].robust[ag] =petal[pid].action[pr_pos].robust[ag]; 
      }
      petal[pid].action[n_pos].assigned=petal[pid].action[pr_pos].assigned;
      petal[pid].action[n_pos].done=petal[pid].action[pr_pos].done;
      petal[pid].action[n_pos].blocked=petal[pid].action[pr_pos].blocked;
      petal[pid].action[n_pos].interupted=petal[pid].action[pr_pos].interupted;
    }
    petal[pid].length++;
  }
  else{
    cout << "DP_processor::shift_actions(), not enough space for petal to increase. ERROR!"<<endl;
    exit(0);
  }
  
    return 1;  
}


/*--------------------------------------------------------------------*/
//
// This shifts the actions of a petal on step later in time. It starts from action pos and shifts all the actions following pos. 
//

int DP_processor::place_action(int pid, int pos, int act_id){

     FuzzyTimeTBX ftbx(1);

  
  if (pos<petal[pid].length){
    if (act_id==0){ //this is the action move to JACO. //Currently this is mostly hardwired 
           
      petal[pid].action[pos].name="Move2FruitShelf";
      petal[pid].action[pos].obj="";
      petal[pid].action[pos].type=A_MainImCompl;
      
      //implementation times for each agent
	petal[pid].action[pos].expected_ftime[0].s=3;
	petal[pid].action[pos].expected_ftime[0].m1=4;
	petal[pid].action[pos].expected_ftime[0].m2=7;
	petal[pid].action[pos].expected_ftime[0].e=8;
	petal[pid].action[pos].robust[0] =9; 
	
	petal[pid].action[pos].expected_ftime[1].s=38;
	petal[pid].action[pos].expected_ftime[1].m1=42;
	petal[pid].action[pos].expected_ftime[1].m2=57;
	petal[pid].action[pos].expected_ftime[1].e=63;
	petal[pid].action[pos].robust[1] =7; 
	
	petal[pid].action[pos].expected_ftime[2].s=111;
	petal[pid].action[pos].expected_ftime[2].m1=111;
	petal[pid].action[pos].expected_ftime[2].m2=111;
	petal[pid].action[pos].expected_ftime[2].e=111;
	petal[pid].action[pos].robust[2] =1; 
	
	petal[pid].action[pos].exp_deftime[0]=ftbx.defuz(petal[pid].action[pos].expected_ftime[0]);
	petal[pid].action[pos].exp_deftime[1]=ftbx.defuz(petal[pid].action[pos].expected_ftime[1]);
	petal[pid].action[pos].exp_deftime[2]=ftbx.defuz(petal[pid].action[pos].expected_ftime[2]);
	
      petal[pid].action[pos].assigned=-1;
      petal[pid].action[pos].done=-1;
      petal[pid].action[pos].blocked=0;
      petal[pid].action[pos].interupted=0;
    }
    else if (act_id==1){ //this is the action move to JACO. //Currently this is mostly hardwired 
               
      petal[pid].action[pos].name="Wait_Filling";
      petal[pid].action[pos].obj="";
      petal[pid].action[pos].type=A_MainImCompl;
      
      //implementation times for each agent
	petal[pid].action[pos].expected_ftime[0].s=10;
	petal[pid].action[pos].expected_ftime[0].m1=14;
	petal[pid].action[pos].expected_ftime[0].m2=93;
	petal[pid].action[pos].expected_ftime[0].e=100;
	petal[pid].action[pos].robust[0] =9; 
	
	petal[pid].action[pos].expected_ftime[1].s=10;
	petal[pid].action[pos].expected_ftime[1].m1=14;
	petal[pid].action[pos].expected_ftime[1].m2=93;
	petal[pid].action[pos].expected_ftime[1].e=100;
	petal[pid].action[pos].robust[1] =9; 
	
	petal[pid].action[pos].expected_ftime[2].s=10;
	petal[pid].action[pos].expected_ftime[2].m1=14;
	petal[pid].action[pos].expected_ftime[2].m2=93;
	petal[pid].action[pos].expected_ftime[2].e=100;
	petal[pid].action[pos].robust[2] =9; 

	petal[pid].action[pos].exp_deftime[0]=ftbx.defuz(petal[pid].action[pos].expected_ftime[0]);
	petal[pid].action[pos].exp_deftime[1]=ftbx.defuz(petal[pid].action[pos].expected_ftime[1]);
	petal[pid].action[pos].exp_deftime[2]=ftbx.defuz(petal[pid].action[pos].expected_ftime[2]);
	
      petal[pid].action[pos].assigned=-1;
      petal[pid].action[pos].done=-1;
      petal[pid].action[pos].blocked=0;
      petal[pid].action[pos].interupted=0;
    }
    else if (act_id==2){ //this is the action move to JACO. //Currently this is mostly hardwired 
               
      petal[pid].action[pos].name="Move2Table";
      petal[pid].action[pos].obj="";
      petal[pid].action[pos].type=A_MainImCompl;
      
      //implementation times for each agent
	petal[pid].action[pos].expected_ftime[0].s=3;
	petal[pid].action[pos].expected_ftime[0].m1=4;
	petal[pid].action[pos].expected_ftime[0].m2=6;
	petal[pid].action[pos].expected_ftime[0].e=7;
	petal[pid].action[pos].robust[0] =9; 
	
	petal[pid].action[pos].expected_ftime[1].s=30;
	petal[pid].action[pos].expected_ftime[1].m1=33;
	petal[pid].action[pos].expected_ftime[1].m2=48;
	petal[pid].action[pos].expected_ftime[1].e=53;
	petal[pid].action[pos].robust[1] =7; 
	
	petal[pid].action[pos].expected_ftime[2].s=111;
	petal[pid].action[pos].expected_ftime[2].m1=111;
	petal[pid].action[pos].expected_ftime[2].m2=111;
	petal[pid].action[pos].expected_ftime[2].e=111;
	petal[pid].action[pos].robust[2] =1; 
	
	petal[pid].action[pos].exp_deftime[0]=ftbx.defuz(petal[pid].action[pos].expected_ftime[0]);
	petal[pid].action[pos].exp_deftime[1]=ftbx.defuz(petal[pid].action[pos].expected_ftime[1]);
	petal[pid].action[pos].exp_deftime[2]=ftbx.defuz(petal[pid].action[pos].expected_ftime[2]);

      petal[pid].action[pos].assigned=-1;
      petal[pid].action[pos].done=-1;
      petal[pid].action[pos].blocked=0;
      petal[pid].action[pos].interupted=0;
    }
    else{
      cout << "DP_processor::place_action(), UN-KNOWN action to add. Possible ERROR!!!"<<endl;
    }
    
  }
  else{
    cout << "DP_processor::place_action(), incorrect petal position. ERROR!"<<endl;
    exit(0);
  }
  
    return 1;  
}

/*--------------------------------------------------------------------*/
//
// This implements the  the intervention of new actions in the petal executed by the agent and imediate stop/completion of the current action 
// executed by the agent in order to continue with the new ones.
//

int DP_processor::intervene(int pid, int agid){

  float rem_perc=0.0;
  if (petal[pid].assigned!=agid){
    cout <<"Error in DP_processor::interupt(). Different agent ids!"<<endl;
    exit(0);
  }

  int curpos=agentMainWorkingAction[agid];


  //add Move2fruitshelf
  shift_actions(pid, curpos+1);// move right all the actions following the current agent action, in order to create free space
  place_action(pid, curpos+1,0); //insert in petal "pid", location "curpos", the action with id 0

  //add WaitFilling
  shift_actions(pid, curpos+2);// move right all the actions following the current agent action, in order to create free space
  place_action(pid, curpos+2,1); //insert in petal "pid", location "curpos", the action with id 1

  //add Move2Table
  shift_actions(pid, curpos+3);// move right all the actions following the current agent action, in order to create free space
  place_action(pid, curpos+3,2); //insert in petal "pid", location "curpos", the action with id 2
  
  for (int i=0; i<petalNumber; i++)
  {
    if (i==pid) 
      continue;
    if ((petal[i].completed>=0) && (petal[i].completed!=UnknownAgent))
      continue;
    if (petal[i].completed==UnknownAgent){
      petal[i].completed=-1;
      int arc_from=-1; //arriving constraint: from action (i.e. arriving to petal pid)
      int arc_to=curpos+3; //arriving constraint: to action

      int depc_from=curpos+1; //departing constraint: from action (i.e. departing from petal pid)
      int depc_to=-1; //departing constraint: to action

      for (int k=0; k<petal[i].length; k++){ //for all actions in the petal 
	//specify details for the departing constraint
	if (k>0){
	  if ( (depc_to==-1) && (petal[i].action[k-1].type==A_Pre) && (petal[i].action[k].type!=A_Pre)){
	    depc_to=k;
	  }
	}
	//specify details for the arriving constraint
	if ((petal[i].action[k].type==A_Post) && (arc_from==-1)){
	  arc_from=k-1;
	}
      }
      
      //prosthikh arriving Constraint      
      if ((arc_from>=0) && (arc_to>=0)){
	if (constrNum+1>=MAX_CONSTRAINTS){
	  cout<<"DP_processor::intervene() Not enough space for new Constriants. Error!";
	  exit(0);
	}
	else{
	  constr[constrNum].fpetal=i;
	  constr[constrNum].fvertex=arc_from;
	  constr[constrNum].tpetal=pid;
	  constr[constrNum].tvertex=arc_to;
	  constr[constrNum].time.s=0; constr[constrNum].time.m1=0; constr[constrNum].time.m2=0; constr[constrNum].time.e=0;
	  constr[constrNum].stateOn=1;
	  constr[constrNum].prevents=-1;
	  linkConstraint2Petals(constrNum);
	  constrNum++;
	}
      }
      
      //prosthikh departing Constraint
      if ((depc_from>=0) && (depc_to>=0)){
	if (constrNum+1>=MAX_CONSTRAINTS){
	  cout<<"DP_processor::intervene() Not enough space for new Constriants. Error!";
	  exit(0);
	}
	else{
	  constr[constrNum].fpetal=pid;
	  constr[constrNum].fvertex=depc_from;
	  constr[constrNum].tpetal=i;
	  constr[constrNum].tvertex=depc_to;
	  constr[constrNum].time.s=0; constr[constrNum].time.m1=0; constr[constrNum].time.m2=0; constr[constrNum].time.e=0;
	  constr[constrNum].stateOn=1;
	  constr[constrNum].prevents=-1;
	  linkConstraint2Petals(constrNum);
	  constrNum++;
	}
      }
      
    }
    //the petal is assigned to human but not compeleted yet
    if ((petal[i].assigned==HUMAN_ID) && (petal[i].completed=-1)) {
	if (constrNum+1>=MAX_CONSTRAINTS){
	  cout<<"DP_processor::intervene() Not enough space for new Constriants. Error!";
	  exit(0);
	}
	else{
	  constr[constrNum].fpetal=i;
	  constr[constrNum].fvertex=1; //this is hard coded but it should not
	  constr[constrNum].tpetal=pid;
	  constr[constrNum].tvertex=curpos+3;
	  constr[constrNum].time.s=0; constr[constrNum].time.m1=0; constr[constrNum].time.m2=0; constr[constrNum].time.e=0;
	  constr[constrNum].stateOn=1;
	  constr[constrNum].prevents=-1;
	  linkConstraint2Petals(constrNum);
	  constrNum++;
	}
      
    }
  }
  
  make_action_completion(agid); //complete the current action to let the agent continue with the following actions of the petal 
  setCurrentActionComplete(agid);
//  petal[pid].action[agentMainWorkingAction[agid]].interupted=1;
  
  
//  showPetals();
/*  cout <<"\n\n Sucess!\n\n";
  exit(0);
  */
  
  return 1;
}



/*--------------------------------------------------------------------*/
//It returns the edgeid leading to the next action
int DP_processor::new_agent_action(int aid){
    int p;
//    bool test=true;
    if ((aid>=0) && (aid<MAX_AGENTS)){
      
      if (agentUrgentWorkingPetal[aid]<0)
	p=agentMainWorkingPetal[aid];
      else
	p=agentUrgentWorkingPetal[aid];      

      for (int j=0; j<petal[p].length; j++){ //find the first available action
            if (petal[p].action[j].assigned<0){ //action is not already assigned to any agent
                agentMainWorkingAction[aid]=j; //don't use: petal[p].edge[j];
                petal[p].action[j].assigned=aid;
		if (aid==HUMAN_ID)
		  cout << FRED("human assigned action (p:")<< p<<FRED("-a:") <<j <<FRED(")\n");
		else if (aid==NAO_ID)
		  cout << FGRN("robot assigned action (p:")<< p<<FGRN("-a:") <<j <<FGRN(")\n");
		else
		  cout << FCYN("robot assigned action (p:")<< p<<FCYN("-a:") <<j <<FCYN(")\n");
		petal[p].action[j].start_time=time(0);
		doneCurrentAction[aid]=0; 
		
		//Edw thelei elegxo an einai ayto to action constrained, opote the leme "einai constrained" -> "nothing send"
		if (j>0){
//		  int cnum=getActiveConstrOnVertex(p, petal[p].action[j].id, aid);
		  int cnum=getActiveConstrOnVertex(p, j, aid);
//		  cout << "constraints on ["<<p<<"].["<<j<<"]:"<<cnum<<endl;
		  if ((cnum>0) && (!petal[p].interupted)){
		    if (aid==HUMAN_ID)
		      cout <<FRED("agent ")<<aid<<FRED("is constrained by:")<<cnum<<FRED(" constraints. To re-assign....")<<endl; 
		    else if (aid==NAO_ID)
		      cout <<FGRN("agent ")<<aid<<FGRN("is constrained by:")<<cnum<<FGRN(" constraints. To re-assign....")<<endl; 
		    else
		      cout <<FCYN("agent ")<<aid<<FCYN("is constrained by:")<<cnum<<FCYN(" constraints. To re-assign....")<<endl; 

		    checkHumanForcedConstraintRelease(aid, p, j); //petal[p].action[j].id
		    
		    //this is only for NAO, if all tasks are worked out, then NAO may go on
		    if (aid==NAO_ID){
//		      cout <<"check special release....";
		      int all_in_process=1;
		      for (int pp=0; pp<petalNumber; pp++){
			if ((petal[pp].assigned<0)){
			  all_in_process=0;
			}
		      }
		      if (all_in_process==1){//release the constraints coming from the human petal
			releaseConstraintsFromHumanPetal(p,j);
			cout <<" constraints from petal "<<p<<" are released\n";
		      }
		    }
		  }
		  cnum=getActiveConstrOnVertex(p, j,aid);
		  if ((cnum>0) && (!petal[p].interupted)){
		    if (aid==HUMAN_ID)
		      cout <<FRED("agent ")<<aid<<FRED("is constrained by:")<<cnum<<FRED(" constraints")<<endl; 
		    else if (aid==NAO_ID)
		      cout <<FGRN("agent ")<<aid<<FGRN("is constrained by:")<<cnum<<FGRN(" constraints")<<endl; 
		    else
		      cout <<FCYN("agent ")<<aid<<FCYN("is constrained by:")<<cnum<<FCYN(" constraints")<<endl; 
		    agentMainWorkingAction[aid]=-1;
		    //agentBusyState[aid]=0; this regards petal assignement not action assignement
		    petal[p].action[j].assigned=-1;
		    petal[p].action[j].blocked=1;
		    return -1;
		  }
		  else{
		    petal[p].action[j].blocked=0;
		    //the action is not constrained and it is send for execution
		    if (aid==HUMAN_ID)
			cout<<FRED("sending for executionM")<<endl;
		    else if (aid==NAO_ID)
			cout<<FGRN("sending for executionM")<<endl;
		    else
			cout<<FCYN("sending for executionM")<<endl;
		    //sendMessage(aid, petal[p].action[j].name.c_str());
		    sendAction2ROS(aid);
		  }
		}
		else{ //it is the first action in petal, which is not constraint
		  if (aid==HUMAN_ID){
		    cout<<FRED("sending for executionK")<<endl;  
		  }
		  else if (aid==NAO_ID){
		    fclose(stderr);
		    cout<<FGRN("sending for executionK")<<endl;  
		    //system("vlc NaoGetBreakfast.mp3 &");
//		    sleep(4);
		    //system("ps aux | grep -ie vlc | awk '{print \"kill -9 \" $2}' | sh -x");
		    system("ps aux | grep -ie vlc | awk '{print \"kill -9 \" $2}' | sh -x");
		    sleep(1);
		  }
		  else{
		    fclose(stderr);
		    cout<<FCYN("sending for executionK")<<endl;  
		    //system("vlc JacoAddIgredients.mp3 &");		    
//		    sleep(3);
		    system("ps aux | grep -ie vlc | awk '{print \"kill -9 \" $2}' | sh -x");
		  }
		  //sendMessage(aid, petal[p].action[j].name.c_str());
		  sendAction2ROS(aid);
		  
		}
		
#ifdef LocalTesting
		if (aid==HUMAN_ID)
		  cout <<FRED("name:")<<petal[p].action[j].name<<"\n";
		else if (aid==NAO_ID)
		  cout <<FGRN("name:")<<petal[p].action[j].name<<"\n";
		else
		  cout <<FCYN("name:")<<petal[p].action[j].name<<"\n";
#else		
		/*
		struct PlannerActionEntry robotAction;
		robotAction.actionName=petal[p].al_actionName[j];
		robotAction.primaryObjectName=petal[p].al_primaryObjectName[j];
		robotAction.secondaryObjectName=petal[p].al_secondaryObjectName[j];
		robotAction.handName=petal[p].al_handName[j];
		robotAction.robotLocation=petal[p].al_robotLocation[j];
		robotAction.objectLocation=petal[p].al_objectLocation[j];
		
		cout << "sending to robot:" <<robotAction.actionName <<endl;
		
		if (!planner2Ice)
		  cout<<"new_agent_action()   cannot access ICE"<<endl;
		else{
		  if (aid==NAO_ID)
		    planner2Ice->setPlannedRobotActionData(robotAction);
//		  if (aid==HUMAN_ID)
//		    planner->setPlannedHumanActionData(robotAction);
		}
		*/
#endif		  
		
                petal[p].action[j].start_time=time(0);
                break;
            }
        }
    }
    else{
        cout << "DP_processor::new_agent_action() wrong Agent Id \n";
        exit(1);
    }
}






/*--------------------------------------------------------------------*/
int DP_processor::make_agent_action(int aid){

  int p;
  int acnum;
      if (agentUrgentWorkingPetal[aid]<0){
	p=agentMainWorkingPetal[aid];
	acnum=agentMainWorkingAction[aid];
      }
      else{
	p=agentUrgentWorkingPetal[aid];      
	acnum=agentUrgentWorkingAction[aid];
      }

    if ((p>=0) && (acnum>=0)){
        petal[p].action[acnum].implementation_time=time(0)-petal[p].action[acnum].start_time; //current_time-strat_time
    }

}


/*--------------------------------------------------------------------*/
//It marks the action currently worked by agent aid as completed
int DP_processor::make_action_completion(int aid){
    int p;
    int edgenum;
    
    if (agentUrgentWorkingPetal[aid]<0){
	p=agentMainWorkingPetal[aid];
	edgenum=agentMainWorkingAction[aid];
      }
    else{
	p=agentUrgentWorkingPetal[aid];      
	edgenum=agentUrgentWorkingAction[aid];
      }
    
    if ((p>=0) && (edgenum>=0)){
        // set time of action completion
        petal[p].action[edgenum].implementation_time=time(0)-petal[p].action[edgenum].start_time; //current_time-strat_time
        petal[p].action[edgenum].done=aid; // set agent that completed the action

        //agentBusyState[aid]=0;
	
	if (aid==HUMAN_ID) 
	  cout << FRED("action[") << p<<FRED("][") << edgenum << FRED("]") << FRED("completed ")<<"\n";
	else if (aid==NAO_ID)          
	  cout << FGRN("action[") << p<<FGRN("][") << edgenum << FGRN("]") << FGRN("completed ")<<"\n";
	else          
	  cout << FCYN("action[") << p<<FCYN("][") << edgenum << FCYN("]") << FCYN("completed ")<<"\n";
      
    }
    else{
        cout <<"DP_processor::make_action_completion(), Error! Agent " << aid << " is not busy\n";
        exit(0);
    }
    return 1;
}

/*--------------------------------------------------------------------*/
void DP_processor::setCurrentActionComplete(int aid){
  doneCurrentAction[aid]=1; 
}

/*--------------------------------------------------------------------*/
//get the time needed to complete the un-assigned actions in the petal (does not include the one that the agent curently works at
int DP_processor::timeOfUnAssignedActions(int ag_id, int pid){
  int t_estimate=0;
  
  if (pid>=0){ 
    for (int j=0; j<petal[pid].length; j++){
      if (petal[pid].action[j].assigned<0){
	t_estimate += (int)petal[pid].action[j].exp_deftime[ag_id];		    
      }
    }
  }
  return t_estimate;
}


/*--------------------------------------------------------------------*/
//It returns 1 if action is completed, 0 otherwise
int DP_processor::checkActionCompletion(int aid){
    
  int p,edgenum;
    
    if (agentUrgentWorkingPetal[aid]<0){
	p=agentMainWorkingPetal[aid];
	edgenum=agentMainWorkingAction[aid];
      }
    else{
	p=agentUrgentWorkingPetal[aid];      
	edgenum=agentUrgentWorkingAction[aid];
      }
  
  
   if ((p>=0) && (edgenum>=0)){
 //        cout << "controlling action--" << agentMainWorkingAction[aid] << " time:" << petal[p].implementation_time[edgunum] <<"\n";
//        int tt=petal[p].implementation_time[edgunum];
	
//		  std::cout << " isRobotDone:" << actionStatusListener->reportRobotActionStatus() << std::endl;

#ifdef LocalTesting	

     
//     kai not interupted

     if ((aid==JACO_ID) && (agentMainWorkingAction[NAO_ID]>=0)){
       	string s(petal[agentMainWorkingPetal[NAO_ID]].action[agentMainWorkingAction[NAO_ID]].name); //make a string with the name of the action
//	cout << s.c_str()<<endl;
       if ((agentMainWorkingPetal[JACO_ID]==5) &&(agentMainWorkingAction[JACO_ID]==1) && (!petal[agentMainWorkingPetal[JACO_ID]].interupted) &&
	   (agentMainWorkingPetal[NAO_ID]==0) &&((s.find("Move2FruitShelf")!=std::string::npos) || (s.find("Wait_Filling")!=std::string::npos))){	
	 int NAO_time2Complete=(int)getExpectedRemainingTimeForAction(NAO_ID, agentMainWorkingPetal[NAO_ID], agentMainWorkingAction[NAO_ID]);
	 int JACO_time2ClearHand= timeOfUnAssignedActions(JACO_ID,agentMainWorkingPetal[JACO_ID]);
	 //  cout <<"------ NAO2complete: "<<NAO_time2Complete<< "JACO2Clear: " << JACO_time2ClearHand<<endl;

       
	 if (NAO_time2Complete-13<JACO_time2ClearHand){
	   //cout <<"------ NAO2complete: "<<NAO_time2Complete<< "JACO2Clear: " << JACO_time2ClearHand<<endl;
	   //cout <<"interupting JACO..."<<endl;	
	   
	   interupt(agentMainWorkingPetal[JACO_ID], JACO_ID);
	   sendCancel2JACO(JACO_ID);
	 }
       }
     }

     
/*     
      int act_ok=-1;      
     if ((aid==JACO_ID) && (agentMainWorkingPetal[JACO_ID]==5) && (agentMainWorkingAction[JACO_ID]==1) && (!petal[agentMainWorkingPetal[JACO_ID]].interupted) ){
       if (agentMainWorkingAction[NAO_ID]>=0){
       	string s(petal[agentMainWorkingPetal[NAO_ID]].action[agentMainWorkingAction[NAO_ID]].name); //make a string with the name of the action
	cout << s.c_str()<<endl;
	if ((agentMainWorkingPetal[NAO_ID]==0) &&((s.find("Move2FruitShelf")!=std::string::npos) || (s.find("Wait_Filling")!=std::string::npos)))
	  act_ok=1;
       }
       
       if ((act_ok==1) || (agentMainWorkingAction[NAO_ID]<0)){      
	 int NAO_time2Complete=(int)getExpectedRemainingTimeForAction(NAO_ID, agentMainWorkingPetal[NAO_ID], agentMainWorkingAction[NAO_ID]);
	 int JACO_time2ClearHand= timeOfUnAssignedActions(JACO_ID,agentMainWorkingPetal[JACO_ID]);
	   cout <<"------ NAO2complete: "<<NAO_time2Complete<< "JACO2Clear: " << JACO_time2ClearHand<<endl;

       
	 if (NAO_time2Complete-13<JACO_time2ClearHand){
	   cout <<"------ NAO2complete: "<<NAO_time2Complete<< "JACO2Clear: " << JACO_time2ClearHand<<endl;
	   cout <<"interupting JACO..."<<endl;	
	   
	   interupt(agentMainWorkingPetal[JACO_ID], JACO_ID);
	   sendCancel2JACO(JACO_ID);
	 }
       }
     }
  */   
/* //this is for testing
	    if ((aid==JACO_ID)){
	      if (((agentMainWorkingPetal[JACO_ID]==5) &&(agentMainWorkingAction[JACO_ID]==1) && (petal[p].action[1].implementation_time>3) && (petal[p].interupted==0)){
		cout <<"interupting..."<<endl;
		interupt(p, aid);
	      }
	    }
*/






	    if ((aid==NAO_ID)){
//	      cout <<"*"<<agentMainWorkingAction[aid]<<"* ";
	      if ((aid==NAO_ID) && (p==0) &&(agentMainWorkingAction[aid]==2) && (petal[p].action[2].implementation_time==20)){
		petal[agentMainWorkingPetal[HUMAN_ID]].action[agentMainWorkingAction[HUMAN_ID]].exp_deftime[HUMAN_ID]+=120;
		cout <<"\n!!!!\n";
		cout <<FGRN("Due to an unpredicted event, the goal for agent ")<<NAO_ID<<FGRN("is changed!")<<endl;

		cout <<FGRN("inteRVining new action set...")<<endl;
		intervene(p, aid);
	      }
	    }
	    
	    
    
	  if (((aid==NAO_ID)&& (edgenum==0) && (petal[p].action[edgenum].implementation_time>56)) || 
	    ((aid==NAO_ID)&& (edgenum==1) && (petal[p].action[edgenum].implementation_time>2)) ||
	    ((aid==NAO_ID)&& (edgenum==2) && (petal[p].action[edgenum].implementation_time>44)) ||
	    ((aid==NAO_ID)&& (edgenum==3) && (petal[p].action[edgenum].implementation_time>3)) || 
	    ((aid==NAO_ID)&& (edgenum==4) && (petal[p].action[edgenum].implementation_time>3)) || 
	    ((aid==NAO_ID)&& (edgenum==5) && (petal[p].action[edgenum].implementation_time>20)) || 
	    ((aid==NAO_ID)&& (edgenum==6) && (petal[p].action[edgenum].implementation_time>3)) || 
	    ((aid==NAO_ID)&& (edgenum==7) && (petal[p].action[edgenum].implementation_time>20)) || 
	    ((aid==NAO_ID)&& (edgenum==8) && (petal[p].action[edgenum].implementation_time>3)) || 
	    ((aid==NAO_ID)&& (doneCurrentAction[aid])) || 
	    ((aid==JACO_ID) &&(p==5) && (edgenum==0) && (petal[p].action[edgenum].implementation_time>16)) ||    //this is for salad mixing
	    ((aid==JACO_ID) &&(p==5) && (edgenum==1) && (petal[p].action[edgenum].implementation_time>44)) ||
	    ((aid==JACO_ID) &&(p==5) && (edgenum==2) && (petal[p].action[edgenum].implementation_time>10)) ||
	    ((aid==JACO_ID) &&(p==5) && (edgenum==3) && (petal[p].action[edgenum].implementation_time>6))  ||
	    ((aid==JACO_ID) &&(p!=5) && (edgenum==0) && (petal[p].action[edgenum].implementation_time>14)) ||    //this is for fruit placing
	    ((aid==JACO_ID) &&(p!=5) && (edgenum==1) && (petal[p].action[edgenum].implementation_time>4)) ||
	    ((aid==JACO_ID) &&(p!=5) && (edgenum==2) && (petal[p].action[edgenum].implementation_time>8)) ||
	    ((aid==JACO_ID) &&(p!=5) && (edgenum==3) && (petal[p].action[edgenum].implementation_time>6))  ||
	    
	    ((aid==HUMAN_ID)&& (petal[p].action[edgenum].implementation_time>1) && (petal[p].action[edgenum].implementation_time+10>petal[p].action[edgenum].exp_deftime[HUMAN_ID])) )
	  {

/*
//Testing xwris to NAO
	   if ((doneCurrentAction[aid]) ||
	    ((aid==NAO_ID)&& (edgenum==0) && (petal[p].action[edgenum].implementation_time>46)) || 
	    ((aid==NAO_ID)&& (edgenum==1) && (petal[p].action[edgenum].implementation_time>4)) ||
	    ((aid==NAO_ID)&& (edgenum==2) && (petal[p].action[edgenum].implementation_time>44)) ||
	    ((aid==NAO_ID)&& (edgenum==3) && (petal[p].action[edgenum].implementation_time>3)) || 
	    ((aid==NAO_ID)&& (edgenum==4) && (petal[p].action[edgenum].implementation_time>3)) || 
	    ((aid==HUMAN_ID)&& (petal[p].action[edgenum].implementation_time>1) && (petal[p].action[edgenum].implementation_time>petal[p].action[edgenum].exp_deftime[HUMAN_ID])) )
	  {
*/
//OTAN paizei me COMMUNICATION me ta robot, tote einai energo to parakatw:
/*
	   if ((doneCurrentAction[aid]) ||
	    ((aid==HUMAN_ID)&& (petal[p].action[edgenum].implementation_time>1) && (petal[p].action[edgenum].implementation_time+3>petal[p].action[edgenum].exp_deftime[HUMAN_ID])) )	     
	  {
*/	    
	      petal[p].action[edgenum].done=aid; // set which agent has completed the action	      
	      petal[p].action[edgenum].implementation_time=time(0)-petal[p].action[edgenum].start_time; //current_time-strat_time
            
	      if (aid==HUMAN_ID) 
		cout << FRED("action[") << p<<FRED("][") << edgenum << FRED("]") << FRED("comPleted ")<<"\n";
	      else if (aid==NAO_ID)          
		cout << FGRN("action[") << p<<FGRN("][") << edgenum << FGRN("]") << FGRN("comPleted ")<<"\n";
	      else          
		cout << FCYN("action[") << p<<FCYN("][") << edgenum << FCYN("]") << FCYN("comPleted ")<<"\n";

	      if (petal[p].to_cNum>0){ //there are constraints departing from this petal
//		cout<<"departing constraints:" << petal[p].to_cNum << endl;
		for (int j=0; j<petal[p].to_cNum; j++){
//		  cout <<"j="<<j<<"  dep"<<constr[petal[p].pcToL[j]].fvertex<<"?"<<petal[p].action[edgenum].id<<endl;
		  if (constr[petal[p].pcToL[j]].fvertex ==  petal[p].action[edgenum].id){
		    constr[petal[p].pcToL[j]].stateOn=0;
		    constr[petal[p].pcToL[j]].prevents=-1;
		    cout<<FYEL("Deactivated constraint:(")<<p<<"."<<constr[petal[p].pcToL[j]].fvertex  <<"->"<< constr[petal[p].pcToL[j]].tpetal <<"." << constr[petal[p].pcToL[j]].tvertex <<")   ";
		  }
		}
	      }
	      return 1;
	  }
//	  Ayto einai on otan eimaste oloi mazi
	  /*ayto mpike embolima logo toy DEMO ginetai kai parapanw alla oxi gia ton human*/
          petal[p].action[edgenum].implementation_time=time(0)-petal[p].action[edgenum].start_time; //current_time-strat_time

#else	
/*
	  if (!actionStatusListener)
	    cout <<"checkRobotActionCompletion() cannot access ICE!"<<endl;
	  else{ 
	      cout << " isRobotDone:" << actionStatusListener->reportRobotActionStatus() << std::endl;
	      bool s= actionStatusListener->reportRobotActionStatus();
	    
	      if (s){
		if (aid==NAO_ID)          
		  cout << FGRN("action[") << p<<FGRN("][") << edgenum << FGRN("]") << FGRN("comPleted by robot ")<<"\n";
		else          
		  cout << FCYN("action[") << p<<FCYN("][") << edgenum << FCYN("]") << FCYN("comPleted by robot ")<<"\n";

		petal[p].action[edgenum].done=aid;
		petal[p].action[edgenum].assigned=aid;
		petal[p].implementation_time[edgenum]=time(0)-petal[p].action[edgenum].;
		//we may also ask time from ICEinterface 
		return 1;
	      }
	  }
	  */
#endif	
    }
    return 0;
}

/*--------------------------------------------------------------------*/
//It returns 1 if petal is fully implemented, 0 otherwise
int DP_processor::petal_completed(int aid){
  
    int p;
    int emergency_petal=0;
  
//    int p;
    if ((aid>=0) && (aid<MAX_AGENTS)){
      if (agentUrgentWorkingPetal[aid]<0){
	  p=agentMainWorkingPetal[aid];
	  emergency_petal=0; //The petal is not an emergency petal
      }
      else{
	  p=agentUrgentWorkingPetal[aid]; 
	  emergency_petal=1; //This is an emergency petal
      }

      for (int j=0; j<petal[p].length; j++){ //scan the whole petal
            if (petal[p].action[j].done<0){ //action is not already completed
                return 0;
            }
      }
        petal[p].completed=aid; //the id of the agent completed the task

//        cout << "petal[" <<p<<"] completed by agent ["<<aid<< "] in";
        for (int j=0; j<petal[p].length; j++){ //scan the whole petal
//            cout << petal[p].action[j].implementation_time << " + ";
        }
//        cout <<"moments\n";
//	cout <<"interupted:"<<petal[p].interupted<<endl; 
	
	if ((petal[p].interupted) && (petal[p].assigned>=0)){ // the petal completes because it has been interupted but it has to be re-executed
	    petal[p].assigned=-1; // not assigned
	    petal[p].completed=-1; // not completed
//	    petal[p].interupted=0;
	    petal[p].urgency=petal[p].urgency*UrgencyUPDATE; // =0.0; //the petal gets low priority
	    
	    for (int i=0; i<petal[p].length; i++){ //find the action that is currently worked out by the agent 
	      petal[p].action[i].done=-1; 
	      petal[p].action[i].assigned=-1; 
	      petal[p].action[i].blocked=0; 
	      petal[p].action[i].interupted=0; 
            
//	      cout <<petal[p].action[i].assigned << " ";
	    }
	}
	
//	printf("(i:%d a:%d c:%d u:%f)\n", (int)petal[p].interupted, (int)petal[p].assigned, (int)petal[p].completed, (float) petal[p].urgency);
//	cout <<"now"<<petal[p].urgency<< endl;
	
/*	
        if (emergency_petal){ // if the completed petal was "emergency" the agent is switched-back to the main petal
	  agentUrgentWorkingPetal[aid]=-1; //the emergency petal for the agent is deactivated
	  return 0; //this declares that the main petal is still, not complete
	}
*/	
    }
    else{
        cout << "DP_processor::make_agent_action() wrong Agent Id \n";
        exit(1);
    }
    return 1;
}

