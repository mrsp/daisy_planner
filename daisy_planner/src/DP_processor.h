

#ifndef _DPPROCESSOR_H
#define _DPPROCESSOR_H



#include "fuzzy_calc.h"
#include <vector>
#include <string>



#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <strings.h>
#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <timestorm_msg/Time.h>
#include <timestorm_msg/Fruit_id.h>
#include <timestorm_msg/Daisy.h>
#include <timestorm_msg/Robot.h>
#include <timestorm_msg/Action.h>
#include <timestorm_msg/Petal.h>
#include <timestorm_msg/Daisy_graph.h>

#include <std_msgs/String.h>


 
using namespace std;
 
typedef vector<string> vstring;


#define MaxPetals 10 //maximum number of petals per DaisyGraph
#define MAX_PETAL_LENGTH 15 //this regards the edges. The vertices nuber is edges+1
#define MAX_CONSTRAINTS 15 //Number of outgoing constraints from a petal to other vertices
#define MAX_AGENTS 3 // this is the total number of Agents used in the experiment, one human and one robot
#define HUMAN_ID 0
#define NAO_ID 1
#define JACO_ID 2


#define UnknownAgent 4


//--------------------------------------------
#define PT_nonInt  0 //The task/petal is not interuptable. The whole petal must be fully completed by the agent  
#define PT_Interupt  1//The task/petal can be interupted and continued by anyone. 
			// The petal conists of preparatory, main and relief actions. At the time of interuption, the %-completion of the main actions is encoded (e.g. 63%)
			// and all remaining releaf actions are executed to "clear"  the state of the agent. 
			// Immediatelly after that, the petal is set incomplete and unassigned, so that **another or the same** agent can further work on it.
			// Moreover, for the main actions, the remaing percentage until completion is set as the new implementation goal  (e.g. 100-63=37%).
#define PT_Intervene  2// The action of the petal can be immediatelly stopped and the petal continues by the same agent.
			// A new action can intervene at any location of the action sequence encoded by the petal. 
			// Thus, the current action of the agent can be changed. 
			// Notice that the agent_id can not change. It will be always the same agent performing the given sequence of actions (i.e. the petal).  
//--------------------------------------------



//--------------------------------------------
#define A_Pre 0   // this is a preparatory action that has to be implemented before the main action
#define A_MainIntAble 1   // this is a main action that can be interupted and continued later from the point it has stoped
#define A_MainImCompl 2   // this is a main action that can not be interupted (to be continued) but can only immediately completed to switch to another action or task.
#define A_Post 3  //  this ia a releaf action that has to be implemented after the main action to enable the agent go in the rest state.
//--------------------------------------------

struct commandlineParameters{
	int nargc=0;
	char** nargv;
};



struct PriorityConstraint
{
    int fvertex;
    int tvertex;
    int fpetal;
    int tpetal;
   struct FuzzyNumber time;//delay time
   int stateOn; //this is whether the constraint is generally an active
   int prevents; //this regards whether "the current moment" the constraint prevents an action to be executed.
//   int linkedP;
};



struct ActionInfo
{
    int id;
    int done;
    int assigned;
    int type;
    int blocked; //0 or 1, this defines whether action is blocked by a constrained
    int interupted; //0 or 1, this defines whether action is interupted or not

    int param; ////// not working at the moment

    struct FuzzyNumber expected_ftime[MAX_AGENTS];//the value read during problem/daisy-graph loading
    float exp_deftime[MAX_AGENTS]; //the non fuzzy expected time per agent
    int implementation_time;
    int robust[MAX_AGENTS];
   
   string name;
   string obj;
   
   time_t start_time;
};


//typedef ::std::vector< ::std::string> PlannedAction;


struct PetalInfo
{
   int id; //to be deleted
   struct FuzzyNumber petal_time[MAX_AGENTS];//total implementation time
   float petal_robustness[MAX_AGENTS]; //total robustness

   //   static std::vector<string> v;
//   vstring v;
   
   struct ActionInfo action[MAX_PETAL_LENGTH+1];
   /*
   vstring al_actionName;
   vstring al_primaryObjectName;
   vstring al_secondaryObjectName;
   vstring al_handName;
   vstring al_robotLocation;
   vstring al_objectLocation;
   */
   int length;
   int assigned;  //id of the assigned agent
   int completed; // if not completed:-1, if completed : the id of the agent that did the job
   int interupted; //0 if not interupted, 1 if interupted

   //constraints
   int pcToL[MAX_CONSTRAINTS]; //to-link of priority constraints
   int pcFromL[MAX_CONSTRAINTS]; //from-link of priority constraints
   int to_cNum;
   int from_cNum;
   float urgency;
   
   
   int inplan; //this is a flag determining whether the petal is considered for plan execution or not
   int i_type; //the petal is of type InteruptType_0 or InteruptType_1 or InteruptType_2
};




class DP_processor
{
public:
    DP_processor();
    ~DP_processor();
//    void set_fuzzy_time(igraph_t *g, int vn, int rid, struct FuzzyNumber r);
    void init();

//    int scanDaisy(igraph_t *cg);
    int showPetals();
    int show_n_Petals();
//    void backConstraints();
    int setInactiveConstraintToPetal(int pid, int tov);
//    void updateAllPetalInfo(igraph_t *cg);

    void updateAllPetalInfo();
    
    int toDoPetal(int aid);
    int simulateRun();
//    void updateExpectedTimes(igraph_t *cg);
    void n_updateExpectedTimes();
    
    void adjustExpectedTimes(int pid, int acid, float percent);
    int make_action_completion(int aid);
//    int make_human_task_completion(int aid);
    int addNewPetal(int petLen);
    int getAgentWorkingPetal(int agent_id);
    int getAgentWorkingAction(int agent_id);
    void setRemainingTimeForAction(int agentid, int pid, int actid, int remT);
    void setCurrentActionComplete(int aid);
    void setPetalsInPlan();

    int getPetalExpectedTime(int agid, double *implemented, double* expected);
    int getActionRemainingTime(int pid, int actid, int agid, float* expected);
    void  copyPetals();
    
    struct PetalInfo petal[MaxPetals];
    int petalNumber;
    struct PriorityConstraint constr[MAX_CONSTRAINTS]; //forward priority constraints
    int constrNum;

    
    
  ros::NodeHandle *n;
  ros::Publisher *NAO_pub;
  ros::Subscriber *NAO_sub;
  ros::Publisher *JACO_pub;
  ros::Subscriber *JACO_sub;
  ros::Publisher *HUMAN_pub;
  ros::Subscriber *HUMAN_sub;
  ros::Publisher *HUMAN_Perc_pub;
  ros::Subscriber *HUMAN_Perc_sub;

  ros::Publisher *MENU_pub;
  ros::Subscriber *MENU_sub;

  ros::Publisher *GRAPH_pub;

  
//  ros::Publisher DP_pub;
//  ros::Subscriber DP_sub;

    int intervene(int pid, int agid);
    
    
    void sendAction2ROS(int ag_id);
    void sendDaisyState();
    void sendCancel2JACO(int ag_id);

    
    int agentBusyState[MAX_AGENTS]; // it referes to the petal level, not action-level.  Binary values: 0 or 1
    int agentMainWorkingPetal[MAX_AGENTS]; // if no petal assigned: -1. This is the main task assigned to an agent
    int agentMainWorkingAction[MAX_AGENTS]; // eg. the 3rd edge of the petal, if no action:-1. These are the tasks on the main petal
    
    int agentUrgentWorkingPetal[MAX_AGENTS]; // if no petal assigned: -1. This is an emergency task assigned to the agent, which must interupt the implementation of the Main task. 
    int agentUrgentWorkingAction[MAX_AGENTS]; // eg. the 3rd edge of the petal, if no action:-1. These are the tasks on the emergency petal
    
    
/*    
    struct PetalInfo n_petal[MaxPetals];
    int n_petalNumber;
    struct PriorityConstraint n_constr[MAX_CONSTRAINTS]; //forward priority constraints
    int n_constrNum;
*/

private:
  //  int PetalID;
  //  int VertexNum;

    void initializePetal(int pid);
//    int get_graph_info(igraph_t *cg, int avariable);
//    int constraints_OK(igraph_t *cg, int vid, int dontcare);
//    int constraints_onv(igraph_t *cg, int vid, int dontcare);
//    void get_petal_info(igraph_t *cg,  int edge, int vid, struct PetalInfo *PI, int *pnum, int uid);
//    void steponPetal(igraph_t *cg,  int *edge, int uid);
//    int findPetal(igraph_t *cg, igraph_vector_t *PHVoutedges, struct PetalInfo *AvPetals, int *ap_num, int user);

//    int findTask(igraph_t *cg, int curpos, int rid);
//    void set_randtimes(igraph_t *cg);
//    void new_action(igraph_t *cg, int rid, int *taskedge, igraph_vector_t *PHVoutedges, int *petid, struct PetalInfo *AvPetals, int *ap_num);
//    int DaisyPlanStep(igraph_t *cg);
//    int makeplan(igraph_t *cg, int **planlist);
    int make_agent_action(int aid);

    int new_agent_action(int aid);
    int checkActionCompletion(int aid);
    int petal_completed(int aid);

//    int setTransitionEdge( igraph_t *cg, igraph_vector_t *outedges, int petn, int curpos);
    int getActiveConstrOnPetal(int pid);
    int getActiveConstrOnVertex(int pid, int vid, int aid);

    int getActiveConstrFromPetal(int pid);
    void resetAllAgentTime(int pid);
    void resetAllAgentFeatures(int pid);
    void resetAgentTime(int pid, int aid);
    void resetAgentFeatures(int pid, int aid);
    int getConstrTimeOnPetal(int my_aid, int pid);
    void showExpectedTimes();
    int timeOfUnAssignedActions(int ag_id, int pid);
//    int checkHumanTaskCompletion(int aid);
    
    float getExpectedRemainingTimeForAction(int agentid, int pid, int actid);
    int interupt(int pid, int agid);

    int shift_actions(int pid, int pos);
    int place_action(int pid, int pos, int act_id);
    
    void linkConstraint2Petals(int constrNum);

    
    void releaseConstraintsFromUnassignedgedPetals(int pid, int verid);
    void releaseConstraintsFromHumanPetal(int pid, int verid);

    void sendMessage(int ag_id, const char* mes);
    void sendMenu();

    
    float getAgentConstrTimeOnPetal(int pid, int verid, int other_aid);
    void checkHumanForcedConstraintRelease(int aid, int pid, int verid);
    int getActionBeforeVertex(int pid, int verid);
    int setTCPIPConnection_0(const char* hostname, int portNo, int *listenFd, struct sockaddr_in *svrAdd, struct hostent **server);
    int sendOverTCPIP(const char* hostname, int portNo, int *listenFd, struct sockaddr_in *svrAdd, struct hostent **server, char* bb);

    int doneCurrentAction[MAX_AGENTS];//total implementation time

    void  icePlanCommunication();

//    void updatePetalInfo(igraph_t *cg, int pid, int aid);
    void updatePetalInfo(int pid, int aid);

    void readDaisy();


};

#endif
