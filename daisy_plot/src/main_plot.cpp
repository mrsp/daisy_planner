

/* 

Ta pleonekthmata ths methodoys se sxesh me alles texnikes
einai h dynatothta ektimishs ths kalyterhs xronikhs stigmhs gia na ginei kapoio action se sxesh me ton orismo constraints apo toys alloys.
Eykolia ypologismwn me fuzzy calculus.
H dynatothta myxhs toy xronoy me alla kritiria beltisopoihshs (oi alloi paizoyn mono me time constraints).
H dynatothta ypologismoy eleyteroy xronoy (free time).

*/

/* -*- mode: C -*-  */
/* 
   IGraph library.
   Copyright (C) 2007-2012  Gabor Csardi <csardi.gabor@gmail.com>
   334 Harvard street, Cambridge, MA 02139 USA
   
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc.,  51 Franklin Street, Fifth Floor, Boston, MA 
   02110-1301 USA

*/

//#include <igraph.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

#include <stdio.h>
#include <pthread.h>
 
#define NUM_THREADS 3

//#define NUM_PORTS 2 
//#define PORT0 2010 
//#define PORT1 2011 


#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>
#include <fstream>
#include <strings.h>
#include <string>
#include <sstream>

//static int connFd;

//#include "DP.h"
#include "CoutColors.h"

#include <ros/ros.h>
#include <ros/package.h>

//#include <timestorm_ps/Time.h>
#include <timestorm_msg/Daisy_graph.h>
#include <std_msgs/String.h>
#include <sstream>

#include <errno.h> 
#include <unistd.h>   //close 
#include <netinet/in.h> 
#include <sys/time.h> //FD_SET, FD_ISSET, FD_ZERO macros 
#include <boost/concept_check.hpp>


//commandlineParameters cp;
int DPinitialized=0;
double humanArousal=1.0;
double arousalToTimeEffect=1.25;

//1.2 puts 2 fruits

double human2target=0.0;
double robot2target=0.0;


using namespace std;


#define _FullReDraw



#include <vector>
#include <cmath>
#include <stdlib.h>
#include <boost/tuple/tuple.hpp>

#include "../gnuplot-iostream/gnuplot-iostream.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MaxPetals 10
#define MAX_PETAL_LENGTH 15

struct ActionData
{
    int id;
    int done;
    int assigned;
    int blocked; //0 or 1, this defines whether action is blocked by a constrained
    int interupted; //0 or 1, this defines whether action is interupted or not
    int param;
    string name;
    string obj;
};



struct PetalData
{
   int id; //to be deleted

   int assigned;  //id of the assigned agent
   int completed; // if not completed:-1, if completed : the id of the agent that did the job
   int interupted; //0 if not interupted, 1 if interupted
   int inplan;

   struct ActionData action[MAX_PETAL_LENGTH+1];
   int length;
};


struct PetalData petal[MaxPetals];
int petalNumber=-1;

int locked=1;

  Gnuplot gp;



  
  
  
//---------------------------------------------------------------
// It assumes as input to pairs of coordinates, the (x0,x1) and (y0,y1)  
void gnuLine2(vector<double> x,   vector<double> y,  int astyle){

//  cout <<"// ("<<x[0]<<","<<y[0]<<") ("<<x[1]<<","<<y[1]<<")"<<endl;

  std::vector<boost::tuple<double, double, double, double> > data;

  data.clear();
  data.push_back(boost::make_tuple(
             x[0],
             y[0],	     
            x[1]-x[0],
            y[1]-y[0]
        ) );
  
  gp << "plot "<< gp.file1d(data)<<" using 1:2:3:4 with vectors arrowstyle "<<astyle<<" notitle  \n ";   

//    gp.flush();

}
  

//--------------------------------------------------
void drawPetals(){

    double v=M_PI/180.0; 

//    cout <<"p: "<< petalNumber;

    std::vector<boost::tuple<double, double, double, double> > pts_A;

#ifdef _FullReDraw
//     gp<< "clear\n";
        gp<< "unset multiplot\n";
	sleep(0.3);
       gp<< "set multiplot\n";
#endif
     

     
    int pnum=petalNumber;
	int circle_seg=360/(pnum);
	double tx, ty;
	double mintx=10000;
	double maxtx=-10000;
	double minty=10000;
	double maxty=-10000;
	
	
	for (int j=0; j<pnum; j++){
//	for (int j=0; j<1; j++){
//          cout << petal[j].interupted<<"  "<<endl;
	  
  
	  if (petal[j].inplan==0){
		  //cout <<"petal"<<j<<" is out of plan*********"<<endl;
		  continue;
	  }
		
	  //if (j==5)
	  //  cout << petal[j].interupted<<"  "<<endl;
	
	float si,co;
	int angle;

	if (j==0){
		angle=j*circle_seg;
		si=sin((angle)*v);//the sin() direction of the petal to be drawn 
		co=cos((angle)*v);//the cos() direction of the petal to be drawn
	}
	else if (j==3){
		angle=j*circle_seg;
		si=sin((angle)*v);//the sin() direction of the petal to be drawn 
		co=cos((angle)*v);//the cos() direction of the petal to be drawn
	}
	else{
		angle=j*circle_seg;
		si=sin((angle)*v);//the sin() direction of the petal to be drawn 
		co=cos((angle)*v);//the cos() direction of the petal to be drawn
	}

        int len=petal[j].length;
	
		vector<double> x,y;  
		x.clear();
		y.clear();
		y.push_back(0.0);
		x.push_back(0.0);

//		for (int i=0; i<3; i++){
		for (int i=0; i<len; i++){
		//  int angle=j*circle_seg;
			//generate coordinates of edges linked on a petal-like drawing	
		  if (i<len-1){
		      if ((angle>=0) & (angle<46)){
			x.push_back(0.3+co*(2*i+1)-sqrt(i*i*i)*0.7);
			y.push_back(0.5+si*(1*i+1)-sqrt(i*i*i)*0.2);
		      }
		      else if ((angle>45) & (angle<91)){
			x.push_back(0.9+co*(1*i+1)-sqrt(i*i*i)*0.3); //0.6
			y.push_back(0.7+si*(2*i+1)-sqrt(i*i*i)*0.1);
		      }
		      else if ((angle>90) & (angle<136)){
			x.push_back(-0.4+co*(1*i+1)+sqrt(i*i*i)*0.4);
			y.push_back(0.5+si*(2*i+1)-sqrt(i*i*i)*0.1);
		      }
		      else if ((angle>135) & (angle<181)){
			x.push_back(-0.3+co*(2*i+1)+sqrt(i*i*i)*0.5);
//			y.push_back(si*(1*i+1)+sqrt(abs(2-i)*abs(2-i))*0.3);
			y.push_back(si*(1*i+1)+sqrt(i*i*i)*0.6);
		      }
		      else if ((angle>180) & (angle<226)){
			x.push_back(-0.5+co*(2*i+1)+sqrt(i*i*i)*0.7);
			y.push_back(si*(1*i+1)-sqrt(i*i*i)*0.6);
		      }
		      else if ((angle>225) & (angle<271)){
			x.push_back(-0.4+co*(1*i+1)+sqrt(i*i*i)*0.4);
			y.push_back(-0.5+si*(2*i+1)+sqrt(i*i*i)*0.1);
		      }
		      else if ((angle>270) & (angle<316) ){
			x.push_back(0.5+co*(1*i+1)-sqrt(i*i*i)*0.4);
			y.push_back(-1.0+si*(2*i+1)+sqrt(i*i*i)*0.1);
		      }
		      else if ((angle>375) ){
			x.push_back(co*(2*i+1)-sqrt(i*i*i)*0.4);
			y.push_back(si*(1*i+1)+sqrt(i*i*i)*0.1);
		      }
		  }
		  else{ //this is the last part that closes the petal loop
			     // cout <<"x_size "<<x.size();
		      tx=x.at(0);
		      ty=y.at(0);
		      y.push_back(0.0);
		      x.push_back(0.0);		
//		      cout<<"p"<<j<<".a"<<i<<" done:"<<petal[j].action[i].done<<" assigned:"<<petal[j].action[i].assigned;
//		      cout<<"      p"<<j<<".a"<<i-1<<" done:"<<petal[j].action[i-1].done<<" assigned:"<<petal[j].action[i-1].assigned<<endl;
		  }
//		  cout<<"drawing the lines"<<endl;	
		  if ( ((petal[j].action[i].done==1) ) || ((petal[j].action[i].done==1) && (i==len-1)) ){
//		    cout << "aa"<<endl;
//				myLine(x,y, len, i, 3, "g");
				gnuLine2(x, y, 22);

		  }
		  else if ( ((petal[j].action[i].done==2) ) || ((petal[j].action[i].done==2) && (i==len-1))) {
//		    cout << "bb"<<endl;
//				myLine(x,y, len, i, 3, "c");
				gnuLine2(x, y, 32);
		  }						
		  else if ( ((petal[j].action[i].done==0) ) || ((petal[j].action[i].done==0) && (i==len-1))) {
//		    cout << "cc"<<endl;
//				myLine(x,y, len, i, 3, "m");
				gnuLine2(x, y, 12);
		  }						
		  else if ((petal[j].action[i].assigned==1)){
//		    cout << "dd"<<endl;
//				myLine(x,y, len, i, 1, "g");
				gnuLine2(x, y, 21);
		  }
		  else if ((petal[j].action[i].assigned==2)){
//				myLine(x,y, len, i, 1, "c");
				gnuLine2(x, y, 31);
		  }
		  else if ((petal[j].action[i].assigned==0)){
//		    cout << "ff"<<endl;
//				myLine(x,y, len, i, 1, "m");
				gnuLine2(x, y, 11);
				
		  }
		  else if (petal[j].action[i].done<0){ //done regards the vetex at the end of the edge
//		    cout << "gg"<<endl;
//				myLine(x,y, len, i, 1, "k");
				gnuLine2(x, y, 51); ////ayto einai lathos
		  }

/*
		if ((petal[j].action[i].blocked==1)){
			  vector<double> tempx,tempy;  
			  tempx.clear();
			  tempx.push_back(x.at(0));
			  tempy.clear();
			  tempy.push_back(y.at(0));

			  plot(tempx,tempy); set("k"); set(6); set("*"); //this line only plots stars
		}
*/
		  if ((x.size()>1) && (y.size()>1)){
			x.erase(x.begin());
			y.erase(y.begin());
		  }
		}  
		x.clear();
		y.clear();


		tx=tx*1.1;
		ty=ty*1.1;

		if (tx<0){
		  tx-=0.3;
		}
		if (ty<0){
		  ty-=0.3;
		}
		if (tx<mintx)
		  mintx=tx;
		if (ty<minty)
		  minty=ty;
		if (tx>maxtx)
		  maxtx=tx;
		if (ty>maxty)
		  maxty=ty;
		
//		string name ("P");
//		name += std::to_string(j);         // c-string
//		text(tx, ty,name);// set(14);		v

//		string name;
//		if (j==0){
//		  name="BRING_BREAKFAST_";
//		}
//		else if (j==1){
//		  name="CLEAN_TABLE_";
//		}
//		else {
//		  name=gDP.xproc->petal[j].al_actionName[0]+"_";
//		}
//		name += std::to_string(j);         // c-string
//		if (tx>1.5) tx=1.1;
//		if (ty>2.0) ty=2.8;
//		if (ty<-2.0) ty=-2.85;
//		text(tx, ty,name);// set(14);		v
		
	}
	
//	gp << " \n";
//    gp.flush(); 11
//    gp<< "unset multiplot\n";
//    sleep(0.5);

	
	
	//------------------- this is for show active constraints
	/*
	string name ("P");
	int mm=0;
	for (int j=0; j<pnum; j++){
		//gDP.proc.petal[j].length;
		//cout<<"\nconstrains vertices:";
		for (int k=0; k<gDP.xproc->petal[j].to_cNum; k++){
		    std::ostringstream oss;
		    oss.clear();
		    //oss <<"C"<<gDP.proc.petal[j].pcToL[k]<<": ("<<gDP.proc.constr[gDP.proc.petal[j].pcToL[k]].fvertex  <<"->"<< gDP.proc.constr[gDP.proc.petal[j].pcToL[k]].tvertex <<") ";
		    oss <<"C"<<gDP.xproc->petal[j].pcToL[k]<<": ("<<gDP.xproc->constr[gDP.xproc->petal[j].pcToL[k]].fpetal  <<"->"<< gDP.xproc->constr[gDP.xproc->petal[j].pcToL[k]].tpetal <<") ";
		    name.clear();
		    name = oss.str();
		    text(2.0, 2.7-0.25*mm,name);//set("r");// set(14);
		    if (gDP.xproc->constr[gDP.xproc->petal[j].pcToL[k]].stateOn){
		      text(1.6, 2.7-0.25*mm,"+++");//set("r");// set(14);
		    }
		    else{
		      text(2.95, 2.7-0.25*mm,"---");set("r");// set(14);
		    }
		    mm++;
		}
	  
	}	
	*/
//	axis(mintx-0.3,maxtx+1.8,minty-0.5,maxty+0.5);
//	axis(-4,4,-4,4);

#ifdef _FullReDraw
	    gp.flush();
	//        gp<< "unset multiplot\n";

#endif
}


//----------------------------------------------------------------


void GRAPHchatterCallback(const timestorm_msg::Daisy_graph::ConstPtr& msg)
{
  locked=1;
//  ROS_INFO("I heard: %d %d \n", msg->pet[0].p_id,  msg->pet[1].p_id);


  petalNumber= msg->p_num;
  if (petalNumber>=MaxPetals){
      cout <<"GRAPHchatterCallback() ERROR. Too many petals to draw......"<<endl;
      return;
  }

  for (int k=0; k<petalNumber; k++){
      petal[k].id=msg->pet[k].p_id;
      petal[k].assigned=msg->pet[k].p_assigned;
      petal[k].completed=msg->pet[k].p_completed;
      petal[k].interupted=msg->pet[k].p_interupted;
      petal[k].length=msg->pet[k].a_num;
      petal[k].inplan=msg->pet[k].p_inplan;


      if (petal[k].length>=MAX_PETAL_LENGTH){
          cout <<"GRAPHchatterCallback() ERROR. TCannot draw too long petals......"<<endl;
          return;
      }
      for (int m=0; m<petal[k].length; m++){

          petal[k].action[m].id=msg->pet[k].act[m].a_id;
          petal[k].action[m].done=msg->pet[k].act[m].a_done;
          petal[k].action[m].assigned=msg->pet[k].act[m].a_assigned;
          petal[k].action[m].blocked=msg->pet[k].act[m].a_blocked;
          petal[k].action[m].interupted=msg->pet[k].act[m].a_interupted;
          petal[k].action[m].name=msg->pet[k].act[m].a_name;
          petal[k].action[m].param=msg->pet[k].act[m].a_param;
      }
  }
  locked=0;

  cout << "new plan state received....      "<< std::flush;
  
  drawPetals();
  

}


//----------------------------------------------------------------
//----------------------------------------------------------------

int main(int argc, char **argv) {

	
//    graph_display();
//    plt::show();

//  	mp.gnuDraw();


    
//    gp<< "set multiplot\n";
    gp << "set xrange [-4.5:6.5]\n";
    gp << "set yrange [-5.5:5.5]\n";
    gp<< "set size 1,1\n";
    gp<< "set origin 0,0\n";

    
    //blue:3 cyan:5, 


    gp<<"set style arrow 11 head empty size screen 0.03,15,135 ls 2 lw 2 lc 13\n"; //magenta empty
    gp<<"set style arrow 12 head filled size screen 0.03,15,135 ls 2 lw 2 lc 13\n"; //magenta filled

    gp<<"set style arrow 21 head empty size screen 0.03,15,135 ls 2 lw 2 lc 2\n"; //green empty
    gp<<"set style arrow 22 head filled size screen 0.03,15,135 ls 2 lw 2 lc 2\n"; //green filled
    
    gp<<"set style arrow 31 head empty size screen 0.03,15,135 ls 2 lw 2 lc 3\n"; //blue empty
    gp<<"set style arrow 32 head filled size screen 0.03,15,135 ls 2 lw 2 lc 3\n"; //blue filled

    gp<<"set style arrow 41 head empty size screen 0.03,15,135 ls 2 lw 2 lc 1\n"; //red empty
    gp<<"set style arrow 42 head filled size screen 0.03,15,135 ls 2 lw 2 lc 1\n"; //red filled
    

    gp<<"set style arrow 51 head empty size screen 0.03,15,135 ls 2 lw 2 lc 7 \n"; //black
    gp<<"set style arrow 52 head filled size screen 0.03,15,135 ls 2 lw 2 lc 7 \n"; //black

    gp<<"set style arrow 61 head empty size screen 0.03,15,135 ls 2 lw 2 lc 8 \n"; //black
    gp<<"set style arrow 62 head filled size screen 0.03,15,135 ls 2 lw 2 lc 8 \n"; //black

    
/*    
    gp<<"set style arrow 1 head filled size screen 0.025,30,45 ls 1\n";
    gp<<"set style arrow 2 head nofilled size screen 0.03,15 ls 2\n";
    gp<<"set style arrow 3 head filled size screen 0.03,15,45 ls 1\n";
    gp<<"set style arrow 4 head filled size screen 0.03,15 ls 2\n";
    gp<<"set style arrow 5 heads noborder size screen 0.03,15,135 ls 1\n";
    gp<<"set style arrow 6 head empty size screen 0.03,15,135 ls 2 lc 3\n";
    gp<<"set style arrow 7 head filled size screen 0.03,15,135 ls 2\n";
    gp<<"set style arrow 8 nohead ls 1\n";
    gp<<"set style arrow 9 heads size screen 0.008,90 ls 2\n";
*/    
    
    gp<< "set style line 1 lt 1 lw 1 lc 16  \n"; //black 

    gp<< "set style line 31 lt 1 lw 1 lc 3  \n"; //blue 
    gp<< "set style line 33 lt 1 lw 3 lc 3  \n";
    gp<< "set style line 36 lt -1 lw 6 lc 3  \n";

    gp<< "set style line 21 lt 2 lw 1 lc 2  \n"; //green
    gp<< "set style line 23 lt 1 lw 3 lc 2  \n";   
    gp<< "set style line 26 lt 2 lw 6 lc 2  \n";
   
    gp<< "set style line 11 lt 3 lw 1 lc 13  \n"; //red or magenta
    gp<< "set style line 13 lt 1 lw 3 lc 13  \n";
    gp<< "set style line 16 lt 3 lw 6 lc 13  \n";
    

  
    ros::init(argc, argv, "timestorm_ps_node");
    ros::NodeHandle n;

    ros::Subscriber GRAPH_sub = n.subscribe("/daisy/graph_chatter", 1, GRAPHchatterCallback);

    ros::Rate loop_rate(0.3);

    ros::spin();


	    sleep(999999999);
//    gp<< "unset multiplot\n";
	

}



