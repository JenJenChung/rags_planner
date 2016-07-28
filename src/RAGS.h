#include "sensor_msgs/LaserScan.h" // Check what the incoming message type is
#include <actionlib/client/simple_action_client.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
#include <queue>
#include <vector>
#include <functional>
#include <cmath>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <stdlib.h>
#include <typeinfo>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "config.h"
#include "vertex.h"
#include "edge.h"
#include "node.h"
#include "graph.h"
#include "queue.h"
#include "search.h"

const double pi = 3.14159265358979323846264338328 ;

#include "util_functions.h"


class RAGS
{
  public:
    RAGS(ros::NodeHandle) ;
    ~RAGS() {}
    
  private:
    ros::Subscriber subResult ;
    ros::Subscriber subSensor ;
    ros::Publisher pubWaypoint ;
    bool fWaypoint ;
    dji_sdk::Waypoint waypoint ;
    
    void waypointStatusCallback(const dji_sdk::MissionPushInfo&) ;
    void sensorCallback(const sensor_msgs::LaserScan&) ;
    
    vector <Node *> SGPaths ; // non-dominated path set as node link list from start to goal
    vector <Node *> newNodes ; // set of available paths to goal
    vector <Vertex *> nextVerts ; // store connected vertices
    Vertex * cmdWaypoint ; // commanded waypoint
    Vertex * goalWaypoint ; // goal waypoint
};

RAGS::RAGS(ros::NodeHandle nh){
  subResult = nh.subscribe("dji_sdk/mission_status", 10, &RAGS::waypointStatusCallback, this) ;
  subSensor = nh.subscribe("base_scan", 10, &RAGS::sensorCallback, this) ;
  pubWaypoint = nh.advertise<dji_sdk::Waypoint>("cmd_wp", 10) ;
  fWaypoint = false ;
  
  // Read in satellite image
  string temp ;
  ros::param::get("RAGS/satellite_image", temp) ;
  cv::Mat img = cv::imread(temp, 0);
  vector< double > iSize;
  iSize.push_back(img.size().width-1);
  iSize.push_back(img.size().height-1);
  vector< vector< double > > vertVec;
	double x, y, radius;
	x = iSize[0];
	y = iSize[1];
  
  // Perform initial sweep to find non-dominated path set
  srand(time(NULL));
	bool loop = true ;
	while (loop) {
    int numVerts = 150 ; //int((x*y)/1000) ;
	  cout << "Generating Random Vertices in " << x << " by " << y << endl;
	  vertVec = makeVertices(x,y,numVerts);
	  radius = sqrt((6.0/pi)*x*y*(log((double)numVerts)/(double)numVerts)) ;
	  cout << "Connecting with radius " << radius << endl;
	  Graph * testGraph = new Graph(vertVec, radius, img);
	  Vertex * sourceSec = testGraph->GetVertices()[0] ; // top-left sector
	  Vertex * goalSec = testGraph->GetVertices()[testGraph->GetNumVertices()-1] ; // bottom-right sector

	  //Create search object and perform path search from source to goal
	  cout << "Creating search object..." ;
	  Search * testSearch = new Search(testGraph, sourceSec, goalSec) ;
	  cout << "complete.\n" ;

	  cout << "Performing path search from (" <<  sourceSec->GetX() << "," << sourceSec->GetY() << ") to (" ;
	  cout << goalSec->GetX() << "," << goalSec->GetY() << ")...\n" ;
	
    pathOut pType = ALL ;
    vector <Node *> GSPaths = testSearch->PathSearch(pType) ;
	  cout << "Path search complete, " << GSPaths.size() << " paths found.\n" ;

	  if (GSPaths.size() < 30 || GSPaths.size() > 100) {
		  delete testGraph ;
		  testGraph = 0 ;
		  delete testSearch ;
		  testSearch = 0 ;
		  GSPaths.clear() ;
		  continue ;
	  }
	  else {
	    loop = false ;
	    // Set linked list from start node to goal node
	    for(int i = 0; i < GSPaths.size(); i++)
		    SGPaths.push_back(GSPaths[i]->ReverseList(0));
	
	    // Compute cost-to-go for all path nodes
	    for(int i = 0; i < SGPaths.size(); i++)
		    SGPaths[i]->SetCTG(GSPaths[i]->GetMeanCost(),GSPaths[i]->GetVarCost()) ;
	    
	    // Initialise search vertex and node variables
	    cmdWaypoint = SGPaths[0]->GetVertex() ;
	    goalWaypoint = GSPaths[0]->GetVertex() ;
	    newNodes = SGPaths ;
	    cmdWaypoint->SetNodes(newNodes) ;
	    
//	    // Write paths to file
//	    stringstream pFileName ;
//		  pFileName << "../results/SGPaths" << trialNum << ".txt" ;
//		
//		  ofstream pathsFile ;
//		  pathsFile.open(pFileName.str().c_str()) ;
//		
//		  for (ULONG i = 0; i < (ULONG)SGPaths.size(); i++)
//		  {
//			  pathsFile << "Path " << i << endl ;
//			  Node * curNode = SGPaths[i] ;
//			  while (curNode->GetParent())
//			  {
//				  pathsFile << "(" << curNode->GetVertex()->GetX() << ","
//					  << curNode->GetVertex()->GetY() << ")\n" ;
//				  curNode = curNode->GetParent() ;
//			  }
//			  pathsFile << "(" << curNode->GetVertex()->GetX() << ","
//				  << curNode->GetVertex()->GetY() << ")\n\n" ;				
//		  }
//		  pathsFile.close() ;
	  }
	  delete testGraph ;
	  testGraph = 0 ;
	  delete testSearch ;
	  testSearch = 0 ;
	}
}

void RAGS::waypointStatusCallback(const dji_sdk::MissionPushInfo& msg){
  if (msg.data_2 == 0){ // Success status message received
    newNodes = cmdWaypoint->GetNodes() ;
    
    // Identify next vertices
	  for (int i = 0; i < newNodes.size(); i++)
	  {
		  bool newVert = true ;
		  for (int j = 0; j < nextVerts.size(); j++)
		  {
			  if ((nextVerts[j]->GetX() == newNodes[i]->GetParent()->GetVertex()->GetX() &&
				  nextVerts[j]->GetY() == newNodes[i]->GetParent()->GetVertex()->GetY()) ||
				  (nextVerts[j]->GetX() == cmdWaypoint->GetX() && nextVerts[j]->GetY() == cmdWaypoint->GetY()))
			  {
				  newVert = false ;
				  break ;
			  }
		  }
		  if (newVert)
			  nextVerts.push_back(newNodes[i]->GetParent()->GetVertex()) ;
	  }
	
	  // Identify next vertex path nodes
    vector <Node *> tmpNodes ; // temporarily store next nodes
	  for (int i = 0; i < nextVerts.size(); i++)
	  {
		  tmpNodes.clear() ;
		  for (int j = 0; j < newNodes.size(); j++)
		  {
			  if (nextVerts[i]->GetX() == newNodes[j]->GetParent()->GetVertex()->GetX() &&
				  nextVerts[i]->GetY() == newNodes[j]->GetParent()->GetVertex()->GetY())
				  tmpNodes.push_back(newNodes[j]->GetParent()) ;
		  }
		  nextVerts[i]->SetNodes(tmpNodes) ;
	  }
	
	  fWaypoint = true ;
	}
}

void RAGS::sensorCallback(const sensor_msgs::LaserScan& msg){
  if (fWaypoint){
    // Set cost-to-come for next vertices
		SetTrueEdgeCosts(cmdWaypoint, nextVerts, msg) ;
		
		// Rank next vertices according to probability of improvement
		sort(nextVerts.begin(),nextVerts.end(),ComputeImprovementProbability) ;
		
		// Set commanded waypoint to best vertex
		cmdWaypoint = nextVerts[0] ;
		
		// Convert waypoint to correct coordinate frame and publish to controller
		dji_sdk::Waypoint waypoint ;
		waypoint.latitude = 0.0 ; // some conversion of coordinate frames
		waypoint.longitude = 0.0 ; // some conversion of coordinate framess
		waypoint.altitude = 0.0 ;
		waypoint.staytime = 0.0 ;
		waypoint.heading = 0.0 ;
		
		pubWaypoint.publish(waypoint) ;
		
		// Clear vectors for next step
		newNodes.clear() ;
		nextVerts.clear() ;
		
    fWaypoint = false ;
  }
}
