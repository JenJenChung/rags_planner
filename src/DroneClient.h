class DroneClient
{
  public:
    DroneClient(ros::NodeHandle) ;
    ~DroneClient() {}
    
    void sendWaypointRequest(DJIDrone *) ;
    
  private:
    ros::Subscriber subWaypoint ;
    void waypointCallback(const dji_sdk::Waypoint&) ;
    dji_sdk::WaypointList cmdWPList ;
    bool cmdWPReceived ;
} ;

DroneClient::DroneClient(ros::NodeHandle nh): cmdWPReceived(false) {
  subWaypoint = nh.subscribe("cmd_wp", 10, &DroneClient::waypointCallback, this) ;
}

void DroneClient::waypointCallback(const dji_sdk::Waypoint& msg){
  cmdWPList.waypoint_list.push_back(msg) ;
  cmdWPReceived = true ;
}

void DroneClient::sendWaypointRequest(DJIDrone * drone){
  if (cmdWPReceived){
    ROS_INFO("PREPARING WAYPOINT");
    drone->waypoint_navigation_send_request(cmdWPList);
    ROS_INFO("SENT WAYPOINT");
    cmdWPList.waypoint_list.clear() ;
    cmdWPReceived = false ;
  }
}
