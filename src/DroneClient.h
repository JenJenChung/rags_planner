class DroneClient
{
  public:
    DroneClient(ros::NodeHandle) ;
    ~DroneClient() {
      delete drone ;
    }
    
  private:
    ros::Subscriber subWaypoint ;
    void waypointCallback(const dji_sdk::Waypoint&) ;
    DJIDrone* drone ;
} ;

DroneClient::DroneClient(ros::NodeHandle nh){
  subWaypoint = nh.subscribe("cmd_wp", 10, &DroneClient::waypointCallback, this) ;
  drone = new DJIDrone(nh);
}

void DroneClient::waypointCallback(const dji_sdk::Waypoint& msg){
  dji_sdk::WaypointList newWPList ;
  newWPList.waypoint_list.push_back(msg) ;
  
  drone->waypoint_navigation_send_request(newWPList);
}
