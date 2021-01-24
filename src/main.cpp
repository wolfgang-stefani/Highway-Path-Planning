#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


class Vehicle{

  public:
    int s, future_s, car_id, lane;
    float speed,d;
    // Add a flag to indicate if this car is assigned or not
    bool initialize = false;
 
    
    void cal_mag_v(double v_x, double v_y, double prev_size){
      speed = sqrt(v_x*v_x + v_y*v_y);
      
      // Prediction the future location
      future_s = s + prev_size * 0.02 * speed;
      update_lane();
    }
    
    void update_lane(){
      if( d >= 0 && d < 4) lane = 0;
      else if (d >= 4 && d < 8) lane = 1;
      else if (d >= 8 && d < 12) lane = 2;

    }
};


class List_Vehicle{
public:
    
    vector<Vehicle> sensor_readings;
    
    double ego_future_s,ego_speed,prev_ego_s;
    int ego_lane = 1;
    double ego_s = 0.0;
    double detect_range = 60;
    double too_close = 15;
    double follow_speed = 30.0;
    double ref_val = 0.0; //MPH
   
    
    // Append new readings to the previous. Handles if the new car appears in the readings
    void append(Vehicle item){
        bool found = false;
        // Initialize Sensor Readings for the first time
        if (sensor_readings.size() == 0 ){
            sensor_readings.push_back(item);
            std::cout<<"---------Sensor: First Adding -- ID:"<< item.car_id<< " Lane:"<< item.lane <<" s:"<<item.s<<" Future s:"<< item.future_s <<std::endl;
        }
        // If there are items inside object, search for the same car id
        else{
            for (int i = 0; i < sensor_readings.size(); ++i) {
                // Same id found, update the infor base on the new item
                if (sensor_readings[i].car_id == item.car_id) {
                    sensor_readings[i].d = item.d;
                    sensor_readings[i].s = item.s;
                    sensor_readings[i].lane = item.lane;
                    sensor_readings[i].speed = item.speed;
                    sensor_readings[i].future_s = item.future_s;
                    found = true;
                    
//                    std::cout<<"-----------Sensor: UPDATE ID:"<<item.car_id<< " Lane:"<< item.lane <<" s:"<<item.s<<" Future s:"<< item.future_s <<" Dist to ego:"<<item.s - ego_s<<std::endl;
                    // Jump out
                    break;
                }
            }
            // If not found, meaning new car appear in the reading
            if (!found) {
                sensor_readings.push_back(item);
                std::cout<<"---------Sensor: Adding -- ID:"<< item.car_id << " Lane:"<< item.lane<<" s:"<<item.s<<" Future s:"<< item.future_s <<" Dist to ego:"<<item.s - ego_s<<std::endl;
            }
        }
    }
    
    // Find if there is sensor reading missing from the previous result. This handles missing sensor reading when the sensored car is too close
    void trace_back (vector<Vehicle> now_reading, int prev_size){
        bool found = false;
        for (int i = 0; i < sensor_readings.size(); ++i) {
            for (int j = 0; j < now_reading.size(); ++j) {
                if (sensor_readings[i].car_id == now_reading[j].car_id){
                    // Found and exit
                    found = true;
                    break;
                }
            }
            if (!found){
                // Check if the previous is moving outside the range or too close
                // check  it is within the range now and in the future
                if ((abs(sensor_readings[i].s - ego_s) <= detect_range) || (abs(sensor_readings[i].future_s - ego_future_s) <= detect_range) ){
                
                    double time_passed = (ego_s - prev_ego_s)/ego_speed;
                    // Update prediction according to previous reading
                    int predict_s = sensor_readings[i].s + sensor_readings[i].speed * time_passed;
                    // assume it is still in the same lane
                    int prev_s = sensor_readings[i].s;
                    
                    sensor_readings[i].s = predict_s;
                    sensor_readings[i].future_s = predict_s + (double)prev_size * 0.02 * sensor_readings[i].speed;
//                    std::cout<<"---------Sensor: predicting lost reading -- ID:"<< sensor_readings[i].car_id<<" Lane:"<< sensor_readings[i].lane<<" prev s:"<< prev_s <<" predicting s: "<<predict_s<<" Future s:"<< sensor_readings[i].future_s <<" Dist to ego:"<<predict_s - ego_s <<std::endl;
                    
                }
                // if it is outside the range, delete this
                else{
                    std::cout<<"---------Sensor: REMOVING Out of bound -- ID:"<< sensor_readings[i].car_id <<" Lane:"<< sensor_readings[i].lane <<" s:"<<sensor_readings[i].s<<" Future s:"<< sensor_readings[i].future_s <<std::endl;
                    
                    sensor_readings.erase(sensor_readings.begin()+i);
                    
                    // potential bug for not removing correctly
                    
                }
            }
            else found = false;
        }
        return;
    }
    
    void update_ego (double car_s, double car_speed,double prev_size){
        ego_speed = car_speed/2.24; // Convert mph to m/s
        prev_ego_s = ego_s;
        ego_s = car_s;
        ego_future_s = car_s + (double)prev_size * 0.02 * ego_speed;
       
    }
    
    void locate_front_car (Vehicle &front){
        Vehicle f;
        f.s =  10000;
        
        for (int i = 0;i < sensor_readings.size();++i) {
            float d = sensor_readings[i].d;
            int s = sensor_readings[i].s;
            
            if (d < (2+4*ego_lane+2) && d > (2+4*ego_lane-2)){
                if (s > ego_s && s - ego_s <detect_range){
                    // Check if this is the closet to ego
                    if (abs(s-ego_s) < abs(f.s - ego_s)){
                        f = sensor_readings[i];
                        std::cout<< "------Found F\n";
                    }
                }
            }
        }
        
        front = f;
    }
    
    void locate_car (Vehicle &right_front, Vehicle &right_back, Vehicle &left_front, Vehicle &left_back, Vehicle &front){
        Vehicle rf,rb,lf,lb,f;
        // Make s big to find the closest sensor car
        rf.s = 10000;
        rb.s = 10000;
        lf.s = 10000;
        lb.s = 10000;
        f.s =  10000;
        
        // Scan through all sensor readings
        for (int i = 0;i < sensor_readings.size();++i) {
            float d = sensor_readings[i].d;
            int s = sensor_readings[i].s;
            
            // Right
            if (d < (2+4*(ego_lane+1)+2) && d > (2+4*(ego_lane+1)-2) && ego_lane < 2){
                // front
                if (s > ego_s && s - ego_s <detect_range){
                    // Check if this is the closet to ego
                    if (abs(s-ego_s) < abs(rf.s - ego_s)){
                        rf = sensor_readings[i];
                        std::cout<< "------Found RF ID:"<<sensor_readings[i].car_id <<"\n";
                        
                    }
                }
                // back
                else if (s <= ego_s && s - ego_s <detect_range){
                    // Check if this is the closet to ego
                    if (abs(s-ego_s) < abs(rb.s - ego_s)){
                        rb = sensor_readings[i];
                        std::cout<< "------Found RB ID:"<<sensor_readings[i].car_id <<"\n";
                    }
                }
            }
            //Left
            else if (d < (2+4*(ego_lane-1)+2) && d > (2+4*(ego_lane-1)-2) && ego_lane >0 ){
                // front
                if (s > ego_s && s - ego_s <detect_range){
                    // Check if this is the closet to ego
                    if (abs(s-ego_s) < abs(lf.s - ego_s)){
                        lf = sensor_readings[i];
                        std::cout<< "------Found LF ID:"<<sensor_readings[i].car_id <<"\n";
                    }
                }
                // back
                else if (s <= ego_s && s - ego_s <detect_range){
                    // Check if this is the closet to ego
                    if (abs(s-ego_s) < abs(lb.s - ego_s)){
                        lb = sensor_readings[i];
                        std::cout<< "------Found LB ID:"<<sensor_readings[i].car_id <<"\n";
                    }
                }
            }
            //Front
            else if (d < (2+4*ego_lane+2) && d > (2+4*ego_lane-2)){
                if (s > ego_s && s - ego_s <detect_range){
                    // Check if this is the closet to ego
                    if (abs(s-ego_s) < abs(f.s - ego_s)){
                        f = sensor_readings[i];
                        std::cout<< "------Found F ID:"<<sensor_readings[i].car_id <<"\n";
                    }
                }
            }
        }
        // Pass those back to reference variables
        right_front = rf;
        right_back = rb;
        left_front = lf;
        left_back = lb;
        front = f;
        
    }

    float follow_cost (Vehicle sensor_reading, bool front_cost, bool future_s){
        float target_car_speed = sensor_reading.speed;
        float norm_speed = (target_car_speed - ego_speed)/ego_speed;
        float dist_cost;
        float speed_cost;
        // Check if the sensor readings is written or not
        if (sensor_reading.initialize == false ) {
            return 0.0;
        }
        
      // std::cout << "COST -- norm_speed:" << norm_speed <<" sensor car speed:"<< target_car_speed<<" ego speed:"<<ego_speed <<std::endl;
        
        if (future_s){
            // if uses future value of s, car_s need to be also future value
            dist_cost = 1 - abs(sensor_reading.future_s - ego_future_s)/detect_range;
        }
        
        else{
            dist_cost = 1 - abs(sensor_reading.s - ego_s)/detect_range;
        }
        
        if (front_cost) {
            speed_cost = pow(2,-norm_speed) - norm_speed;
        }
        else{
            speed_cost = pow(2,norm_speed) + norm_speed;
        }
        return (speed_cost + dist_cost)/2;
        
    }
    
    double one_side_cost (Vehicle front, Vehicle back, bool right, bool future_s_flag){
        
        
        float front_cost = follow_cost(front, true, future_s_flag);
        float back_cost = follow_cost(back, false, future_s_flag);
        
        // Calculate total cost
        std::cout<<"--Cost: front cost: "<< front_cost<< std::endl;
        std::cout<<"--Cost: back cost: "<< back_cost<< std::endl;
        double cost;
        // Calculate total Right cost
        if (front_cost != 0.0 && back_cost != 0.0 ) {
            cost = (front_cost + back_cost)/2;
        }
        else  cost = front_cost + back_cost;
        // Check if the right cost is real small
        if (fabs(cost)<0.001){
            cost = 0.0;
        }
        //    std::cout << "**** Total cost: "<<cost << std::endl;
        
        return cost;
        
    }
    
    
    int calculate_cost(bool future_s_flag = true){
        // Process right lane
        Vehicle right_front,right_back,left_front,left_back,front;
        
        locate_car(right_front,right_back,left_front,left_back,front);
        
        double right_cost = one_side_cost(right_front,right_back, true,future_s_flag);
        std::cout << "--Cost: ***** Total Right cost: "<<right_cost << std::endl;
        
        
        double left_cost = one_side_cost(left_front,left_back,false,future_s_flag);
        std::cout << "--Cost: ***** Total Left cost: "<<left_cost << std::endl;
        
        
        // Calculate same lane front car cost
        
        float front_cost = follow_cost(front, true,future_s_flag);
        std::cout << "--Cost: ***** front cost: "<<front_cost << std::endl<< std::endl;
        
        // Write to follow speed
        if (front_cost > 1){
            // Convert m/s back to mph, if front cost too high, set half of the front car speed
            follow_speed = front.speed * 2.24 * 0.2;
        }
        else {
            follow_speed = front.speed * 2.24 * 0.6 ;
        }
        // Base on the cost decide which lane to change
        // IF both lane has no cars
        if (right_cost == 0 && left_cost == 0 ){
            // shift to right if not at the right most lane
            if (ego_lane < 2) {
                return 2;
            }
            // if at the right most lane, turn left
            else if (ego_lane == 2){
                return 1;
            }
        }
        
        // if the costs are too high stay and slow down
        else if ((right_cost > 0.9 && left_cost > 0.9) || (front_cost < right_cost &&  front_cost < left_cost)||((right_cost > 0.9 || right_cost > front_cost) && ego_lane == 0) || ((left_cost > 0.9 || left_cost > front_cost) && ego_lane == 2)){
            // DONT turn since it too risky
            std::cout << "--Cost: *** Too Risky to turn"<<std::endl;
            return 0;
        }
        
        else{
            
            if ((right_cost > left_cost && ego_lane > 0) || (ego_lane == 2 && left_cost < front_cost) ){
                //turn left:
                return 1;
            }
            else if ((right_cost < left_cost && ego_lane < 2) || (ego_lane == 0 && right_cost < front_cost)){
                //trun right:
                return 2;
            }
    
            else {
                // If there are equal don't change lane
                std::cout << "--Cost: Left and right "<< left_cost<<" "<< right_cost <<std::endl;
                return 0;
            }
        }
    }
    
    
    bool safe_to_turn (bool right_turn, bool future_s_flag){
        
        Vehicle right_front,right_back,left_front,left_back,front;
        
        locate_car(right_front,right_back,left_front,left_back,front);
        
        float front_cost = follow_cost(front, true,future_s_flag);
        
        if (right_turn){
            
            // if there is no readings written, add 100 in order to return safe to tunr later in the calculations
            if (right_front.initialize == false){
                right_front.s = ego_s + 100;
                right_front.future_s =ego_future_s + 100;
            }
            
            if (right_back.initialize == false){
                right_back.s = ego_s - 100;
                right_back.future_s =ego_future_s - 100;
            }
            
            // Check s to my own ego cars' s to see if it is too close
            if (right_front.s - ego_s <= too_close || ego_s - right_back.s <=too_close|| right_front.future_s - ego_future_s <=too_close || ego_future_s - right_back.future_s <= too_close){
                std::cout << "---Safe_check: ego s:"<< ego_s << " front s:"<< right_front.s <<" back s:" << right_back.s<< std::endl;
                std::cout << "---Safe_check: ego future s:"<< ego_future_s << " front future s:"<< right_front.future_s <<" back future s:" << right_back.future_s<< std::endl;
                std::cout << "---Safe_check: !!!! NOT SAFE ON Right!!!!!" << std::endl;
                
                return false;
            }
            // If not too close, recalculate the cost to be safe
            else{
                
                double side_cost = one_side_cost(right_front,right_back, true, future_s_flag); // using future values


                std::cout << "---Safe_check: Recalculate cost  side cost:"<< side_cost<<" front cost:"<< front_cost << std::endl;
                if (side_cost < front_cost){
                    return true;
                }
                else return false;
            }
        }
        else{
            // if there is no readings written, add 100 in order to return safe to tunr later in the calculations
            if (left_front.initialize == false){
                left_front.s = ego_s + 100;
                left_front.future_s =ego_future_s + 100;
            }
            
            if (left_back.initialize == false){
                left_back.s = ego_s - 100;
                left_back.future_s =ego_future_s - 100;
            }
            
            if (left_front.s - ego_s <= too_close || ego_s - left_back.s <=too_close || left_front.future_s - ego_future_s <=too_close || ego_future_s - left_back.future_s <= too_close){
                std::cout << "---Safe_check: ego s:"<< ego_s << " front s:"<< left_front.s <<" back s:" << left_back.s<< std::endl;
                std::cout << "---Safe_check: ego future s:"<< ego_future_s << " front future s:"<< left_front.future_s <<" back future s:" << left_back.future_s<< std::endl;
                std::cout << "---Safe_check: !!!! NOT SAFE ON Left!!!!!" << std::endl;
                return false;
            }
            else{
                double side_cost = one_side_cost(left_front,left_back, false,future_s_flag);
                
//                float front_cost = follow_cost(front, true,future_s_flag);
                std::cout << "---Safe_check: Recalculate cost  side cost:"<< side_cost<<" front cost:"<< front_cost << std::endl;
                if (side_cost < front_cost){
                    return true;
                }
                else return false;
            }
            
        }
        
    }
    
};


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
    
    int ego_state = 0;
    bool change_lane_fail = false;
    int target_lane;
    bool lane_change_set = false;
    int fail_count = 0;
    List_Vehicle vehicle_list;
    bool slow_down = false;

 h.onMessage([&slow_down,&vehicle_list,&fail_count,&target_lane,&lane_change_set,&change_lane_fail,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
           &map_waypoints_dx,&map_waypoints_dy,&ego_state]
          (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
           uWS::OpCode opCode) {
              
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      // autonomous driving if s != ... ? DEBUG
      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          
          /**
          Planner is receiving localization and sensor fusion data from the simulator ? DEBUG
          */
          
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path (path so far?) data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data (a list of all other cars on the same side of the road)
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          double pos_x;
          double pos_y;
          double angle;
          int prev_size = previous_path_x.size();
          bool front_car_detected = false;
            

          /**
           * TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
           */
          
          // Calculate future car_s
          vehicle_list.update_ego(car_s, car_speed,prev_size);
           
          if (prev_size >  0) {
            car_s = end_path_s;
          }
            
          vector<Vehicle> temp_readings;
            
          // Reading sensor fusion
          for(int i = 0; i < sensor_fusion.size();++i){
            float d = sensor_fusion[i][6];
            Vehicle car;
            car.initialize = true;
            car.s = sensor_fusion[i][5];
            car.car_id = sensor_fusion[i][0];
            car.d = d;
            car.cal_mag_v(sensor_fusion[i][3], sensor_fusion[i][4], (double) prev_size);
                
            // Enter 30 meters within the front car
            if (abs(car.s-car_s) < vehicle_list.detect_range) {
              
//                    if (d < (2+4*vehicle_list.ego_lane+2) && d > (2+4*vehicle_list.ego_lane-2) && car.s > car_s && car.s-car_s <vehicle_list.detect_range){
//                        front_car_detected = true;
//                    }
                vehicle_list.append(car);
                temp_readings.push_back(car);
              }
            }
            
          // Trace back missing sensor reading and delete out of range items
          vehicle_list.trace_back(temp_readings, prev_size);
            
          // Find the front cars only to see if the car is in front of us.
          Vehicle front;
          vehicle_list.locate_front_car(front);
          float front_cost = vehicle_list.follow_cost(front, true,true);
            
          if (front_cost == 0) {
              front_car_detected = false;
              slow_down = false;
          }
          else {
              front_car_detected = true;
              std::cout<<"----Front Cost: "<<front_cost<<"\n";
              if (front_cost > 0.9) {
                  slow_down=true;
              }
              else if (front_cost < 0.5) {
                  slow_down = false;
              }
          }
            
          // Decision making/jumping states
          switch (ego_state) {
                case 0:
              
                    if (front_car_detected){
                      // calculate the cost for left and right and decide which one to do
                      std::cout<< "FSM: !!!! Front car detected"<< std::endl;
                      int turn_result = vehicle_list.calculate_cost();
                        
                        
                      if (turn_result == 1) { //1 left 2 right 0 not assigned
                        // Jump to PLCL
                        std::cout << "FSM: Trying left" << std::endl;
                        ego_state = 1;
                        fail_count = 0 ;
                      }
                      else if (turn_result == 2) {
                        // Jumpe to PLCR
                        std::cout << "FSM: Trying right" << std::endl;
                        ego_state = 2;
                        fail_count = 0;
                      }
                      
                      else {
                        ego_state = 0; //stay
                        std::cout << "FSM: Keep lane" << std::endl;
                      }
                    }
                
                    break;
                    
                // Prepare for lane change left
                case 1:
                    // check if the future position is still vaild for lane change on the left
                    // and compare to the cost of distance to the front car
                    std::cout << "FSM: Prepare for lane change left   Fail count:"<< fail_count << std::endl;
                    // Use fail count to run at least this state 3 times before move to next state to have stable sensor reading
                    if (vehicle_list.safe_to_turn(false,true) && fail_count >= 3){
                        std::cout << "FSM: Safe to turn left" << std::endl;
                        ego_state = 3;
                        target_lane = vehicle_list.ego_lane - 1;
                        lane_change_set = false;
                        fail_count = 0;
                    }
                    else{
                        // stay in this state
                        // And increase fail count
                        ego_state = 1;
                        fail_count += 1;
                        
                        if (fail_count > 10 ){
                            if (fail_count > 20){
                            // If fail too much jump back to state 0 to restart
                                ego_state = 0;
                            }
                        }
                    
                    }
                    break;
                // Prepare for lane change right
                case 2:
                    std::cout << "FSM: Prepare for lane change Right   Fail count:"<< fail_count << std::endl;
                    // check if the future position is still safe for lane change on the right
                    if (vehicle_list.safe_to_turn(true,true) && fail_count >=3){
                        std::cout << "FSM: Safe to turn right" << std::endl;
                        ego_state = 4;
                        target_lane = vehicle_list.ego_lane + 1;
                        lane_change_set = false;
                        fail_count = 0;
                    }
                    else{
                        // stay in this state
                        // And increase fail count
                        ego_state = 2;
                        fail_count += 1;
                        
                        if (fail_count > 20){
                            ego_state = 0;
                            
                        }
                        
                    }
                    break;
                // Lane change to left
                case 3:
                    if (car_d < (2+4*target_lane+1) && car_d > (2+4*(target_lane)-1)) {
                        ego_state = 0;
                        std::cout << "FSM: ***"<<std::endl<<"Lane shift done"<<std::endl;
                        fail_count = 0;
                    }
                    // Set lane change action and change the ego_lane to target change lane. Only run once
                    if (!lane_change_set) {
                        vehicle_list.ego_lane -= 1;
                        lane_change_set = true;
                    }
                
                    break;
                // Lane change to right
                case 4:
                    if (car_d < (2+4*target_lane+1) && car_d > (2+4*(target_lane)-1) ) {
                        ego_state = 0;
                        std::cout << "FSM: ***"<<std::endl<<"Lane shift done"<<std::endl;
                        fail_count = 0;
                    }
                    
                    if (!lane_change_set) {
                        vehicle_list.ego_lane += 1;
                        lane_change_set = true;
                    }
                    break;
                default:
                    std::cout<<"FSM: ego state number unknown, moving back to state 0\n";
                    ego_state = 0;
                    
            }
                   
          if (vehicle_list.ref_val < 49.5 && !slow_down){
                vehicle_list.ref_val += 0.225;

            }
          else if (slow_down && vehicle_list.ref_val > vehicle_list.follow_speed){
                vehicle_list.ref_val -= 0.225;
                std::cout <<"------Speed Control: Slowing Down  ref val: " << vehicle_list.ref_val<<" Target_follow_speed: " << vehicle_list.follow_speed<< std::endl;
                
                
            }
          else if (vehicle_list.ref_val <= vehicle_list.follow_speed){
                slow_down = false;
                std::cout << "------Speed Control: Reseting slow down" << std::endl;
            }
            
          // ptsx and ptsy are the (future) way points
          vector<double> ptsx;
          vector<double> ptsy;
            
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
            
          if(prev_size < 2){
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
                
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
                
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          }
          
          else {
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];
                
              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
                
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
                
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
                
          }
            
          // Add 3 future points to ptsx and ptsy
          // As car_s is in frenet coordinates, we need to convert it to global x,y coordinates using the getXY function
          vector<double> next_wp0 = getXY(car_s + 30, (2+4*vehicle_list.ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2+4*vehicle_list.ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2+4*vehicle_list.ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
            
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
            
          // Transform ptsx and ptsy to 0 degree of yaw coordinate
          for (int i = 0; i < ptsx.size();++i){
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
                
              ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }
                        
          // Add all previous points to next_x_vals and next_y_vals
          // This will help to get a smooth transition to the new points that we calculate later
          for (int i = 0; i < previous_path_x.size();++i){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }
            
          // Interpolating Points
          // The spline function interpolates points between known waypoints.
          // There are also other methods you could use like Bezier curve fitting or fitting polynomials.
          // For trajectory generation we use the spline function instead of polynomial trajectory generation.
          tk::spline s;
          s.set_points(ptsx, ptsy);
          
          // Now we need to find all spline points till the horizon (here 30m) with such a spacing that ego vehicle travels at desired speed
          // If we know the x points (here 30m) spline is able to get the corresponding y points
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y* target_y);
            
          double x_add_on = 0;
            
          for (int i = 1; i <= 50-previous_path_x.size(); ++i) {
              double N = (target_dist/(0.02*vehicle_list.ref_val/2.24)); // Change mph to m/s
              double x_point = x_add_on+target_x/N;
              double y_point = s(x_point);
                
              x_add_on = x_point;
                
              double x_ref = x_point;
              double y_ref = y_point;
                
              //Rotate back to normal
              x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));
                
              x_point += ref_x;
              y_point += ref_y;
                
                
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
                
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } 
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}