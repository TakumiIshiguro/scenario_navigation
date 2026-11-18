#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int8MultiArray.h>
#include "geometry_msgs/Twist.h"
#include "scenario_navigation/Scenario.h"
#include "scenario_navigation/PassageType.h"
#include "scenario_navigation_msgs/cmd_dir_intersection.h"
#include <std_srvs/SetBool.h>
#include <unistd.h>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>

#include <iostream>

class cmdVelController {
     public:
        cmdVelController();
        int SCENARIO_MAX = 10;
        void getRosParam(void);
       
        // <intersection>
        // bool compareScenarioAndPassageType(const std_msgs::String::ConstPtr& intersection_name);
        // void loadNextScenario(void);//
        // void updateLastNode(const std_msgs::String::ConstPtr& intersection_n);
        // bool compareLastNodeAndCurrentNode(const std_msgs::String::ConstPtr& intersection_name);
        // void passageTypeCallback(const std_msgs::String::ConstPtr& intersection_name);
        bool compareScenarioAndPassageType(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name);
        bool directionAvailable(const std::string& intersection_name, const std::string& direction);
        bool frontOpen(const std::string& intersection_name);
        bool nextTurnFeasibleAtStep(int step, const std::string& intersection_name);
        bool nextTurnFeasibleAt(const std::string& intersection_name);
        std::vector<std::string> targetLabelsForStep(int step);
        std::vector<std::string> currentTargetLabels(void);
        void startTargetGuard(bool after_turn);
        void loadNextScenario(void);//
        void updateLastNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_n);
        bool compareLastNodeAndCurrentNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name);
        void passageTypeCallback(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name);
        
        void turnFinish (bool change);
        void printStatus(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& passage_type);
        void publishStatus(const std::string& mode, const std::string& current_node);
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void careAvoidanceCallback(const std_msgs::Bool::ConstPtr& msg);
        bool careAvoidanceActive(void);
        void stopCallback(const std_msgs::Bool::ConstPtr& stop); //
        bool scenarioCallback(scenario_navigation::Scenario::Request& scenario,
                              scenario_navigation::Scenario::Response& res);
        bool nextscenarioCallback(std_srvs::SetBool::Request& next_req ,
                                  std_srvs::SetBool::Response& next_resp);
     private:
        ros::NodeHandle node_;
       

        // <intersection>
        ros::Publisher stop_pub_;
        ros::Publisher cmd_data_pub;
        ros::Publisher status_pub_;
        ros::Subscriber intersection_sub_;
        ros::Subscriber passage_type_sub_;
        ros::Subscriber stop_sub_;
        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber care_avoidance_sub_;
        ros::ServiceServer scenario_server_,next_scenario_srv_;


        std::list<std::string> target_type_;
        std::list<std::int16_t> target_order_;
        std::list<std::string> target_direction_;
        std::list<std::string> target_action_;

        std::list<std::string>::iterator target_type_itr_begin_;
        // std_msgs::Int8MultiArray cmd_data;
        scenario_navigation_msgs::cmd_dir_intersection cmd_data;
        std::list<std::int16_t>::iterator target_order_itr_begin_;
        std::list<std::string>::iterator target_direction_itr_begin_;
        std::list<std::string>::iterator target_action_itr_begin_;

        std::string action;
        int scenario_num_ = 0;
        int scenario_progress_cnt_ = 0;
        // Count of matching landmarks passed so far on the current step
        // (incremented, then compared against the step's order, in
        // passageTypeCallback()). Starts at 0 -- no landmark passed yet --
        // so an order-N step completes at exactly the Nth matching
        // landmark. Reset to 0 on every step change (see the goal-reached
        // branch and nextscenarioCallback()); the initializer here is what
        // the first step of a scenario uses.
        int scenario_order_cnt_ = 0;
        // How far the robot has been commanded to travel (see cmdVelCallback()
        // below) since the current step began (reset in loadNextScenario()).
        double step_moving_distance_m_ = 0.0;
        // One junction reads as junction -> straight_road -> junction, or as
        // 3_way_left -> 3_way_right, when the classifier flickers, and each
        // of those looks like a new node. So nothing counts as a target
        // until the robot has travelled target_guard_distance_m_ from the
        // last counted target or the end of the last turn. This also covers
        // the junction a turn leaves behind, whose label is still showing
        // when the turn ends.
        //
        // After a count, the label showing when the guard ends is the node
        // already counted. After a turn nothing has been counted yet, so
        // that label is compared against the target: a dead end already in
        // view would otherwise never read as a change.
        double target_guard_distance_m_ = 1.0;
        double target_guard_travelled_m_ = 0.0;
        bool target_guard_active_ = false;
        bool target_guard_after_turn_ = false;
        // A turn ends once /cmd_vel has rotated this far toward the commanded
        // side, less any rotation back the other way. CARE's rotation counts
        // too, since it changes the heading all the same. A corridor label
        // ending it instead left the heading a few degrees short, and pure
        // pursuit does not correct that on the straight that follows.
        double turning_max_angle_deg_ = 90.0;
        // Within +-this of turning_max_angle_deg_, a corridor label with an
        // open front ends the turn early (from max - window), and the turn
        // is forced to end only at max + window. Zero keeps the angle alone.
        // 5 deg from Gazebo runs: wider windows let a label end the turn at
        // 76-87 deg, and the under-rotated robot then met the next wall.
        double turning_label_window_deg_ = 5.0;
        // Fallback for a turn that never rotates that far. 30 s: at 60 s a turn
        // whose end was missed kept the command through the whole next corridor
        // and took the following corner too (158 deg in Gazebo); at 20 s a slow,
        // wide turn that CARE pushed back off a wall was cut at 51 deg.
        double turning_timeout_sec_ = 30.0;
        double turn_active_angle_rad_ = 0.0;
        double turn_direction_ = 0.0; // +1 for a left turn, -1 for a right turn.
        double turn_active_duration_sec_ = 0.0;
        // vnm_ros/CARE's obstacle-avoidance-active flag (std_msgs/Bool, e.g.
        // /vnm/care_avoidance_active). /cmd_vel is the final velocity, so it
        // also carries CARE's repulsive steering -- which can spin the robot
        // before the commanded scenario turn has really begun. Disabled
        // (no subscriber) when scenario_executor/care_avoidance_topic is
        // empty; harmless when nothing publishes it (a stale reading older
        // than care_avoidance_stale_timeout_sec_ is ignored, so a dropped
        // "true" reads as inactive).
        bool care_avoidance_active_ = false;
        bool have_care_avoidance_ = false;
        ros::Time care_avoidance_stamp_;
        double care_avoidance_stale_timeout_sec_ = 1.0;
        ros::Time last_cmd_vel_stamp_;
        bool have_last_cmd_vel_stamp_ = false;
        double moving_linear_threshold_ = 0.02;
        double moving_angular_threshold_ = 0.02;
        float rotate_rad_ = 0.0;
        bool turn_flg_ = false;
        bool change_node_flg_ = false;
        bool satisfy_conditions_flg_ = false;
        bool stop_flg_ = true;
        bool request_update_last_node_flg = true;
        std_msgs::String last_node_;
        // std::string node_name;
        int str_list[3] = {1,0,0};
        int left_list[3] = {0,1,0};
        int right_list[3] = {0,0,1};
        int stop_list[3] = {0,0,0};
};

cmdVelController::cmdVelController(){
    stop_pub_ = node_.advertise<std_msgs::Bool>("stop", 1, false);
    //passage_type_sub_ = node_.subscribe<std_msgs::String> ("passage_type", 1, &cmdVelController::passageTypeCallback, this); //intersection name
    passage_type_sub_ = node_.subscribe<scenario_navigation_msgs::cmd_dir_intersection>("passage_type", 1, &cmdVelController::passageTypeCallback, this); //intersection name
    stop_sub_ = node_.subscribe<std_msgs::Bool> ("stop", 1, &cmdVelController::stopCallback, this);
    scenario_server_ = node_.advertiseService("scenario", &cmdVelController::scenarioCallback, this);
    next_scenario_srv_ = node_.advertiseService("next_scenario", &cmdVelController::nextscenarioCallback, this);
    // cmd_data_pub = node_.advertise<std_msgs::Int8MultiArray >("cmd_dir", 1);
    cmd_data_pub = node_.advertise<scenario_navigation_msgs::cmd_dir_intersection>("cmd_dir_intersection",1);
    // The [STATUS] line as JSON, for scripts/scenario_overlay.py. Latched so
    // the final COMPLETE state survives this node shutting down.
    status_pub_ = node_.advertise<std_msgs::String>("scenario_status", 1, true);
    // Tracks commanded robot motion so the step distance above counts
    // only commanded travel, not wall-clock time.
    cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &cmdVelController::cmdVelCallback, this);
    // cmd_data.data.resize(3);

    // updateLastNode();
    getRosParam();
}

void cmdVelController::getRosParam(void){
    SCENARIO_MAX = 10;
    node_.getParam("scenario_executor/scenario_max", SCENARIO_MAX);
    moving_linear_threshold_ = 0.02;
    node_.getParam("scenario_executor/moving_linear_threshold", moving_linear_threshold_);
    moving_angular_threshold_ = 0.02;
    node_.getParam("scenario_executor/moving_angular_threshold", moving_angular_threshold_);
    target_guard_distance_m_ = 1.0;
    node_.getParam("scenario_executor/target_guard_distance_m", target_guard_distance_m_);
    turning_max_angle_deg_ = 90.0;
    node_.getParam("scenario_executor/turning_max_angle_deg", turning_max_angle_deg_);
    turning_label_window_deg_ = 5.0;
    node_.getParam("scenario_executor/turning_label_window_deg", turning_label_window_deg_);
    turning_timeout_sec_ = 30.0;
    node_.getParam("scenario_executor/turning_timeout_sec", turning_timeout_sec_);
    care_avoidance_stale_timeout_sec_ = 1.0;
    node_.getParam("scenario_executor/care_avoidance_stale_timeout_sec", care_avoidance_stale_timeout_sec_);
    // Defaults to vnm_ros/CARE's avoidance-active topic; harmless when CARE
    // is not running (no publisher -> careAvoidanceActive() stays false and
    // the turn-start latch uses cmd_vel alone). Set the param to "" to
    // disable the subscriber entirely.
    std::string care_avoidance_topic = "/vnm/care_avoidance_active";
    node_.getParam("scenario_executor/care_avoidance_topic", care_avoidance_topic);
    if(! care_avoidance_topic.empty()){
        care_avoidance_sub_ = node_.subscribe<std_msgs::Bool>(
            care_avoidance_topic, 1, &cmdVelController::careAvoidanceCallback, this);
    }
}

void cmdVelController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    ros::Time now = ros::Time::now();
    bool moving = std::abs(msg->linear.x) > moving_linear_threshold_ ||
                  std::abs(msg->angular.z) > moving_angular_threshold_;
    if(have_last_cmd_vel_stamp_ && moving){
        double dt = (now - last_cmd_vel_stamp_).toSec();
        double distance = std::abs(msg->linear.x) * dt;
        step_moving_distance_m_ += distance;
        target_guard_travelled_m_ += distance;
        if(turn_flg_){
            turn_active_angle_rad_ += turn_direction_ * msg->angular.z * dt;
            turn_active_duration_sec_ += dt;
        }
    }
    last_cmd_vel_stamp_ = now;
    have_last_cmd_vel_stamp_ = true;
}

void cmdVelController::careAvoidanceCallback(const std_msgs::Bool::ConstPtr& msg){
    care_avoidance_active_ = msg->data;
    have_care_avoidance_ = true;
    care_avoidance_stamp_ = ros::Time::now();
}

// True only while a fresh reading says CARE is actively steering around an
// obstacle. No subscriber / no message yet / stale message -> false, so
// this never suppresses turn-start detection unless CARE is genuinely,
// currently active.
bool cmdVelController::careAvoidanceActive(void){
    if(! have_care_avoidance_){
        return false;
    }
    if((ros::Time::now() - care_avoidance_stamp_).toSec() > care_avoidance_stale_timeout_sec_){
        return false;
    }
    return care_avoidance_active_;
}
//<intersection>

// Whether a passage of the given intersection type actually has an exit in
// the given direction (left/right). Used so that, e.g., a scenario step
// "go straight until end, then turn left" does not treat a corner_right
// (which has no left branch at all) as satisfying "end".
bool cmdVelController::directionAvailable(const std::string& intersection_name, const std::string& direction){
    if(direction == "left"){
        return intersection_name == "corner_left" ||
               intersection_name == "3_way_left" ||
               intersection_name == "3_way_center" ||
               intersection_name == "cross_road";
    }
    if(direction == "right"){
        return intersection_name == "corner_right" ||
               intersection_name == "3_way_right" ||
               intersection_name == "3_way_center" ||
               intersection_name == "cross_road";
    }
    return true;
}

// Whether the label shows a corridor straight ahead. "turning" and the
// labels without a front exit (dead_end, corners, 3_way_center) do not.
bool cmdVelController::frontOpen(const std::string& intersection_name){
    return intersection_name == "straight_road" ||
           intersection_name == "3_way_left" ||
           intersection_name == "3_way_right" ||
           intersection_name == "cross_road";
}

// Whether the scenario step immediately after the given one, if it is a
// turn, is actually possible at this intersection type. Steps that are not
// followed by a turn (or that are the last step) are always feasible here.
bool cmdVelController::nextTurnFeasibleAtStep(int step, const std::string& intersection_name){
    int next_step = step + 1;
    if(next_step >= scenario_num_){
        return true;
    }
    std::string next_action = *std::next(target_action_itr_begin_, next_step);
    if(next_action == "turn_left"){
        return directionAvailable(intersection_name, "left");
    }
    if(next_action == "turn_right"){
        return directionAvailable(intersection_name, "right");
    }
    return true;
}

bool cmdVelController::nextTurnFeasibleAt(const std::string& intersection_name){
    return nextTurnFeasibleAtStep(scenario_progress_cnt_, intersection_name);
}

// The set of intersection_name values that satisfy the given step's target
// (its target_type/target_direction), after also filtering out anything
// whose next-step turn (if any) is not actually possible there. Empty when
// that step has no type to wait for (e.g. a turn or stop step).
std::vector<std::string> cmdVelController::targetLabelsForStep(int step){
    std::vector<std::string> labels;
    if(scenario_num_ == 0 || step < 0 || step >= scenario_num_){
        return labels;
    }
    std::string target_type = *std::next(target_type_itr_begin_, step);
    std::string target_direction = *std::next(target_direction_itr_begin_, step);

    if(target_type == "straight_road"){
        labels.push_back("straight_road");
    }
    else if(target_type == "3_way"){
        labels = {"3_way_right","3_way_center","3_way_left"};
    }
    else if(target_type == "end"){
        for(const std::string& candidate : {"3_way_center","corner_left","corner_right","dead_end"}){
            if(nextTurnFeasibleAtStep(step, candidate)) labels.push_back(candidate);
        }
    }
    else if(target_type == "corner" || target_type == "corridor"){
        if(target_direction == "left"){
            labels = {"3_way_center","3_way_left","corner_left"};
        }
        else if(target_direction == "right"){
            labels = {"3_way_center","3_way_right","corner_right"};
        }
        else{
            // No direction word in the sentence (e.g. "次の角まで直進"):
            // accept a corner or 3-way node regardless of which way it
            // turns, instead of never matching anything -- but still only
            // if the next step's turn (if any) is actually possible there.
            for(const std::string& candidate :
                {"corner_left","corner_right","3_way_left","3_way_right","3_way_center"}){
                if(nextTurnFeasibleAtStep(step, candidate)) labels.push_back(candidate);
            }
        }
    }
    return labels;
}

// The set of intersection_name values that currently satisfy the active
// scenario step's target. This is the single source of truth behind
// compareScenarioAndPassageType(), which only ever runs while turn_flg_ is
// false (see passageTypeCallback()), so it is always this step, never a
// lookahead.
std::vector<std::string> cmdVelController::currentTargetLabels(void){
    return targetLabelsForStep(scenario_progress_cnt_);
}

bool cmdVelController::compareScenarioAndPassageType(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& passage_type){//シナリオと現在のノードの比較
    std::vector<std::string> labels = currentTargetLabels();
    return std::find(labels.begin(), labels.end(), passage_type->intersection_name) != labels.end();
}

void cmdVelController::startTargetGuard(bool after_turn){
    target_guard_active_ = target_guard_distance_m_ > 0.0;
    target_guard_after_turn_ = after_turn;
    target_guard_travelled_m_ = 0.0;
}

void cmdVelController::loadNextScenario(void){
    // Reset the step's travelled distance. Drop the pending
    // cmd_vel interval too, so time spent between the last cmd_vel message
    // and this step's start is never attributed to the new step.
    step_moving_distance_m_ = 0.0;
    have_last_cmd_vel_stamp_ = false;
    turn_active_angle_rad_ = 0.0;
    turn_active_duration_sec_ = 0.0;
    // std::string action = *std::next(target_action_itr_begin_, scenario_progress_cnt_);
    action = *std::next(target_action_itr_begin_, scenario_progress_cnt_);

// stop robot
    stop_flg_ = true;
    if(action == "stop"){
        ROS_INFO("Robot gets a goal");
        std_msgs::Bool stop_flg_for_pub;
        stop_flg_for_pub.data = stop_flg_;
        stop_pub_.publish(stop_flg_for_pub);
        std::copy(std::begin(stop_list),std::end(stop_list),std::begin(cmd_data.cmd_dir));
        cmd_data_pub.publish(cmd_data);
        publishStatus("COMPLETE", last_node_.data);
        ros::Duration(0.2).sleep();  // let the final status go out
        // "stop" is always the scenario's final step (see
        // scenario_parser_jsk.py), so reaching it means the destination has
        // been reached. Shut the node down instead of idling forever.
        ROS_INFO("Scenario complete. Shutting down.");
        ros::shutdown();
    }
    else{
        ROS_INFO("Execute next action(%s)", action.c_str());//string <=> char
        stop_flg_ = false;
        change_node_flg_ = false;

        if(action.find("turn") != std::string::npos){//turn find
            turn_flg_ = true;//曲がる動作を開始

            //if(action.find("left")){
            if(action == "turn_left"){
                turn_direction_ = 1.0;
                std::copy(std::begin(left_list),std::end(left_list),std::begin(cmd_data.cmd_dir));
            }
            //else if(action.find("right")){
            else if(action == "turn_right"){
                turn_direction_ = -1.0;
                std::copy(std::begin(right_list),std::end(right_list),std::begin(cmd_data.cmd_dir));
            }
        }
        else{
            // Switch to straight immediately, even if the robot is still
            // physically rotating out of the turn that just completed.
            // turnFinish() only fires once /passage_type has reported the
            // destination corridor (so the turn is visually all but done)
            // *and* the minimum-turn-time floor has passed, so dropping the
            // turn command right here is exactly what stops the
            // over-rotation: the robot then holds its heading down the new
            // corridor instead of being told to keep turning until cmd_vel
            // happens to settle on its own.
            std::copy(std::begin(str_list),std::end(str_list),std::begin(cmd_data.cmd_dir));
        }
    }
}

// <intersection>
// void cmdVelController::updateLastNode(const std_msgs::String::ConstPtr& intersection_n){
void cmdVelController::updateLastNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_n){
    ROS_INFO("update last node");
    last_node_.data = intersection_n->intersection_name;
    // node_name = last_node_.data;
    ROS_INFO("last node is (%s)",intersection_n->intersection_name.c_str());
 
    
}

// bool cmdVelController::compareLastNodeAndCurrentNode(const std_msgs::String::ConstPtr& intersection_name){//前回と今回のノードの比較
bool cmdVelController::compareLastNodeAndCurrentNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name){//前回と今回のノードの比較
    if(last_node_.data == intersection_name->intersection_name){
            return true;
        }
    else{
        return false;
    }
 }

// void cmdVelController::passageTypeCallback(const std_msgs::String::ConstPtr& passage_type){//メイン処理部
void cmdVelController::passageTypeCallback(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& passage_type){//メイン処理部
//std::copy(std::begin(left_list),std::end(left_list),std::begin(cmd_data.data));
    // Checked before the "turning" early return below, since rotating is
    // when "turning" is published.
    double turn_angle_deg = turn_active_angle_rad_ * 180.0 / M_PI;
    double turn_window_deg = std::max(0.0, turning_label_window_deg_);
    // Inside the window, a label with a corridor ahead means the robot
    // already faces down the new corridor, so the label ends the turn.
    bool label_ends_turn =
        turn_window_deg > 0.0 && turning_max_angle_deg_ > 0.0 &&
        turn_angle_deg >= turning_max_angle_deg_ - turn_window_deg &&
        frontOpen(passage_type->intersection_name);
    bool angle_ends_turn =
        turning_max_angle_deg_ > 0.0 &&
        turn_angle_deg >= turning_max_angle_deg_ + turn_window_deg;
    if(turn_flg_ &&
       (label_ends_turn || angle_ends_turn ||
        (turning_timeout_sec_ > 0.0 &&
         turn_active_duration_sec_ >= turning_timeout_sec_))){
        ROS_INFO("turn ended at %.1f/%.1f+-%.1f deg in %.1f/%.1f s by %s (%s)",
                 turn_angle_deg, turning_max_angle_deg_, turn_window_deg,
                 turn_active_duration_sec_, turning_timeout_sec_,
                 label_ends_turn ? "label" : angle_ends_turn ? "angle" : "timeout",
                 passage_type->intersection_name.c_str());
        turnFinish(true);
        cmd_data_pub.publish(cmd_data);
        printStatus(passage_type);
        return;
    }
    if(passage_type->intersection_name == "turning"){
        // corridor_classifier reports "turning" independently of this
        // executor's own turn_flg_ state (it is driven by /cmd_vel, not by
        // which scenario step is loaded). It is not a real corridor shape,
        // so it must never be treated as "the node changed": doing so
        // used to make turnFinish() fire the instant a turn *started*
        // (since it looked like a brand new node), skipping straight to
        // the next scenario step while the robot was still mid-turn.
        // Just keep publishing whatever cmd_dir the current step already
        // loaded and wait for a real (non-turning) passage type.
        cmd_data_pub.publish(cmd_data);
        printStatus(passage_type);
        return;
    }
    if(! stop_flg_){
        if(request_update_last_node_flg){
            updateLastNode(passage_type);
            request_update_last_node_flg = false;
            // A target already showing at the first reading would never
            // read as a change from itself, so compare it directly.
            if(! turn_flg_ && compareScenarioAndPassageType(passage_type)){
                change_node_flg_ = true;
            }
        }
    }
        if(! turn_flg_){//actionがturnではない場合
            if(target_guard_active_ &&
               target_guard_travelled_m_ >= target_guard_distance_m_){
                target_guard_active_ = false;
                updateLastNode(passage_type);
                change_node_flg_ = target_guard_after_turn_;
            }
            if(target_guard_active_){
                ROS_INFO("Target guard %.2f/%.2f m",
                         target_guard_travelled_m_, target_guard_distance_m_);
            }
            else if(change_node_flg_){//道タイプが変更されている場合
                satisfy_conditions_flg_ = compareScenarioAndPassageType(passage_type);
                if(satisfy_conditions_flg_){
                    ROS_INFO("find target node !! ");
                    scenario_order_cnt_++;
                    change_node_flg_ = false;
                    updateLastNode(passage_type);
                    startTargetGuard(false);
                    int order = *std::next(target_order_itr_begin_, scenario_progress_cnt_);
                    if(order <= scenario_order_cnt_){
                        ROS_INFO("Robot reaches target_node!!");
                        scenario_order_cnt_ = 0;
                        scenario_progress_cnt_++;
                        loadNextScenario();
                    }
                }
                else{
                    ROS_INFO("Not the goal of the scenario !!");
                }
            }
            else{
                if(!compareLastNodeAndCurrentNode(passage_type)){
                    updateLastNode(passage_type);
                    change_node_flg_ = true;
                    ROS_INFO("Node changed!!");
                }
                else{
                    ROS_INFO("Same node as last time !! ");
                }
            }
        }
    cmd_data_pub.publish(cmd_data);//
    printStatus(passage_type);
}

// Print the executor's current scenario-progress state: which step we are
// on, what that step is waiting for, and the flags driving the state
// machine above, so the current situation can be read directly from stdout
// instead of inferred from the scattered ROS_INFO lines.
void cmdVelController::printStatus(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& passage_type){
    if(scenario_num_ == 0){
        ROS_INFO("[STATUS] no scenario loaded yet");
        return;
    }
    int step = scenario_progress_cnt_;
    if(step >= scenario_num_){
        ROS_INFO("[STATUS] scenario finished (step %d/%d)", scenario_num_, scenario_num_);
        return;
    }
    std::string step_type = *std::next(target_type_itr_begin_, step);
    int step_order = *std::next(target_order_itr_begin_, step);
    std::string step_direction = *std::next(target_direction_itr_begin_, step);
    std::string step_action = *std::next(target_action_itr_begin_, step);

    std::string mode = turn_flg_ ? "TURNING"
                       : stop_flg_ ? "STOPPED"
                       : target_guard_active_ ? "TARGET_GUARD"
                       : change_node_flg_ ? "AT_NEW_NODE(comparing)"
                       : "WAITING_FOR_NODE_CHANGE";

    ROS_INFO(
        "[STATUS] step=%d/%d action=%s target_type=%s target_direction=%s "
        "order=%d/%d mode=%s turn_angle=%.1f/%.1fdeg(care=%d) "
        "step_moving_dist=%.2fm target_guard_dist=%.2f/%.2fm "
        "| current_node=%s last_node=%s",
        step + 1, scenario_num_,
        step_action.c_str(), step_type.c_str(), step_direction.c_str(),
        scenario_order_cnt_, step_order,
        mode.c_str(),
        turn_active_angle_rad_ * 180.0 / M_PI, turning_max_angle_deg_,
        (int)careAvoidanceActive(),
        step_moving_distance_m_,
        target_guard_travelled_m_, target_guard_distance_m_,
        passage_type->intersection_name.c_str(), last_node_.data.c_str()
    );
    publishStatus(mode, passage_type->intersection_name);
}

// All values come from the scenario parser and the executor's own labels,
// which contain nothing that needs JSON escaping.
void cmdVelController::publishStatus(const std::string& mode, const std::string& current_node){
    std::ostringstream json;
    json << "{\"step\":" << scenario_progress_cnt_
         << ",\"mode\":\"" << mode << "\""
         << ",\"current_node\":\"" << current_node << "\""
         << ",\"last_node\":\"" << last_node_.data << "\""
         << ",\"order\":" << scenario_order_cnt_
         << ",\"care_active\":" << (careAvoidanceActive() ? "true" : "false")
         << ",\"turn_angle_deg\":" << turn_active_angle_rad_ * 180.0 / M_PI
         << ",\"turn_max_angle_deg\":" << turning_max_angle_deg_
         << ",\"step_distance_m\":" << step_moving_distance_m_
         << ",\"target_guard\":" << (target_guard_active_ ? "true" : "false")
         << ",\"target_guard_travelled_m\":" << target_guard_travelled_m_
         << ",\"target_guard_distance_m\":" << target_guard_distance_m_;
    json << ",\"target_labels\":[";
    std::vector<std::string> labels = targetLabelsForStep(scenario_progress_cnt_);
    for(size_t i = 0; i < labels.size(); ++i){
        json << (i > 0 ? "," : "") << "\"" << labels[i] << "\"";
    }
    json << "],\"steps\":[";
    for(int i = 0; i < scenario_num_; ++i){
        json << (i > 0 ? "," : "")
             << "{\"action\":\"" << *std::next(target_action_itr_begin_, i) << "\""
             << ",\"type\":\"" << *std::next(target_type_itr_begin_, i) << "\""
             << ",\"direction\":\"" << *std::next(target_direction_itr_begin_, i) << "\""
             << ",\"order\":" << *std::next(target_order_itr_begin_, i) << "}";
    }
    json << "]}";
    std_msgs::String message;
    message.data = json.str();
    status_pub_.publish(message);
}

void cmdVelController::turnFinish(bool change){
    ROS_INFO("finish turn");
    turn_flg_ = false;
    scenario_progress_cnt_++;
    loadNextScenario();
    if(change){
        startTargetGuard(true);
    }
    change_node_flg_ =false;
}

void cmdVelController::stopCallback(const std_msgs::Bool::ConstPtr& stop_flg){
    stop_flg_ = stop_flg->data;
}

bool cmdVelController::scenarioCallback(scenario_navigation::Scenario::Request& scenario, //一括読み込み
                                        scenario_navigation::Scenario::Response& res){
    scenario_num_++;
    target_type_.push_back(scenario.type);
    target_order_.push_back(scenario.order);
    target_direction_.push_back(scenario.direction);
    target_action_.push_back(scenario.action);
    std::string last_action = *std::next(target_action_.begin(), scenario_num_ - 1);//先頭を取得  scenario_num方向へすすめる

// check whether scenario is loaded
    if(last_action == "stop"){
        ROS_INFO("Completed loading scenario");
        //先頭のイテレータ取得
        target_type_itr_begin_ = target_type_.begin();
        target_order_itr_begin_ = target_order_.begin();
        target_direction_itr_begin_ = target_direction_.begin();
        target_action_itr_begin_ = target_action_.begin();

        stop_flg_ = false;
        std_msgs::Bool stop_flg_for_pub;
        stop_flg_for_pub.data = stop_flg_;
        stop_pub_.publish(stop_flg_for_pub);
        loadNextScenario();
    }
    else{
        stop_flg_ = true;
        std_msgs::Bool stop_flg_for_pub;
        stop_flg_for_pub.data = stop_flg_;
        stop_pub_.publish(stop_flg_for_pub);
    }


//  debug
    std::cout << "####################################" << std::endl;
    std::cout << "type is " << scenario.type << std::endl;
    std::cout << "order is "  << std::hex << scenario.order << std::endl;
    std::cout << "direction is " << scenario.direction << std::endl;
    std::cout << "action is " << scenario.action << std::endl;
    std::cout << "####################################" << std::endl;

    return true;
}
bool cmdVelController::nextscenarioCallback(std_srvs::SetBool::Request& next_req,
                              std_srvs::SetBool::Response& next_res){
    if(next_req.data){
        ROS_INFO("Next scenario");
        scenario_order_cnt_ = 0;
        scenario_progress_cnt_++;
        loadNextScenario();
    }
    else
        ROS_INFO("Please send true data");
}
    
int main(int argc, char** argv){
    ros::init(argc, argv, "scenario_executor");
    cmdVelController cmd_vel_controller;
    while(ros::ok()){
        ros::spin();
    }

    return 0;
}

