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
        bool nextTurnFeasibleAt(const std::string& intersection_name);
        std::vector<std::string> currentTargetLabels(void);
        void publishTargetLabels(void);
        void loadNextScenario(void);//
        void updateLastNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_n);
        bool compareLastNodeAndCurrentNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name);
        void passageTypeCallback(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name);
        
        void turnFinish (bool change);
        void printStatus(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& passage_type);
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
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
        ros::Publisher target_labels_pub_;
        ros::Subscriber intersection_sub_;
        ros::Subscriber passage_type_sub_;
        ros::Subscriber stop_sub_;
        ros::Subscriber cmd_vel_sub_;
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
        int scenario_order_cnt_ = 1;
        int reach_target_type_cnt_ = 0;
        int reach_different_type_cnt_ = 0;
        int reach_target_type_cnt_margin_ = 6;
        int reach_different_type_cnt_margin_ = 6;
        // A real turn alone takes several seconds, so any apparent node
        // change/target match this soon after a step starts is almost
        // certainly a stale reading from just before the transition (or
        // noise), not genuine progress. passage_type is ignored entirely
        // until this many seconds of actual robot motion (see
        // cmdVelCallback() below, not wall-clock time) have elapsed since
        // the current step began (see loadNextScenario() and
        // passageTypeCallback()). Counting only moving time means a stop
        // for any reason -- e.g. corridor_classifier "collecting temporal
        // context" after a turn -- does not silently eat into the budget.
        double step_min_duration_sec_ = 8.0;
        double step_moving_duration_sec_ = 0.0;
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
    // Comma-separated intersection_name values that currently satisfy the
    // active scenario step's target (empty when there is none, e.g. mid-turn
    // or stopped). Lets a consumer such as corridor_classifier's direction
    // debouncer bypass its hold period the instant a matching label is
    // detected, instead of risking the target being missed or delayed.
    target_labels_pub_ = node_.advertise<std_msgs::String>("scenario_target_labels", 1);
    // Tracks actual robot motion so the step grace period above counts
    // only moving time, not wall-clock time.
    cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &cmdVelController::cmdVelCallback, this);
    // cmd_data.data.resize(3);

    // updateLastNode();
    getRosParam();
}

void cmdVelController::getRosParam(void){
    SCENARIO_MAX = 10;
    node_.getParam("scenario_executor/scenario_max", SCENARIO_MAX);
    step_min_duration_sec_ = 8.0;
    node_.getParam("scenario_executor/step_min_duration_sec", step_min_duration_sec_);
    moving_linear_threshold_ = 0.02;
    node_.getParam("scenario_executor/moving_linear_threshold", moving_linear_threshold_);
    moving_angular_threshold_ = 0.02;
    node_.getParam("scenario_executor/moving_angular_threshold", moving_angular_threshold_);
}

void cmdVelController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    ros::Time now = ros::Time::now();
    bool moving = std::abs(msg->linear.x) > moving_linear_threshold_ ||
                  std::abs(msg->angular.z) > moving_angular_threshold_;
    if(have_last_cmd_vel_stamp_ && moving){
        step_moving_duration_sec_ += (now - last_cmd_vel_stamp_).toSec();
    }
    last_cmd_vel_stamp_ = now;
    have_last_cmd_vel_stamp_ = true;
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

// Whether the scenario step immediately after the current one, if it is a
// turn, is actually possible at this intersection type. Steps that are not
// followed by a turn (or that are the last step) are always feasible here.
bool cmdVelController::nextTurnFeasibleAt(const std::string& intersection_name){
    int next_step = scenario_progress_cnt_ + 1;
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

// The set of intersection_name values that currently satisfy the active
// scenario step's target (its target_type/target_direction), after also
// filtering out anything whose next-step turn (if any) is not actually
// possible there. Empty when the current step has no type to wait for
// (e.g. a turn or stop step). This is the single source of truth behind
// both compareScenarioAndPassageType() and the published
// scenario_target_labels topic, so they can never disagree.
std::vector<std::string> cmdVelController::currentTargetLabels(void){
    std::vector<std::string> labels;
    if(scenario_num_ == 0 || scenario_progress_cnt_ >= scenario_num_){
        return labels;
    }
    std::string target_type = *std::next(target_type_itr_begin_, scenario_progress_cnt_);
    std::string target_direction = *std::next(target_direction_itr_begin_, scenario_progress_cnt_);

    if(target_type == "straight_road"){
        labels.push_back("straight_road");
    }
    else if(target_type == "3_way"){
        labels = {"3_way_right","3_way_center","3_way_left","corner_right","corner_left"};
    }
    else if(target_type == "end"){
        for(const std::string& candidate : {"3_way_center","corner_left","corner_right","dead_end"}){
            if(nextTurnFeasibleAt(candidate)) labels.push_back(candidate);
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
                if(nextTurnFeasibleAt(candidate)) labels.push_back(candidate);
            }
        }
    }
    return labels;
}

bool cmdVelController::compareScenarioAndPassageType(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& passage_type){//シナリオと現在のノードの比較
    std::vector<std::string> labels = currentTargetLabels();
    return std::find(labels.begin(), labels.end(), passage_type->intersection_name) != labels.end();
}

void cmdVelController::publishTargetLabels(void){
    std::vector<std::string> labels = currentTargetLabels();
    std_msgs::String message;
    for(size_t i = 0; i < labels.size(); ++i){
        if(i > 0) message.data += ",";
        message.data += labels[i];
    }
    target_labels_pub_.publish(message);
}

void cmdVelController::loadNextScenario(void){
    // Reset the moving-time budget for the new step. Drop the pending
    // cmd_vel interval too, so time spent between the last cmd_vel message
    // and this step's start is never attributed to the new step.
    step_moving_duration_sec_ = 0.0;
    have_last_cmd_vel_stamp_ = false;
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
    }
    else{
        ROS_INFO("Execute next action(%s)", action.c_str());//string <=> char
        stop_flg_ = false;
        change_node_flg_ = false;
        
        if(action.find("turn") != std::string::npos){//turn find
            turn_flg_ = true;//曲がる動作を開始

            //if(action.find("left")){
            if(action == "turn_left"){
                std::copy(std::begin(left_list),std::end(left_list),std::begin(cmd_data.cmd_dir));
            
            }
            //else if(action.find("right")){
            else if(action == "turn_right"){
                std::copy(std::begin(right_list),std::end(right_list),std::begin(cmd_data.cmd_dir));
                
            }
        }
        else{
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
        publishTargetLabels();
        printStatus(passage_type);
        return;
    }
    if(step_moving_duration_sec_ < step_min_duration_sec_){
        // A real turn alone takes several seconds, so any apparent node
        // change or target match this soon after the current step began
        // is almost certainly a stale reading from just before the
        // transition (or noise), not genuine progress. Ignore passage_type
        // entirely -- do not touch last_node_/change_node_flg_/turn_flg_
        // state or compare against the target -- until this grace period
        // has elapsed, so a step can never advance in an instant.
        cmd_data_pub.publish(cmd_data);
        publishTargetLabels();
        printStatus(passage_type);
        return;
    }
    if(! stop_flg_){
        if(request_update_last_node_flg){
            updateLastNode(passage_type);
            request_update_last_node_flg = false;
        }
    }
        if(! turn_flg_){//actionがturnではない場合
            if(change_node_flg_){//道タイプが変更されている場合
                satisfy_conditions_flg_ = compareScenarioAndPassageType(passage_type);
                if(satisfy_conditions_flg_){
                    ROS_INFO("find target node !! ");
                    scenario_order_cnt_++;
                    change_node_flg_ = false;
                    updateLastNode(passage_type);
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
        else { //turn_flg_on　action中は継続
            if (!compareLastNodeAndCurrentNode(passage_type))
            {
                turnFinish(true);  //change_flg on ターン終了まで継続
            } 
        }
    if (passage_type->intersection_name == "dead_end") {
        ROS_INFO("Dead end detected. Stopping robot.");
        std_msgs::Bool stop_flg_for_pub;
        stop_flg_for_pub.data = stop_flg_;
        stop_pub_.publish(stop_flg_for_pub);
        std::copy(std::begin(stop_list),std::end(stop_list),std::begin(cmd_data.cmd_dir));
    }
    else if (!stop_flg_) {
        // The block above only ever latches cmd_dir to stop_list; nothing
        // else restores it. If an earlier (possibly spurious, single-frame)
        // dead_end reading set it and the scenario has not since advanced
        // to a new loadNextScenario() call, cmd_dir would otherwise stay
        // stuck at [0, 0, 0] forever even once the passage type moves on.
        // Re-derive it from the currently loaded action whenever we are not
        // actually looking at a dead end.
        if (turn_flg_) {
            if (action == "turn_left") {
                std::copy(std::begin(left_list),std::end(left_list),std::begin(cmd_data.cmd_dir));
            }
            else if (action == "turn_right") {
                std::copy(std::begin(right_list),std::end(right_list),std::begin(cmd_data.cmd_dir));
            }
        }
        else {
            std::copy(std::begin(str_list),std::end(str_list),std::begin(cmd_data.cmd_dir));
        }
    }
    cmd_data_pub.publish(cmd_data);//
    publishTargetLabels();
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

    bool in_grace_period = step_moving_duration_sec_ < step_min_duration_sec_;

    std::string mode = in_grace_period
                       ? "STEP_GRACE_PERIOD"
                       : turn_flg_ ? "TURNING"
                       : stop_flg_ ? "STOPPED"
                       : change_node_flg_ ? "AT_NEW_NODE(comparing)"
                       : "WAITING_FOR_NODE_CHANGE";

    ROS_INFO(
        "[STATUS] step=%d/%d action=%s target_type=%s target_direction=%s "
        "order=%d/%d mode=%s step_moving_time=%.1f/%.1fs | current_node=%s last_node=%s",
        step + 1, scenario_num_,
        step_action.c_str(), step_type.c_str(), step_direction.c_str(),
        scenario_order_cnt_, step_order,
        mode.c_str(), step_moving_duration_sec_, step_min_duration_sec_,
        passage_type->intersection_name.c_str(), last_node_.data.c_str()
    );
}

void cmdVelController::turnFinish(bool change){
    ROS_INFO("finish turn");
    turn_flg_ = false;
    scenario_progress_cnt_++;
    loadNextScenario();
    request_update_last_node_flg = change;
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

