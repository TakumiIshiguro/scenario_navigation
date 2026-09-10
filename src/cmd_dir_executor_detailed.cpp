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
        bool nextTurnFeasibleAtStep(int step, const std::string& intersection_name);
        bool nextTurnFeasibleAt(const std::string& intersection_name);
        std::vector<std::string> targetLabelsForStep(int step);
        std::vector<std::string> currentTargetLabels(void);
        std::vector<std::string> activeTargetLabels(void);
        bool atDeadEndStopTail(void);
        void publishTargetLabels(void);
        void loadNextScenario(void);//
        void updateLastNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_n);
        bool compareLastNodeAndCurrentNode(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name);
        void passageTypeCallback(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& intersection_name);
        
        void turnFinish (bool change);
        void printStatus(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& passage_type);
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
        ros::Publisher target_labels_pub_;
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
        int reach_target_type_cnt_ = 0;
        int reach_different_type_cnt_ = 0;
        int reach_target_type_cnt_margin_ = 6;
        int reach_different_type_cnt_margin_ = 6;
        // Consecutive "dead_end" /passage_type readings. A sustained
        // dead_end means the robot is driving at a wall (a wrong turn, or
        // the corridor really ends), so once this reaches
        // dead_end_stop_streak_ the robot is held stopped (see the top of
        // passageTypeCallback()). Counted consecutively (any non-dead_end
        // reading resets it), so a stray dead_end frame in a turn's label
        // sweep never triggers it -- but a genuine dead_end held for the
        // whole streak stops the robot even mid-turn.
        int dead_end_streak_ = 0;
        int dead_end_stop_streak_ = 8;
        // True while the robot is being held stopped by the dead_end streak
        // above. Not a scenario abort: the moment /passage_type reports any
        // other corridor type, the hold is released and the current step's
        // command is restored, so driving resumes.
        bool dead_end_stopped_ = false;
        // How long the robot has actually been moving (see cmdVelCallback()
        // below, not wall-clock time) since the current step began (reset
        // in loadNextScenario()). Status telemetry only.
        double step_moving_duration_sec_ = 0.0;
        // Set when the reading latched into last_node_ right as the grace
        // period above ends already satisfies the current step's target
        // (see passageTypeCallback()'s request_update_last_node_flg block).
        // Accepting that immediately would trust a single frame, which can
        // be a transient/stale reading from the tail of a turn rather than
        // a genuine arrival. Instead it must keep reading the same way for
        // immediate_match_confirm_sec_ of further motion before it counts;
        // if it changes before then, it is treated as an ordinary node
        // change instead (see immediate_match_duration_sec_'s accumulation
        // in cmdVelCallback()).
        bool immediate_match_flg_ = false;
        double immediate_match_duration_sec_ = 0.0;
        double immediate_match_confirm_sec_ = 2.0;
        // Minimum-turn-time floor: a turn step cannot complete on a
        // "straight_road" reading (see the turn branch of
        // passageTypeCallback()) until turn_active_duration_sec_ below
        // reaches this. Raise scenario_executor/turning_min_duration_sec if
        // turns end too early; lower it if the robot over-rotates.
        double turning_min_duration_sec_ = 5.0;
        // The floor above is measured from when the robot *actually starts
        // rotating*, not from when the turn step loaded -- the downstream
        // controller takes a while to ramp the turn up, and counting that
        // lag would end turns early. turn_started_ latches true the first
        // time |cmd_vel.angular.z| reaches turn_start_angular_speed_rad_s_
        // during a turn step (see cmdVelCallback()); turn_active_duration_sec_
        // then accumulates moving time from that point on (it keeps counting
        // even if angular.z later dips, so it cannot stall). Both reset per
        // step in loadNextScenario().
        double turn_start_angular_speed_rad_s_ = 0.10;
        bool turn_started_ = false;
        double turn_active_duration_sec_ = 0.0;
        // vnm_ros/CARE's obstacle-avoidance-active flag (std_msgs/Bool, e.g.
        // /vnm/care_avoidance_active). /cmd_vel is the final velocity, so it
        // also carries CARE's repulsive steering -- which can spin the robot
        // before the commanded scenario turn has really begun. While this
        // reads true, turn_started_ above is not allowed to latch. Disabled
        // (no subscriber) when scenario_executor/care_avoidance_topic is
        // empty; harmless when nothing publishes it (a stale reading older
        // than care_avoidance_stale_timeout_sec_ is ignored, so a dropped
        // "true" cannot suppress the latch forever, and with no message at
        // all the latch uses cmd_vel alone).
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
    // Comma-separated intersection_name values that currently satisfy the
    // active target (empty when there is none, e.g. stopped). While a turn
    // step is active this looks ahead to the step the turn leads into
    // (see activeTargetLabels()), so a consumer such as corridor_classifier
    // can recognize that destination even mid-turn and publish it instead
    // of "turning" the moment it is visible (see turning_gate.py /
    // corridor_classifier_node.py), instead of always waiting for the
    // physical turn to finish.
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
    moving_linear_threshold_ = 0.02;
    node_.getParam("scenario_executor/moving_linear_threshold", moving_linear_threshold_);
    moving_angular_threshold_ = 0.02;
    node_.getParam("scenario_executor/moving_angular_threshold", moving_angular_threshold_);
    immediate_match_confirm_sec_ = 2.0;
    node_.getParam("scenario_executor/immediate_match_confirm_sec", immediate_match_confirm_sec_);
    turning_min_duration_sec_ = 5.0;
    node_.getParam("scenario_executor/turning_min_duration_sec", turning_min_duration_sec_);
    turn_start_angular_speed_rad_s_ = 0.10;
    node_.getParam("scenario_executor/turn_start_angular_speed_rad_s", turn_start_angular_speed_rad_s_);
    dead_end_stop_streak_ = 8;
    node_.getParam("scenario_executor/dead_end_stop_streak", dead_end_stop_streak_);
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
    // Start the minimum-turn-time clock only once the robot is genuinely
    // rotating for the commanded turn, not when the turn step loaded (see
    // turn_started_) and not while CARE is steering around an obstacle
    // (that angular velocity is on /cmd_vel too but is not the turn).
    if(turn_flg_ && ! turn_started_ &&
       std::abs(msg->angular.z) >= turn_start_angular_speed_rad_s_ &&
       ! careAvoidanceActive()){
        turn_started_ = true;
    }
    if(have_last_cmd_vel_stamp_ && moving){
        double dt = (now - last_cmd_vel_stamp_).toSec();
        step_moving_duration_sec_ += dt;
        if(immediate_match_flg_){
            immediate_match_duration_sec_ += dt;
        }
        if(turn_started_){
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
        labels = {"3_way_right","3_way_center","3_way_left","corner_right","corner_left"};
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

// Like currentTargetLabels(), but while a turn step is active (turn_flg_),
// the turn itself has no type to wait for, so this looks ahead to the step
// the turn leads into instead. Published on scenario_target_labels so
// corridor_classifier can, while its own turning gate is active, publish a
// recognized destination label instead of "turning" (see turning_gate.py /
// corridor_classifier_node.py). "straight_road" is always included, since
// every scenario turn is followed by a "go straight" step and a
// "straight_road" reading is what ends the turn here (see the turn branch
// of passageTypeCallback()).
std::vector<std::string> cmdVelController::activeTargetLabels(void){
    if(! turn_flg_){
        return targetLabelsForStep(scenario_progress_cnt_);
    }
    std::vector<std::string> labels = targetLabelsForStep(scenario_progress_cnt_ + 1);
    if(std::find(labels.begin(), labels.end(), "straight_road") == labels.end()){
        labels.push_back("straight_road");
    }
    return labels;
}

// True when the rest of the scenario from here is exactly an "end" step
// (突き当りまで直進) followed by the final "stop" step. While turning, the
// "end" step is the one the turn leads into (scenario_progress_cnt_ + 1);
// otherwise it is the current step. Only in this shape is a "dead_end"
// reading a valid way to finish the turn -- and the dead-end hold at the top
// of passageTypeCallback() is suppressed so it cannot stop the robot before
// the turn (and then the end step) completes on that dead_end.
bool cmdVelController::atDeadEndStopTail(void){
    int end_step = turn_flg_ ? scenario_progress_cnt_ + 1 : scenario_progress_cnt_;
    int stop_step = end_step + 1;
    if(end_step < 0 || stop_step >= scenario_num_){
        return false;
    }
    return *std::next(target_type_itr_begin_, end_step) == "end"
        && *std::next(target_action_itr_begin_, stop_step) == "stop";
}

bool cmdVelController::compareScenarioAndPassageType(const scenario_navigation_msgs::cmd_dir_intersection::ConstPtr& passage_type){//シナリオと現在のノードの比較
    std::vector<std::string> labels = currentTargetLabels();
    return std::find(labels.begin(), labels.end(), passage_type->intersection_name) != labels.end();
}

void cmdVelController::publishTargetLabels(void){
    std::vector<std::string> labels = activeTargetLabels();
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
    // A pending confirmation belongs to the step that started it; never
    // let it carry over (e.g. if next_scenario forces a step change while
    // one is still pending).
    immediate_match_flg_ = false;
    immediate_match_duration_sec_ = 0.0;
    turn_started_ = false;
    turn_active_duration_sec_ = 0.0;
    dead_end_streak_ = 0;
    dead_end_stopped_ = false;
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
                std::copy(std::begin(left_list),std::end(left_list),std::begin(cmd_data.cmd_dir));
            }
            //else if(action.find("right")){
            else if(action == "turn_right"){
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
    // Dead-end hold: a run of "dead_end" readings means the robot is
    // heading into a wall (including mid-turn -- a wrong turn into a dead
    // end still needs to stop). One stray "dead_end" frame (e.g. flashing
    // by mid-turn while the classifier sweeps labels) is ignored, since any
    // non-dead_end reading resets the count. dead_end_stop_streak_ in a row
    // holds the robot stopped; this is NOT a scenario abort -- the moment
    // /passage_type reports any other corridor type the hold is released
    // (block below) and the current step's command is restored.
    //
    // Exception: when the scenario tail is "突き当りまで直進．停止．"
    // (atDeadEndStopTail()), a "dead_end" is the expected destination, not a
    // wrong turn -- do not accumulate the hold streak, so the turn branch
    // below can finish the turn on that "dead_end" (after the min-turn-time
    // floor) and the following "end" step can then complete normally.
    if(passage_type->intersection_name == "dead_end" && ! atDeadEndStopTail()){
        dead_end_streak_++;
    }
    else{
        dead_end_streak_ = 0;
    }
    if(dead_end_streak_ >= dead_end_stop_streak_){
        if(! dead_end_stopped_){
            ROS_WARN("dead_end x%d: holding robot until the view clears", dead_end_streak_);
            dead_end_stopped_ = true;
        }
        stop_flg_ = true;
        std_msgs::Bool stop_msg;
        stop_msg.data = true;
        stop_pub_.publish(stop_msg);
        std::copy(std::begin(stop_list),std::end(stop_list),std::begin(cmd_data.cmd_dir));
        cmd_data_pub.publish(cmd_data);
        publishTargetLabels();
        printStatus(passage_type);
        return;
    }
    if(dead_end_stopped_){
        // Dead end cleared (streak was just reset above): release the hold,
        // restore the current step's command, and fall through to resume
        // normal processing.
        dead_end_stopped_ = false;
        stop_flg_ = false;
        std_msgs::Bool stop_msg;
        stop_msg.data = false;
        stop_pub_.publish(stop_msg);
        if(turn_flg_ && action == "turn_left"){
            std::copy(std::begin(left_list),std::end(left_list),std::begin(cmd_data.cmd_dir));
        }
        else if(turn_flg_ && action == "turn_right"){
            std::copy(std::begin(right_list),std::end(right_list),std::begin(cmd_data.cmd_dir));
        }
        else{
            std::copy(std::begin(str_list),std::end(str_list),std::begin(cmd_data.cmd_dir));
        }
        ROS_INFO("dead_end cleared: resuming (action=%s)", action.c_str());
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
        publishTargetLabels();
        printStatus(passage_type);
        return;
    }
    if(! stop_flg_){
        if(request_update_last_node_flg){
            updateLastNode(passage_type);
            request_update_last_node_flg = false;
            // Normally change_node_flg_ only becomes true once a later
            // reading differs from the node just latched above. But if the
            // target is already satisfied the instant this latch happens
            // (e.g. a short corridor where dead_end is visible immediately
            // after the turn), current == last_node_ forever after and
            // compareLastNodeAndCurrentNode() would never report a change --
            // so the goal check below would never run and the scenario
            // would stall. Start a confirmation window instead of trusting
            // this single frame outright (see immediate_match_flg_'s
            // declaration for why).
            if(! turn_flg_ && compareScenarioAndPassageType(passage_type)){
                immediate_match_flg_ = true;
                immediate_match_duration_sec_ = 0.0;
            }
        }
    }
        if(! turn_flg_){//actionがturnではない場合
            if(immediate_match_flg_){
                if(! compareScenarioAndPassageType(passage_type)){
                    // Changed before being confirmed: not a genuine
                    // immediate match after all. current now differs from
                    // last_node_ (the value latched when the window
                    // started), so the ordinary change-detection branch
                    // below will pick it up as a normal node change.
                    immediate_match_flg_ = false;
                }
                else if(immediate_match_duration_sec_ >= immediate_match_confirm_sec_){
                    // Held long enough to trust it.
                    immediate_match_flg_ = false;
                    change_node_flg_ = true;
                }
            }
            if(immediate_match_flg_){
                // Still within the confirmation window: nothing else to
                // evaluate this frame (avoids comparing against last_node_
                // below, which could spuriously read as "changed" while a
                // valid-but-not-yet-trusted match is still pending).
            }
            else if(change_node_flg_){//道タイプが変更されている場合
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
            // End the turn once /passage_type reports "straight_road": the
            // robot is then facing straight down a corridor again, so the
            // turn maneuver is complete. While rotating, the classifier
            // only ever sees walls / corners / intersections and sweeps
            // through those labels (corner_left -> 3_way_center ->
            // corner_right -> ...), never straight_road, so straight_road
            // is a reliable "turn finished" signal where the intermediate
            // labels are not. Every scenario turn is followed by a
            // "go straight" step (see config/Scenarios), so straight_road
            // is always the expected reading right after the turn.
            //
            // Gated by the minimum-turn-time floor
            // (turn_active_duration_sec_ >= turning_min_duration_sec_,
            // counted from when the robot actually started rotating -- see
            // turn_started_) so a spurious straight_road, or the ramp-up
            // lag before the turn really begins, cannot cut the turn short.
            // turnFinish() -> loadNextScenario() then switches cmd_dir to
            // [1,0,0] immediately, published at the end of this callback.
            //
            // When the turn leads straight into "突き当りまで直進．停止．"
            // (atDeadEndStopTail()), a short corridor can put the robot at
            // the dead end before any "straight_road" is ever seen, so also
            // accept "dead_end" as the turn-finished signal here. Same
            // min-turn-time floor: the robot still rotates for the full
            // turning_min_duration_sec_ before this can fire.
            if (turn_active_duration_sec_ >= turning_min_duration_sec_ &&
                (passage_type->intersection_name == "straight_road" ||
                 (atDeadEndStopTail() &&
                  passage_type->intersection_name == "dead_end")))
            {
                turnFinish(true);  //change_flg on ターン終了まで継続
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

    std::string mode = dead_end_stopped_ ? "DEAD_END_HOLD"
                       : turn_flg_ ? "TURNING"
                       : stop_flg_ ? "STOPPED"
                       : immediate_match_flg_ ? "CONFIRMING_IMMEDIATE_MATCH"
                       : change_node_flg_ ? "AT_NEW_NODE(comparing)"
                       : "WAITING_FOR_NODE_CHANGE";

    ROS_INFO(
        "[STATUS] step=%d/%d action=%s target_type=%s target_direction=%s "
        "order=%d/%d mode=%s turn_time=%.1f/%.1fs(started=%d,care=%d) "
        "step_moving_time=%.1fs immediate_match_time=%.1f/%.1fs "
        "| current_node=%s last_node=%s",
        step + 1, scenario_num_,
        step_action.c_str(), step_type.c_str(), step_direction.c_str(),
        scenario_order_cnt_, step_order,
        mode.c_str(),
        turn_active_duration_sec_, turning_min_duration_sec_,
        (int)turn_started_, (int)careAvoidanceActive(),
        step_moving_duration_sec_,
        immediate_match_duration_sec_, immediate_match_confirm_sec_,
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

