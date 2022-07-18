
#include <iostream>
#include <string>
using namespace std;

// FSM implementation in Hierarchical Hybrid Predictive Control of an Autonomous Road Vehicle

#define DIS_TO_FV_LIMIT 200.0 // m
#define DIS_TO_RV_LIMIT 200.0 // m

float distance_to_front_vehicle = 180.0f;
float distance_to_rear_vehicle = 180.0f;
float v_front = 85.0f; // kph
float v_rear = 0.0f; // kph
// 车辆设定运行速度的最小值
float v_lcl = 80.0f; // kph
// 车辆设定运行速度的最大值
float v_lch = 100.0f; // kph
// 无前车后车时的参考车速
float v_reference = 0.5f * (v_lcl + v_lch); // kph
float v_target = v_reference; // ego vehicle speed
// 根据导航从匝道汇入高速公路
bool merge_is_required = false;
// 根据导航从匝道离开高速公路
bool exit_is_required = false;
bool change_lane_is_allowed = true;

// condition: c1,c2,c3,e1,e2,f1
bool c_1 = v_front < v_target - 0.5f ? true : false;
bool c_2 = v_rear > v_target + 0.5f ? true : false;
bool c_3 = merge_is_required == true || exit_is_required == true ? true : false;
bool e_1 = v_target < v_lcl - 0.5f ? true : false;
bool e_2 = v_target > v_lch + 0.5f ? true : false;
bool f_1 = change_lane_is_allowed == true ? true : false;

enum State{
    S0, // none
    S1, // Normal Tracking
    S2, // Following
    S3, // Leading
    S4, // Lane Change
};

int main(){
    State current_state = S1;
    for (size_t i = 0; i < 5; i++) {
    cout << "state before case is : " << current_state << endl;
    cout << "----------" << endl;
    switch (current_state) {
      case S1: {
        if (distance_to_front_vehicle < DIS_TO_FV_LIMIT && c_1) {   
            current_state = S2;
            v_target = v_front;
        } else if (distance_to_rear_vehicle < DIS_TO_RV_LIMIT && c_2) {
            current_state = S3;
            v_target = v_rear;
        } else if (c_3 && f_1) {
            v_target = v_reference;
            current_state = S4;
        }

        c_1 = v_front < v_target - 0.5f ? true : false;
        c_2 = v_rear > v_target + 0.5f ? true : false;
        e_1 = v_target < v_lcl - 0.5f ? true : false;
        e_2 = v_target > v_lch + 0.5f ? true : false;

        // cout << "c1 : " << c_1 << "  " << "c2 : " << c_2 << "  "
        //      << "e1 : " << e_1 << "  " << "e2 : " << e_2 << endl;
        cout << "target velocity : " << v_target << endl;
        cout << "state after S1 is : " << current_state << endl;
        cout << "----------" << endl;
        break;
        }

      case S2: {
        if (!c_1 && distance_to_front_vehicle >= DIS_TO_FV_LIMIT) {
            current_state = S1;
            v_target = v_reference;
        } else if ( (c_1 && e_1 && f_1) || (c_3 && f_1) ) {
            current_state = S4;
            v_target = v_reference;
        }

        c_1 = v_front < v_target - 0.5f ? true : false;
        c_2 = v_rear > v_target + 0.5f ? true : false;
        e_1 = v_target < v_lcl - 0.5f ? true : false;
        e_2 = v_target > v_lch + 0.5f ? true : false;

        // cout << "c1 : " << c_1 << "  " << "c2 : " << c_2 << "  "
        //      << "e1 : " << e_1 << "  " << "e2 : " << e_2 << endl;
        cout << "target velocity : " << v_target << endl;
        cout << "state after S2 is : " << current_state << endl;
        cout << "----------" << endl;
        break;
        }
        
      case S3: {
        if (!c_2 && distance_to_rear_vehicle >= DIS_TO_RV_LIMIT) {
            current_state = S1;
            v_target = v_reference;
        } else if (c_1 && c_2) {
            current_state = S2;
            v_target = v_front;
        } else if ( (c_2 && e_2 && f_1) || (c_3 && f_1) ) {
           current_state = S4;
           v_target = v_reference;
        }

        c_1 = v_front < v_target - 0.5f ? true : false;
        c_2 = v_rear > v_target + 0.5f ? true : false;
        e_1 = v_target < v_lcl - 0.5f ? true : false;
        e_2 = v_target > v_lch + 0.5f ? true : false;

        // cout << "c1 : " << c_1 << "  " << "c2 : " << c_2 << "  "
        //      << "e1 : " << e_1 << "  " << "e2 : " << e_2 << endl;
        cout << "target velocity : " << v_target << endl;
        cout << "state after S3 is : " << current_state << endl;
        cout << "----------" << endl;
        break;
        }

       case S4: {
        // TODO(yxb): do lane change
        if (!f_1) {
            current_state = S1;
            v_target = v_reference;
        }

        c_1 = v_front < v_target - 0.5f? true : false;
        c_2 = v_rear > v_target + 0.5f ? true : false;
        e_1 = v_target < v_lcl - 0.5f ? true : false;
        e_2 = v_target > v_lch + 0.5f ? true : false;

        // cout << "c1 : " << c_1 << "  " << "c2 : " << c_2 << "  "
        //      << "e1 : " << e_1 << "  " << "e2 : " << e_2 << endl;
        cout << "target velocity : " << v_target << endl;
        cout << "state after S4 is : " << current_state << endl;
        cout << "----------" << endl;
        break;
        }

      default:
        cout << "nothing happened!" << endl;
        break;
    }
    }

    return 0;
}