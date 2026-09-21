#ifndef R2_MODE1_BT_BLACKBOARD_KEYS_HPP_
#define R2_MODE1_BT_BLACKBOARD_KEYS_HPP_

namespace r2_mode1_bt {

// Blackboard keys for sharing data between BT nodes
namespace bb_keys {

// Navigation
constexpr const char* kPointIndex = "point_index";
constexpr const char* kGrabCount = "grab_count";
constexpr const char* kCurrentGoal = "current_goal";
constexpr const char* kNavArrived = "nav_arrived";
constexpr const char* kCurrentNavTarget = "current_nav_target";
constexpr const char* kLastArrivedIndex = "last_arrived_index";
constexpr const char* kGoFinal = "go_final";
constexpr const char* kForceFinal = "force_final";

// Object detection
constexpr const char* kObjectPresent = "object_present";
constexpr const char* kCurrentDepth = "current_depth";
constexpr const char* kPreGrabDepth = "pre_grab_depth";

// TF alignment
constexpr const char* kTagY = "tag_y";
constexpr const char* kAligned = "aligned";

// Barcode
constexpr const char* kBarcodeData = "barcode_data";
constexpr const char* kSpecialBarcodeTriggered = "special_barcode_triggered";

// Grab feedback
constexpr const char* kCheckingGrabFeedback = "checking_grab_feedback";
constexpr const char* kGrabStartTime = "grab_start_time";
constexpr const char* kGrabSuccess = "grab_success";
constexpr const char* kGrabFeedbackSuccess = "grab_feedback_success";

// Control flags
constexpr const char* kNodeRunning = "node_running";
constexpr const char* kCurrentState = "current_state";

}  // namespace bb_keys

}  // namespace r2_mode1_bt

#endif  // R2_MODE1_BT_BLACKBOARD_KEYS_HPP_
