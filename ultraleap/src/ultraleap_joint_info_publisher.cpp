/* Copyright (C) 2012-2017 Ultraleap Limited. All rights reserved.
 *
 * Use of this code is subject to the terms of the Ultraleap SDK agreement
 * available at https://central.leapmotion.com/agreements/SdkAgreement unless
 * Ultraleap has signed a separate license agreement with you or your
 * organisation.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ultraleap_msgs/msg/ultraleap_joint_info.hpp"

extern "C" {
#include "LeapC.h"
#include "device_connection.h"
}

using namespace std::chrono_literals;
using namespace ultraleap_msgs::msg;

typedef std::vector<float> Vector3;

// JointInfo := Position and quaternion rotation of each joint
class UltraleapJointInfoPublisher : public rclcpp::Node {
 public:
  UltraleapJointInfoPublisher() : Node("ultraleap_joint_info_publisher") {
    connect_to_ultraleap_device(); 

    ultraleap_joint_info_publisher_ = this->create_publisher<UltraleapJointInfo>("ultraleap_joint_info", 10);

    ultraleap_joint_info_timer_ = this->create_wall_timer(20ms, std::bind(&UltraleapJointInfoPublisher::ultraleap_joint_info_publisher_callback, this));
  }

 private:
  void connect_to_ultraleap_device() {
    lastFrameID = 0;

    LEAP_CONNECTION* connection_handle = OpenConnection();
    while (!IsConnected) {
      usleep(100);  // wait a bit to let the connection complete
    }

    RCLCPP_INFO(this->get_logger(), "CONNECTED");
    LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();
    if (deviceProps) {
      RCLCPP_INFO(this->get_logger(), "Using device %s.\n", deviceProps->serial);
    }
  }

  void add_vector_position_to_message(const LEAP_VECTOR position, JointInfo* message) { 
    message->x = position.x;
    message->y = position.y; 
    message->z = position.z;
  }

  void add_leap_quaternion_rotation_to_message(const LEAP_QUATERNION current_quaternion, const LEAP_QUATERNION next_quaternion, auto message) {    
    for (int i = 0; i < 4; ++i) {
        message->current_quaternion[i] = static_cast<float>(current_quaternion.v[i]);
        message->next_quaternion[i] = static_cast<float>(next_quaternion.v[i]);
    }
  }

  void add_finger_joint_info_to_message(const LEAP_BONE current_bone, const LEAP_BONE next_bone, JointInfo* message) { 
    add_vector_position_to_message(current_bone.next_joint, message); // Equivalent to next_bone.prev_joint
    add_leap_quaternion_rotation_to_message(current_bone.rotation, next_bone.rotation, message);
  }

  // Note that the finger tip is not a joint, so angle and rotation information is left as default 
  void add_finger_tip_position_to_message(const LEAP_BONE distal_bone, JointInfo* message) { 
    add_vector_position_to_message(distal_bone.next_joint, message);
  }

  void add_finger_info_to_message(const LEAP_DIGIT finger, FingerInfo* message) { 
    add_finger_joint_info_to_message(finger.metacarpal, finger.proximal, &(message->mcp));
    add_finger_joint_info_to_message(finger.proximal, finger.intermediate, &(message->pip));
    add_finger_joint_info_to_message(finger.intermediate, finger.distal, &(message->dip));
    add_finger_tip_position_to_message(finger.distal, &(message->tip)); 
  }

  void ultraleap_joint_info_publisher_callback() {
    LEAP_TRACKING_EVENT* frame = GetFrame();

    if (frame && (frame->tracking_frame_id > lastFrameID)) {
      lastFrameID = frame->tracking_frame_id;
      RCLCPP_INFO(this->get_logger(), "Frame %lli with %i hands.\n", (long long int)frame->tracking_frame_id, frame->nHands);

      UltraleapJointInfo message = UltraleapJointInfo(); 

      if (frame->nHands >= 1) {
        LEAP_HAND* hand = &frame->pHands[0];

        LEAP_DIGIT thumb = hand->thumb;

        for (int digit_id = 0; digit_id < 4; digit_id++) {

          // digits indexing starts at 1 to avoid the thumb 
          LEAP_DIGIT finger = hand->digits[digit_id + 1];
          
          add_finger_info_to_message(finger, &message.digit_info.finger_info[digit_id]);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing UltraleapJointInfo");
        ultraleap_joint_info_publisher_ -> publish(message);
      }
    }
  }

  rclcpp::TimerBase::SharedPtr ultraleap_joint_info_timer_;
  rclcpp::Publisher<UltraleapJointInfo>::SharedPtr ultraleap_joint_info_publisher_;
  int64_t lastFrameID;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltraleapJointInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}
