// Copyright 2020 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 * @author Jeremie Deray
 */

#ifndef can_mux__can_mux_HPP_
#define can_mux__can_mux_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <can_msgs/msg/frame.hpp>

#include <list>
#include <memory>
#include <string>

using std::chrono_literals::operator""s;

namespace can_mux
{
// Forwarding declarations:
class CanMuxDiagnostics;
struct CanMuxDiagnosticsStatus;
class VelocityTopicHandle;
class LockTopicHandle;

/**
 * @brief The TwistMux class implements a top-level can multiplexer module
 * that priorize different velocity command topic inputs according to locks.
 */
class CanMux : public rclcpp::Node
{
public:
  template<typename T>
  using handle_container = std::list<T>;

  using velocity_topic_container = handle_container<VelocityTopicHandle>;
  using lock_topic_container = handle_container<LockTopicHandle>;

  CanMux();
  ~CanMux() = default;

  void init();

  bool hasPriority(const VelocityTopicHandle & Can);

  void publishCan(const can_msgs::msg::Frame::ConstSharedPtr & msg);

  void updateDiagnostics();

protected:
  typedef CanMuxDiagnostics diagnostics_type;
  typedef CanMuxDiagnosticsStatus status_type;

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  static constexpr std::chrono::duration<int64_t> DIAGNOSTICS_PERIOD = 1s;

  /**
   * @brief velocity_hs_ Velocity topics' handles.
   * Note that if we use a vector, as a consequence of the re-allocation and
   * the fact that we have a subscriber inside with a pointer to 'this', we
   * must reserve the number of handles initially.
   */
  std::shared_ptr<velocity_topic_container> velocity_hs_;
  std::shared_ptr<lock_topic_container> lock_hs_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr cmd_pub_;

  can_msgs::msg::Frame last_cmd_;

  template<typename T>
  void getTopicHandles(const std::string & param_name, handle_container<T> & topic_hs);

  int getLockPriority();

  std::shared_ptr<diagnostics_type> diagnostics_;
  std::shared_ptr<status_type> status_;
};

}  // namespace can_mux

#endif  // can_mux__can_mux_HPP_
