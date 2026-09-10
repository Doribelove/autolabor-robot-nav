#include <autolabor_operator_gui/main_window.h>

namespace autolabor_operator_gui
{
void MainWindow::enqueuePendingLitterInput(const PendingLitterInput& input)
{
  // Bound backlog; loss of queued frames invalidates unconfirmed streaks.
  // Retain the latest map so an overflow cannot strand the new page.
  if (pending_litter_inputs_.size() >= 128)
  {
    pending_litter_inputs_.clear();
    PendingLitterInput reset;
    reset.reset = true;
    reset.map = pending_litter_map_;
    reset.received_at = input.received_at;
    reset.received_stamp = input.received_stamp;
    pending_litter_inputs_.push_back(reset);
  }
  pending_litter_inputs_.push_back(input);
}

void MainWindow::refreshPendingLitter()
{
  std::deque<PendingLitterInput> inputs;
  std::deque<std::uint64_t> completed_ids;
  std::deque<std::string> status_updates;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    inputs.swap(pending_litter_inputs_);
    completed_ids.swap(pending_litter_completed_ids_);
    status_updates.swap(pending_litter_status_updates_);
  }
  pending_litter_page_->setAvailable(static_map_mode_ && master_online_ && ros_interfaces_ready_);
  const auto now = ros::Time::now();
  const auto wall_now = ros::WallTime::now();
  for (const auto& input : inputs)
    pending_litter_page_->consume(input, now, wall_now);
  for (const auto object_id : completed_ids)
    pending_litter_page_->markCleaned(object_id);
  for (const auto& status : status_updates)
    pending_litter_page_->updateCleaningStatus(QString::fromStdString(status));
  if (inputs.empty()) pending_litter_page_->consume({}, now, wall_now);
}
}  // namespace autolabor_operator_gui
