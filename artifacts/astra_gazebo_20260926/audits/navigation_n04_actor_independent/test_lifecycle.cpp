// Synthetic concurrency validation of exact final lifecycle guard syntax.
// Not a Gazebo shutdown simulation; no ROS/Gazebo initialized.
#include <mutex>
#include <memory>
#include <thread>
#include <future>
#include <functional>
#include <atomic>
#include <iostream>
#include <stdexcept>
struct Fixture {
 struct Lifecycle {std::mutex mutex;Fixture* owner{nullptr};};
 std::shared_ptr<Lifecycle> lifecycle_{std::make_shared<Lifecycle>()};
 std::function<void()> retained_callback;
 std::promise<void>& entered;std::shared_future<void> release;std::atomic<int>& count;
 Fixture(std::promise<void>&e,std::shared_future<void>r,std::atomic<int>&c):entered(e),release(r),count(c){
  lifecycle_->owner=this;auto life=lifecycle_;
  retained_callback=[life](){std::lock_guard<std::mutex> lock(life->mutex);if(life->owner)life->owner->update();};
 }
 void update(){entered.set_value();release.wait();++count;}
 ~Fixture(){std::lock_guard<std::mutex> lock(lifecycle_->mutex);lifecycle_->owner=nullptr;}
};
int main(){int checks=0;for(int i=0;i<20;++i){std::promise<void>e,r;auto ef=e.get_future();std::atomic<int>c{0};Fixture*f=new Fixture(e,r.get_future().share(),c);auto callback=f->retained_callback;std::thread active(callback);ef.wait();auto destroying=std::async(std::launch::async,[f](){delete f;});if(destroying.wait_for(std::chrono::milliseconds(2))!=std::future_status::timeout)throw std::runtime_error("guard did not wait");++checks;r.set_value();active.join();destroying.get();callback();if(c!=1)throw std::runtime_error("queued callback dereferenced destroyed owner");++checks;}std::cout<<"checks="<<checks<<" passed (synthetic shared guard, not live Gazebo shutdown)\n";}
