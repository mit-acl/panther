#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <chrono>
#include <ros/ros.h>

namespace PANTHER_timers
{
class Timer
{
  typedef std::chrono::high_resolution_clock high_resolution_clock;
  typedef std::chrono::milliseconds milliseconds;
  typedef std::chrono::microseconds microseconds;
  typedef std::chrono::nanoseconds nanoseconds;

public:
  explicit Timer(bool run = false)
  {
    if (run)
    {
      tic();
    }
  }
  void tic()
  {
    start_ = high_resolution_clock::now();
  }
  double elapsedSoFarMs() const
  {
    return (std::chrono::duration_cast<nanoseconds>(high_resolution_clock::now() - start_)).count() / (1e6);
  }
  void toc()
  {
    end_ = high_resolution_clock::now();
  }
  double getMsSaved()
  {
    return (std::chrono::duration_cast<nanoseconds>(end_ - start_)).count() / (1e6);
  }
  void reset()
  {
    start_ = high_resolution_clock::now();
    // end_ = high_resolution_clock::now();
  }
  // double elapsedUs() const
  // {
  //   return (std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - start_)).count();
  // }
  template <typename T, typename Traits>
  friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const Timer& timer)
  {
    return out << " " << timer.elapsedSoFarMs() << " ms ";
  }

private:
  high_resolution_clock::time_point start_;
  high_resolution_clock::time_point end_;
};

class ROSTimer
{
public:
  ROSTimer(bool run = false)
  {
    if (run)
      tic();
  }
  void tic()
  {
    start_ = ros::Time::now().toSec();
  }
  double elapsedSoFarMs() const
  {
    return 1000 * (ros::Time::now().toSec() - start_);
  }
  template <typename T, typename Traits>
  friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const ROSTimer& timer)
  {
    return out << timer.elapsedSoFarMs();
  }

private:
  double start_;
};

class ROSWallTimer
{
public:
  ROSWallTimer(bool run = false)
  {
    if (run)
      tic();
  }
  void tic()
  {
    start_ = ros::WallTime::now().toSec();
  }
  double elapsedSoFarMs() const
  {
    return 1000 * (ros::WallTime::now().toSec() - start_);
  }
  template <typename T, typename Traits>
  friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const ROSWallTimer& timer)
  {
    return out << timer.elapsedSoFarMs();
  }

private:
  double start_;
};
}  // namespace PANTHER_timers

#endif  // TIMER_HPP_