/*
 *  Copyright (C) 2017 Eduardo Perdices <eperdices at gsyc dot es>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef ORB_SLAM2_TIMER_H_
#define ORB_SLAM2_TIMER_H_

#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

namespace ORB_SLAM2 {

class Timer {
 public:
  explicit Timer(bool autostart) : time_(0.0) {
    if (autostart)
      Start();
  }

  inline double GetTime() {
    return time_;
  }

  inline double GetMsTime() {
    return time_*1000.0;
  }

  inline void Start() {
    gettimeofday(&start_time_, NULL);
  }


  inline void Stop() {
    timeval end_time;
    gettimeofday(&end_time, NULL);
    long seconds  = end_time.tv_sec  - start_time_.tv_sec;
    long useconds = end_time.tv_usec - start_time_.tv_usec;
    time_ = ((seconds) + useconds*0.000001);
  }

 private:
  timeval start_time_;
  double time_;
};

}  // namespace ORB_SLAM2


#endif  // ORB_SLAM2_TIMER_H_
