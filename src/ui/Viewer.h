/**
 *
 *  Copyright (C) 2017 Eduardo Perdices <eperdices at gsyc dot es>
 *
 *  The following code is a derivative work of the code from the ORB-SLAM2 project,
 *  which is licensed under the GNU Public License, version 3. This code therefore
 *  is also licensed under the terms of the GNU Public License, version 3.
 *  For more information see <https://github.com/raulmur/ORB_SLAM2>.
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

#ifndef SD_SLAM_VIEWER_H
#define SD_SLAM_VIEWER_H

#include <mutex>
#include <string>
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

namespace SD_SLAM {

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class Viewer {
 public:
  Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking);

  // Main thread function. Draw points, keyframes, the current camera pose and the last processed
  // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
  void Run();

  void RequestFinish();

  void RequestStop();

  bool isFinished();

  bool isStopped();

  void Release();

 private:
  bool Stop();

  System* mpSystem;
  FrameDrawer* mpFrameDrawer;
  MapDrawer* mpMapDrawer;
  Tracking* mpTracker;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  bool mbStopped;
  bool mbStopRequested;
  std::mutex mMutexStop;
};

}  // namespace SD_SLAM


#endif  // SD_SLAM_VIEWER_H

