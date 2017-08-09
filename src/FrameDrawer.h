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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
  FrameDrawer(Map* pMap);

  // Update info from the last processed frame.
  void Update(Tracking *pTracker);

  // Draw last processed frame.
  cv::Mat DrawFrame();

protected:

  void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

  // Info of the frame to be drawn
  cv::Mat mIm;
  int N;
  vector<cv::KeyPoint> mvCurrentKeys;
  vector<bool> mvbMap;
  int mnTracked;
  vector<cv::KeyPoint> mvIniKeys;
  vector<int> mvIniMatches;
  int mState;

  Map* mpMap;

  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
