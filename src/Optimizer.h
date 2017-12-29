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

#ifndef SD_SLAM_OPTIMIZER_H
#define SD_SLAM_OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "extra/g2o/types/types_seven_dof_expmap.h"

namespace SD_SLAM {

class LoopClosing;

class Optimizer {
 public:
  void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF = 0,
                 const bool bRobust = true);
  void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                     const unsigned long nLoopKF = 0, const bool bRobust = true);
  void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);
  int static PoseOptimization(Frame* pFrame);

  // if bFixScale is true, 6DoF optimization (stereo, rgbd), 7DoF otherwise (mono)
  void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                     const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                     const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                     const std::map<KeyFrame *, std::set<KeyFrame *> > &LoopConnections,
                     const bool &bFixScale);

  // if bFixScale is true, optimize SE3 (stereo, rgbd), Sim3 otherwise (mono)
  static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
              g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_OPTIMIZER_H
