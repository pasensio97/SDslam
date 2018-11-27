/**
 *
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

#ifndef SD_SLAM_PATTERNDETECTOR_H_
#define SD_SLAM_PATTERNDETECTOR_H_

#include <iostream>
#include <Eigen/Dense>
#include "KeyFrame.h"

namespace SD_SLAM {

class PatternDetector {
 public:
  PatternDetector();
  ~PatternDetector();

  inline Eigen::Matrix4d GetRT() { return RT_; }
  inline std::vector<std::pair<int, Eigen::Vector3d>>& GetPoints() { return points_; }

  bool Detect(const Frame &frame);

  // Search pattern in image and save detected pixels
  bool SearchPattern(const Frame &frame, std::vector<cv::Point2d>& pixels);

  // Calculate RT from image points
  bool GetRT(const Frame &frame, const std::vector<cv::Point2d>& pixels, Eigen::Matrix4d& RT);
  
  void SetCellSizeW(double s_cell_w);
  void SetCellSizeH(double s_cell_h);

 private:
  // Search chessboard in image
  bool SearchChessboard(const cv::Mat &img, std::vector<cv::Point> &candidate);

  // Get 3D position for each 2D pixel
  void Get3DPoints(const Frame &frame, const std::vector<cv::Point2d>& pixels,
                   std::vector<std::pair<int, Eigen::Vector3d>> &points);

  // Return true if point is inside rectangle
  bool IsInsideRectangle(const std::vector<cv::Point2d>& rectangle, const cv::Point2d& point);
  double TriangleArea(const cv::Point2d& a, const cv::Point2d& b, const cv::Point2d& c);

  
  std::vector<cv::Point2d> imgPoints1_;                   // Detected points in image
  std::vector<std::pair<int, Eigen::Vector3d>> points_;   // Selected 2D/3D points
  Eigen::Matrix4d RT_;                                    // Camera RT (absolute coordinates)

  int cb_rows_;
  int cb_cols_;
  
  double cell_w = 0.0283;  // Cell size (m)
  double cell_h = 0.0283;  // Cell size (m)
};

}  // namespace SD_SLAM

#endif  // SD_SLAM_PATTERNDETECTOR_H_
