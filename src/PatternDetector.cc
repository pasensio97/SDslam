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

#include "PatternDetector.h"
#include "MapPoint.h"
#include "Converter.h"
#include "extra/log.h"

using std::vector;
using std::pair;

namespace SD_SLAM {

PatternDetector::PatternDetector() {
  // Pattern internal rows and cols
  cb_rows_ = 4; 
  cb_cols_ = 6;
}

PatternDetector::~PatternDetector() {
}

bool PatternDetector::Detect(const Frame &frame) {
  vector<cv::Point> imgPoints;

  if(SearchPattern(frame, imgPoints1_))
    if (GetRT(frame, imgPoints1_, RT_))
      return true;

  return false;
}

bool PatternDetector::SearchPattern(const Frame &frame, vector<cv::Point2d>& pixels) {
  vector<cv::Point> imgPoints;
  const cv::Mat &img = frame.mvImagePyramid[0];

  pixels.clear();

  if(SearchChessboard(img, imgPoints)) {
    // Save extremes (clockwise order)
    pixels.push_back(imgPoints[0]);
    pixels.push_back(imgPoints[cb_cols_-1]);
    pixels.push_back(imgPoints[(cb_rows_*cb_cols_)-1]); 
    pixels.push_back(imgPoints[(cb_rows_-1)*cb_cols_]);
    return true;
  }

  return false;
}

bool PatternDetector::GetRT(const Frame &frame, const vector<cv::Point2d>& pixels, Eigen::Matrix4d& RT) {
  vector<cv::Point2f> positions_cv_2d;
  vector<cv::Point3f> positions_cv_3d;
  cv::Mat raux, taux, rvec, cvR, cvT;

  // Save features inside chessboard
  points_.clear();
  for (size_t i = 0, iend=frame.mvKeys.size(); i < iend; i++) {
    if (IsInsideRectangle(pixels, frame.mvKeys[i].pt))
      points_.push_back(std::make_pair(i, Eigen::Vector3d::Zero()));
  }

  if (points_.size() < 50) {
    LOGD("Pattern doesn't have enough points (%lu)", points_.size());
    return false;
  }

  // Get 3d positions for each pixel
  Get3DPoints(frame, pixels, points_);

  // Solve PnP
  for (size_t i = 0, iend=points_.size(); i < iend; i++) {
    Eigen::Vector3d p = points_[i].second;
    int idx = points_[i].first;
    
    positions_cv_2d.push_back(frame.mvKeys[idx].pt);
    positions_cv_3d.push_back(cv::Point3f(p(0), p(1), 0.0));    
  }

  cv::Mat K = Converter::toCvMat(frame.mK);
  cv::solvePnP(positions_cv_3d, positions_cv_2d, K, frame.mDistCoef, raux, taux);

  raux.convertTo(rvec, CV_32F);
  taux.convertTo(cvT, CV_32F);
  cv::Rodrigues(rvec, cvR);

  Eigen::Matrix4d RT_aux;
  RT_aux.setIdentity();
  RT_aux.block<3, 3>(0, 0) = Converter::toMatrix3d(cvR);
  RT_aux.block<3, 1>(0, 3) = Converter::toVector3d(cvT);

  // Absolute position
  RT = RT_aux.inverse();

  LOGD("Pattern found!");

  return true;
}

bool PatternDetector::SearchChessboard(const cv::Mat &img, vector<cv::Point> &candidate) {
  const cv::Size patternsize(cb_cols_, cb_rows_);
  return cv::findChessboardCorners(img, patternsize, candidate);
}

void PatternDetector::Get3DPoints(const Frame &frame, const vector<cv::Point2d>& pixels, 
                                  vector<pair<int, Eigen::Vector3d>> &points) {
  cv::Point2f input[4];
  cv::Point2f output[4];
  Eigen::Vector3d p2d, p3d;
  cv::Mat Hcv, haux;
  const double size_w = 0.151;  // Cell size (m)
  const double size_h = 0.0906;  // Cell size (m)

  input[0] = pixels[0];
  input[1] = pixels[1];
  input[2] = pixels[2];
  input[3] = pixels[3];

  output[0].x = 0;
  output[0].y = 0;
  output[1].x = 0;
  output[1].y = size_w;
  output[2].x = size_h;
  output[2].y = size_w;
  output[3].x = size_h;
  output[3].y = 0;

  // Calc transformation between image and 3d plane
  haux = cv::getPerspectiveTransform(input, output);
  haux.convertTo(Hcv, CV_32F);
  Eigen::Matrix3d H = Converter::toMatrix3d(Hcv);

  // Get 3d position for each selected 2d point
  for (auto i=points.begin(); i != points.end(); i++) {
    int idx = i->first;
    p2d << frame.mvKeys[idx].pt.x, frame.mvKeys[idx].pt.y, 1.0;
    p3d = H*p2d;
    p3d /= p3d(2);
    i->second << p3d(0), p3d(1), 0.0;
  }
}

bool PatternDetector::IsInsideRectangle(const vector<cv::Point2d>& rectangle, const cv::Point2d& point) {
  double area1, area2;

  // Sum of areas of each triangle 
  area1 = TriangleArea(rectangle[0], rectangle[1], point);
  area1 += TriangleArea(rectangle[1], rectangle[2], point);
  area1 += TriangleArea(rectangle[2], rectangle[3], point);
  area1 += TriangleArea(rectangle[3], rectangle[0], point);

  // Quadrilateral area
  area2 = TriangleArea(rectangle[0], rectangle[1], rectangle[3]);
  area2 += TriangleArea(rectangle[1], rectangle[2], rectangle[3]);

  return area1 == area2;
}

double PatternDetector::TriangleArea(const cv::Point2d& a, const cv::Point2d& b, const cv::Point2d& c) {
  double area = abs((a.x*(b.y-c.y)+b.x*(c.y-a.y)+c.x*(a.y-b.y))/2.0);
  return area;
}

}  // namespace SD_SLAM
