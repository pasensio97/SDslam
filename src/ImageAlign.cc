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

#include "ImageAlign.h"
#include "extra/timer.h"
#include "extra/log.h"

using std::vector;
using std::endl;
using std::set;

namespace SD_SLAM {

ImageAlign::ImageAlign() {
  stop_ = false;
  chi2_ = 1e10;
  error_ = 1e10;
  n_meas_ = 0;

  patch_size_ = 4;
  max_level_ = 4;
  min_level_ = 2;
  max_its_ = 30;
}

ImageAlign::~ImageAlign() {
}

bool ImageAlign::ComputePose(Frame &CurrentFrame, const Frame &LastFrame) {
  int size, patch_area, counter;
  float scale;
  int max_points = 300;

  cam_fx_ = CurrentFrame.fx;
  cam_fy_ = CurrentFrame.fy;
  cam_cx_ = CurrentFrame.cx;
  cam_cy_ = CurrentFrame.cy;

  Timer total(true);

  if (static_cast<int>(CurrentFrame.mvImagePyramid.size()) <= max_level_) {
    LOGE("Not enough pyramid levels");
    return false;
  }

  // Save valid points seen in last frame
  counter = 0;
  for (int i=0; i<LastFrame.N && counter<max_points; i++) {
    MapPoint* pMP = LastFrame.mvpMapPoints[i];
    if (!pMP || LastFrame.mvbOutlier[i])
      continue;

    Eigen::Vector3d p = pMP->GetWorldPos();
    points_.push_back(p);
    counter++;
  }

  size = points_.size();
  if (size == 0) {
    LOGE("No points to track!");
    return false;
  }

  patch_area = patch_size_*patch_size_;
  patch_cache_ = cv::Mat(size, patch_area, CV_32F);
  visible_pts_.resize(size, false);
  jacobian_cache_.resize(Eigen::NoChange, size*patch_area);

  // Initial displacement between frames.
  Eigen::Matrix4d current_se3 = CurrentFrame.GetPose() * LastFrame.GetPoseInverse();
  Eigen::Matrix4d last_pose = LastFrame.GetPose();

  for (int level=max_level_; level >= min_level_; level--) {
    jacobian_cache_.setZero();

    scale = CurrentFrame.mvInvScaleFactors[level];
    Optimize(CurrentFrame.mvImagePyramid[level], LastFrame.mvImagePyramid[level], last_pose, current_se3, scale);
  }

  Eigen::Matrix4d pose = current_se3 * last_pose;
  CurrentFrame.SetPose(pose);

  total.Stop();
  LOGD("Align time is %.2fms", total.GetMsTime());

  return true;
}

bool ImageAlign::ComputePose(Frame &CurrentFrame, KeyFrame *LastKF, bool fast) {
  int size, patch_area, counter;
  float scale;
  int max_points;

  if (fast)
    max_points = 100;
  else
    max_points = 300;

  cam_fx_ = CurrentFrame.fx;
  cam_fy_ = CurrentFrame.fy;
  cam_cx_ = CurrentFrame.cx;
  cam_cy_ = CurrentFrame.cy;

  Timer total(true);

  if (static_cast<int>(CurrentFrame.mvImagePyramid.size()) <= max_level_) {
    LOGE("Not enough pyramid levels");
    return false;
  }

  // Save valid points seen in last keyframe
  const set<MapPoint*> mappoints = LastKF->GetMapPoints();
  counter = 0;
  for (auto it=mappoints.begin(); it != mappoints.end() && counter<max_points; it++) {
    MapPoint* pMP = *it;
    Eigen::Vector3d p = pMP->GetWorldPos();
    points_.push_back(p);
    counter++;
  }

  size = points_.size();
  if (size == 0) {
    LOGE("No points to track!");
    return false;
  }

  patch_area = patch_size_*patch_size_;
  patch_cache_ = cv::Mat(size, patch_area, CV_32F);
  visible_pts_.resize(size, false);
  jacobian_cache_.resize(Eigen::NoChange, size*patch_area);

  // Initial displacement between frames.
  Eigen::Matrix4d current_se3 = CurrentFrame.GetPose() * LastKF->GetPoseInverse();
  Eigen::Matrix4d last_pose = LastKF->GetPose();

  for (int level=max_level_; level >= min_level_; level--) {
    jacobian_cache_.setZero();

    scale = CurrentFrame.mvInvScaleFactors[level];
    Optimize(CurrentFrame.mvImagePyramid[level], LastKF->mvImagePyramid[level], last_pose, current_se3, scale);

    // High error in max level means frames are not close, skip other levels
    if (fast && error_ > 0.01) {
      error_ = 1e10;
      return false;
    }
  }

  Eigen::Matrix4d pose = current_se3 * last_pose;
  CurrentFrame.SetPose(pose);

  if (!fast) {
    total.Stop();
    LOGD("Align time is %.2fms", total.GetMsTime());
  }

  return true;
}

bool ImageAlign::ComputePose(KeyFrame *CurrentKF, KeyFrame *LastKF) {
  int size, patch_area, counter;
  float scale;
  int max_points = 100;

  cam_fx_ = CurrentKF->fx;
  cam_fy_ = CurrentKF->fy;
  cam_cx_ = CurrentKF->cx;
  cam_cy_ = CurrentKF->cy;

  if (static_cast<int>(CurrentKF->mvImagePyramid.size()) <= max_level_) {
    LOGE("Not enough pyramid levels");
    return false;
  }

  // Save valid points seen in last keyframe
  const set<MapPoint*> mappoints = LastKF->GetMapPoints();
  counter = 0;
  for (auto it=mappoints.begin(); it != mappoints.end() && counter<max_points; it++) {
    MapPoint* pMP = *it;
    Eigen::Vector3d p = pMP->GetWorldPos();
    points_.push_back(p);
    counter++;
  }

  size = points_.size();
  if (size == 0) {
    LOGE("No points to track!");
    return false;
  }

  patch_area = patch_size_*patch_size_;
  patch_cache_ = cv::Mat(size, patch_area, CV_32F);
  visible_pts_.resize(size, false);
  jacobian_cache_.resize(Eigen::NoChange, size*patch_area);

  // Initial displacement between frames.
  Eigen::Matrix4d current_se3 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d last_pose = LastKF->GetPose();

  // Only last level
  int level = max_level_;
  jacobian_cache_.setZero();

  scale = 1.0/CurrentKF->mvScaleFactors[level];
  Optimize(CurrentKF->mvImagePyramid[level], LastKF->mvImagePyramid[level], last_pose, current_se3, scale);

  // High error in max level means frames are not close, skip other levels
  if (error_ > 0.03) {
    error_ = 1e10;
    return false;
  }

  return true;
}

void ImageAlign::Optimize(const cv::Mat &src, const cv::Mat &last_img, const Eigen::Matrix4d &last_pose,
                          Eigen::Matrix4d &se3, float scale) {
  Eigen::Matrix<double, 6, 1>  x;
  Eigen::Matrix4d se3_bk = se3;
  bool small = false;

  // Perform iterative estimation
  for (int i = 0; i < max_its_; i++) {
    H_.setZero();
    Jres_.setZero();

    // compute initial error
    n_meas_ = 0;
    double new_chi2 = ComputeResiduals(src, last_img, last_pose, se3, scale, i == 0);
    if (n_meas_ == 0)
      stop_ = true;

    // Solve linear system
    x = H_.ldlt().solve(Jres_);
    if (static_cast<bool>(std::isnan(static_cast<double>(x[0])))) {
      // Matrix was singular and could not be computed
      stop_ = true;
    }

    // Check if error increased since last iteration
    if ((i > 0 && new_chi2 > chi2_) || stop_) {
      se3 = se3_bk;  // rollback
      break;
    }

    // If error didn't decreased too much, stop optimization
    if (i > 0 && new_chi2 > chi2_*0.99)
      small = true;

    // Update se3
    se3_bk = se3;
    se3 = se3 * Exp(-x);

    chi2_ = new_chi2;

    // Stop when converged
    error_ = AbsMax(x);
    if (error_ <= 1e-10 || small)
      break;
  }
}

double ImageAlign::ComputeResiduals(const cv::Mat &src, const cv::Mat &last_img, const Eigen::Matrix4d &last_pose,
                                    const Eigen::Matrix4d &se3, float scale, bool patches) {
  Eigen::Vector2d p2d;
  int half_patch, patch_area, border;

  half_patch = patch_size_/2;
  patch_area = patch_size_*patch_size_;
  border = half_patch+1;

  // Compute patches only the first time
  if (patches)
    PrecomputePatches(last_img, last_pose, scale);

  Eigen::Matrix4d pose = se3 * last_pose;
  Eigen::Matrix3d R = pose.block<3,3>(0,0);
  Eigen::Vector3d T = pose.block<3,1>(0,3);

  float chi2 = 0.0;
  size_t counter = 0;
  vector<bool>::iterator vit = visible_pts_.begin();

  // Check each point detected in last image
  for (auto it=points_.begin(); it != points_.end(); it++, counter++, vit++) {
    Eigen::Vector3d p = *it;

    // check if point is within image
    if (!*vit)
      continue;

    // Project in current frame with candidate pose and check if it fits within image
    if(!Project(R, T, p, p2d))
      continue;

    const float u_cur = p2d(0)*scale;
    const float v_cur = p2d(1)*scale;
    const int u_last_i = floorf(u_cur);
    const int v_last_i = floorf(v_cur);
    if (u_last_i < 0 || v_last_i < 0 || u_last_i-border < 0 || v_last_i-border < 0 || u_last_i+border >= src.cols || v_last_i+border >= src.rows)
      continue;

    // compute bilateral interpolation weights for the current image
    const float subpix_u_cur = u_cur-u_last_i;
    const float subpix_v_cur = v_cur-v_last_i;
    const float w_last_tl = (1.0-subpix_u_cur) * (1.0-subpix_v_cur);
    const float w_last_tr = subpix_u_cur * (1.0-subpix_v_cur);
    const float w_last_bl = (1.0-subpix_u_cur) * subpix_v_cur;
    const float w_last_br = subpix_u_cur * subpix_v_cur;

    float* patch_cache_ptr = reinterpret_cast<float*>(patch_cache_.data) + patch_area*counter;
    size_t pixel_counter = 0;  // is used to compute the index of the cached jacobian

    for (int y=v_last_i-half_patch; y < v_last_i+half_patch; y++) {
      const uint8_t* row_ptr = src.ptr<uint8_t>(y);
      const uint8_t* row_next_ptr = src.ptr<uint8_t>(y+1);
      for (int x=u_last_i-half_patch; x < u_last_i+half_patch; x++, pixel_counter++, patch_cache_ptr++) {
        // compute residual
        const float intensity_cur = w_last_tl*row_ptr[x] + w_last_tr*row_ptr[x+1] + w_last_bl*row_next_ptr[x] + w_last_br*row_next_ptr[x+1];
        const float res = intensity_cur - (*patch_cache_ptr);

        float weight = 1.0;
        chi2 += res*res*weight;
        n_meas_++;

        // Compute Jacobian, weighted Hessian and weighted "steepest descend images" (times error)
        const Eigen::Matrix<double, 6, 1> J = jacobian_cache_.col(counter*patch_area + pixel_counter);
        H_.noalias() += J*J.transpose()*weight;
        Jres_.noalias() -= J*res*weight;
      }
    }
  }

  return chi2/n_meas_;
}

void ImageAlign::PrecomputePatches(const cv::Mat &src, const Eigen::Matrix4d &pose, float scale) {
  Eigen::Vector2d p2d;
  int half_patch, patch_area, border;

  half_patch = patch_size_/2;
  patch_area = patch_size_*patch_size_;
  border = half_patch+1;

  Eigen::Matrix3d R = pose.block<3,3>(0,0);
  Eigen::Vector3d T = pose.block<3,1>(0,3);

  size_t counter = 0;
  Eigen::Matrix<double, 2, 6> frame_jac;
  vector<bool>::iterator vit = visible_pts_.begin();

  // Check each point detected in last image
  for (auto it=points_.begin(); it != points_.end(); it++, counter++, vit++) {
    Eigen::Vector3d p = *it;

    // Project in last frame and check if it fits within image
    if(!Project(R, T, p, p2d))
      continue;

    const float u_ref = p2d(0)*scale;
    const float v_ref = p2d(1)*scale;
    const int u_first_i = floorf(u_ref);
    const int v_first_i = floorf(v_ref);
    if (u_first_i-border < 0 || v_first_i-border < 0 || u_first_i+border >= src.cols || v_first_i+border >= src.rows)
      continue;
    *vit = true;

    // Evaluate projection jacobian
    Eigen::Vector3d xyz = R*p+T;
    Jacobian3DToPlane(xyz, &frame_jac);

    // compute bilateral interpolation weights for reference image
    const float subpix_u_ref = u_ref-u_first_i;
    const float subpix_v_ref = v_ref-v_first_i;
    const float w_first_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
    const float w_first_tr = subpix_u_ref * (1.0-subpix_v_ref);
    const float w_first_bl = (1.0-subpix_u_ref) * subpix_v_ref;
    const float w_first_br = subpix_u_ref * subpix_v_ref;
    size_t pixel_counter = 0;
    float* cache_ptr = reinterpret_cast<float*>(patch_cache_.data) + patch_area*counter;

    for (int y=v_first_i-half_patch; y < v_first_i+half_patch; y++) {
      const uint8_t* row_ptr = src.ptr<uint8_t>(y);
      const uint8_t* row_prev_ptr = src.ptr<uint8_t>(y-1);
      const uint8_t* row_next_ptr = src.ptr<uint8_t>(y+1);
      const uint8_t* row_next2_ptr = src.ptr<uint8_t>(y+2);
      for (int x=u_first_i-half_patch; x < u_first_i+half_patch; x++, cache_ptr++, pixel_counter++) {
        // precompute interpolated reference patch color
        *cache_ptr = w_first_tl*row_ptr[x] + w_first_tr*row_ptr[x+1] + w_first_bl*row_next_ptr[x] + w_first_br*row_next_ptr[x+1];

        // we use the inverse compositional: thereby we can take the gradient always at the same position
        // get gradient of warped image (~gradient at warped position)
        float dx = 0.5f * ((w_first_tl*row_ptr[x+1] + w_first_tr*row_ptr[x+2] + w_first_bl*row_next_ptr[x+1] + w_first_br*row_next_ptr[x+2])
                          -(w_first_tl*row_ptr[x-1] + w_first_tr*row_ptr[x] + w_first_bl*row_next_ptr[x-1] + w_first_br*row_next_ptr[x]));
        float dy = 0.5f * ((w_first_tl*row_next_ptr[x] + w_first_tr*row_next_ptr[x+1] + w_first_bl*row_next2_ptr[x] + w_first_br*row_next2_ptr[x+1])
                          -(w_first_tl*row_prev_ptr[x] + w_first_tr*row_prev_ptr[x+1] + w_first_bl*row_ptr[x] + w_first_br*row_ptr[x+1]));

        // cache the jacobian
        jacobian_cache_.col(counter*patch_area + pixel_counter) = (dx*frame_jac.row(0) + dy*frame_jac.row(1))*(cam_fx_*scale);
      }
    }
  }
}

bool ImageAlign::Project(const Eigen::Matrix3d &R, const Eigen::Vector3d &T,
                         const Eigen::Vector3d &p, Eigen::Vector2d &res) {
  Eigen::Vector3d x3Dc = R*p+T;

  const double invzc = 1.0/x3Dc(2);
  if (invzc<0)
    return false;

  res(0) = cam_fx_*x3Dc(0)*invzc+cam_cx_;
  res(1) = cam_fy_*x3Dc(1)*invzc+cam_cy_;
  return true;
}

void ImageAlign::Jacobian3DToPlane(const Eigen::Vector3d &p, Eigen::Matrix<double, 2, 6> *J) {
  const double x = p(0);
  const double y = p(1);
  const double z_inv = 1./p(2);
  const double z_inv_2 = z_inv*z_inv;

  (*J)(0, 0) = -z_inv;                  // -1/z
  (*J)(0, 1) = 0.0;                     // 0
  (*J)(0, 2) = x*z_inv_2;               // x/z^2
  (*J)(0, 3) = y*(*J)(0, 2);            // x*y/z^2
  (*J)(0, 4) = -(1.0 + x*(*J)(0, 2));   // -(1.0 + x^2/z^2)
  (*J)(0, 5) = y*z_inv;                 // y/z

  (*J)(1, 0) = 0.0;                   // 0
  (*J)(1, 1) = -z_inv;                // -1/z
  (*J)(1, 2) = y*z_inv_2;             // y/z^2
  (*J)(1, 3) = 1.0 + y*(*J)(1, 2);    // 1.0 + y^2/z^2
  (*J)(1, 4) = -(*J)(0, 3);           // -x*y/z^2
  (*J)(1, 5) = -x*z_inv;              // x/z
}

double ImageAlign::AbsMax(const Eigen::VectorXd &v) {
  double max, abs;
  int size;

  size = v.size();
  max = -1;

  for (int i=0; i < size; i++) {
    abs = fabs(v(i));
    if (abs > max) {
      max = abs;
    }
  }
  return max;
}

Eigen::Matrix4d ImageAlign::Exp(const Eigen::Matrix<double, 6, 1> &update) {
  Eigen::Vector3d upsilon = update.head<3>();
  Eigen::Vector3d omega = update.tail<3>();

  double theta;
  Eigen::Quaterniond q = RotationExp(omega, &theta);
  Eigen::Matrix3d Omega = RotationHat(omega);
  Eigen::Matrix3d Omega_sq = Omega*Omega;
  Eigen::Matrix3d V;

  if (theta < 1e-10) {
    V = q.toRotationMatrix();
    // Note: That is an accurate expansion!
  } else {
    double theta_sq = theta*theta;
    V = (Eigen::Matrix3d::Identity()
         + (1-cos(theta))/(theta_sq)*Omega
         + (theta-sin(theta))/(theta_sq*theta)*Omega_sq);
  }

  Eigen::Vector3d t = V*upsilon;
  Eigen::Matrix3d rot = q.toRotationMatrix();
  Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
  res.block<3,3>(0,0) = rot;
  res.block<3,1>(0,3) = t;
  return res;
}

Eigen::Quaterniond ImageAlign::RotationExp(const Eigen::Vector3d &omega, double *theta) {
  *theta = omega.norm();
  double half_theta = 0.5*(*theta);

  double imag_factor;
  double real_factor = cos(half_theta);
  if ((*theta) < 1e-10) {
    double theta_sq = (*theta)*(*theta);
    double theta_po4 = theta_sq*theta_sq;
    imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
  } else {
    double sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta/(*theta);
  }

  return Eigen::Quaterniond(real_factor, imag_factor*omega.x(), imag_factor*omega.y(), imag_factor*omega.z());
}

Eigen::Matrix3d ImageAlign::RotationHat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d Omega;
  Omega <<  0, -v(2),  v(1)
      ,  v(2),     0, -v(0)
      , -v(1),  v(0),     0;
  return Omega;
}

}  // namespace SD_SLAM
