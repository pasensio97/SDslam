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

using std::vector;
using std::cout;
using std::cerr;
using std::endl;

namespace ORB_SLAM2 {

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

bool ImageAlign::ComputePose(Frame &CurrentFrame, const Frame &LastFrame, bool fast) {
  int patch_area;

  if (static_cast<int>(LastFrame.mvImagePyramid.size()) <= max_level_) {
    cerr << "[ERROR] Not enough pyramid levels" << endl;
    return false;
  }

  patch_area = patch_size_*patch_size_;
  patch_cache_ = cv::Mat(LastFrame.N, patch_area, CV_32F);
  visible_fts_.resize(LastFrame.N, false);
  jacobian_cache_.resize(Eigen::NoChange, LastFrame.N*patch_area);

  // Initial displacement between frames.
  cv::Mat current_se3 = CurrentFrame.GetPose() * LastFrame.GetPoseInverse();

  for (int level=max_level_; level >= min_level_; level--) {
    jacobian_cache_.setZero();
    Optimize(CurrentFrame, LastFrame, current_se3, level);

    // High error in max level means frames are not close, skip other levels
    if (fast && error_ > 0.01) {
      error_ = 1e10;
      break;
    }
  }

  CurrentFrame.SetPose(current_se3 * LastFrame.GetPose());

  return true;
}

void ImageAlign::Optimize(const Frame &CurrentFrame, const Frame &LastFrame, cv::Mat &se3, int level) {
  Eigen::Matrix<double, 6, 1>  x;
  cv::Mat se3_bk = se3.clone();

  // Perform iterative estimation
  for (int i = 0; i < max_its_; i++) {
    H_.setZero();
    Jres_.setZero();

    // compute initial error
    n_meas_ = 0;
    double new_chi2 = ComputeResiduals(CurrentFrame, LastFrame, se3, level, true, i == 0);
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
      se3 = se3_bk.clone();  // rollback
      break;
    }

    // Update se3
    se3_bk = se3.clone();
    se3 = se3 * Exp(-x);

    chi2_ = new_chi2;

    // Stop when converged
    error_ = AbsMax(x);
    if (error_ <= 1e-10)
      break;
  }
}

double ImageAlign::ComputeResiduals(const Frame &CurrentFrame, const Frame &LastFrame, const cv::Mat &se3,
                                      int level, bool linearize, bool patches) {
  Eigen::Vector2d p2d;
  int half_patch, patch_area, border;
  float scale;

  half_patch = patch_size_/2;
  patch_area = patch_size_*patch_size_;
  border = half_patch+1;

  scale = CurrentFrame.mvInvScaleFactors[level];
  const cv::Mat& last_img = CurrentFrame.mvImagePyramid[level];

  // Compute patches only the first time
  if (patches)
    PrecomputePatches(LastFrame, level);

  cv::Mat pose = se3 * LastFrame.GetPose();
  float chi2 = 0.0;
  size_t counter = 0;
  vector<bool>::iterator vit = visible_fts_.begin();

  // Check each point detected in last image
  for (int i=0; i<LastFrame.N; i++, counter++, vit++) {
    MapPoint* pMP = LastFrame.mvpMapPoints[i];
    if (!pMP || LastFrame.mvbOutlier[i])
      continue;

    // check if point is within image
    if (!*vit)
      continue;

    // Project in current frame with candidate pose and check if it fits within image
    if(!Project(CurrentFrame, pose, pMP, p2d))
      continue;

    const float u_cur = p2d(0)*scale;
    const float v_cur = p2d(1)*scale;
    const int u_last_i = floorf(u_cur);
    const int v_last_i = floorf(v_cur);
    if (u_last_i < 0 || v_last_i < 0 || u_last_i-border < 0 || v_last_i-border < 0 || u_last_i+border >= last_img.cols || v_last_i+border >= last_img.rows)
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
      const uint8_t* row_ptr = last_img.ptr<uint8_t>(y);
      const uint8_t* row_next_ptr = last_img.ptr<uint8_t>(y+1);
      for (int x=u_last_i-half_patch; x < u_last_i+half_patch; x++, pixel_counter++, patch_cache_ptr++) {
        // compute residual
        const float intensity_cur = w_last_tl*row_ptr[x] + w_last_tr*row_ptr[x+1] + w_last_bl*row_next_ptr[x] + w_last_br*row_next_ptr[x+1];
        const float res = intensity_cur - (*patch_cache_ptr);

        float weight = 1.0;
        chi2 += res*res*weight;
        n_meas_++;

        if (linearize) {
          // compute Jacobian, weighted Hessian and weighted "steepest descend images" (times error)
          const Eigen::Matrix<double, 6, 1> J = jacobian_cache_.col(counter*patch_area + pixel_counter);
          H_.noalias() += J*J.transpose()*weight;
          Jres_.noalias() -= J*res*weight;
        }
      }
    }
  }

  return chi2/n_meas_;
}

void ImageAlign::PrecomputePatches(const Frame &LastFrame, int level) {
  Eigen::Vector2d p2d;
  int half_patch, patch_area, border;
  float scale;

  half_patch = patch_size_/2;
  patch_area = patch_size_*patch_size_;
  border = half_patch+1;

  scale = LastFrame.mvInvScaleFactors[level];
  const cv::Mat& first_img = LastFrame.mvImagePyramid[level];

  size_t counter = 0;
  Eigen::Matrix<double, 2, 6> frame_jac;
  vector<bool>::iterator vit = visible_fts_.begin();

  // Check each point detected in last image
  for (int i=0; i<LastFrame.N; i++, counter++, vit++) {
    MapPoint* pMP = LastFrame.mvpMapPoints[i];
    if (!pMP || LastFrame.mvbOutlier[i])
      continue;

    // Project in last frame and check if it fits within image
    if(!Project(LastFrame, LastFrame.mTcw, pMP, p2d))
      continue;

    const float u_ref = p2d(0)*scale;
    const float v_ref = p2d(1)*scale;
    const int u_first_i = floorf(u_ref);
    const int v_first_i = floorf(v_ref);
    if (u_first_i-border < 0 || v_first_i-border < 0 || u_first_i+border >= first_img.cols || v_first_i+border >= first_img.rows)
      continue;
    *vit = true;

    // 3D point in frame coordinates
    cv::Mat Rcw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = LastFrame.mTcw.rowRange(0,3).col(3);
    cv::Mat x3Dw = pMP->GetWorldPos();
    cv::Mat x3Dc = Rcw*x3Dw+tcw;
    Eigen::Vector3d xyz(x3Dc.at<float>(0), x3Dc.at<float>(1), x3Dc.at<float>(2));

    // Evaluate projection jacobian
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
      const uint8_t* row_ptr = first_img.ptr<uint8_t>(y);
      const uint8_t* row_prev_ptr = first_img.ptr<uint8_t>(y-1);
      const uint8_t* row_next_ptr = first_img.ptr<uint8_t>(y+1);
      const uint8_t* row_next2_ptr = first_img.ptr<uint8_t>(y+2);
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
        jacobian_cache_.col(counter*patch_area + pixel_counter) = (dx*frame_jac.row(0) + dy*frame_jac.row(1))*(LastFrame.fx / LastFrame.mvScaleFactors[level]);
      }
    }
  }
}

bool ImageAlign::Project(const Frame &frame, const cv::Mat &se3, MapPoint *point, Eigen::Vector2d &res) {
  cv::Mat Rcw = se3.rowRange(0,3).colRange(0,3);
  cv::Mat tcw = se3.rowRange(0,3).col(3);
  cv::Mat x3Dw = point->GetWorldPos();
  cv::Mat x3Dc = Rcw*x3Dw+tcw;

  const float xc = x3Dc.at<float>(0);
  const float yc = x3Dc.at<float>(1);
  const float invzc = 1.0/x3Dc.at<float>(2);

  if (invzc<0)
    return false;

  res(0) = frame.fx*xc*invzc+frame.cx;
  res(1) = frame.fy*yc*invzc+frame.cy;
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

cv::Mat ImageAlign::Exp(const Eigen::Matrix<double, 6, 1> &update) {
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
  cv::Mat res = cv::Mat::eye(4,4,CV_32F);
  res.at<float>(0,0) = rot(0,0);
  res.at<float>(0,1) = rot(0,1);
  res.at<float>(0,2) = rot(0,2);
  res.at<float>(0,3) = t(0);
  res.at<float>(1,0) = rot(1,0);
  res.at<float>(1,1) = rot(1,1);
  res.at<float>(1,2) = rot(1,2);
  res.at<float>(1,3) = t(1);
  res.at<float>(2,0) = rot(2,0);
  res.at<float>(2,1) = rot(2,1);
  res.at<float>(2,2) = rot(2,2);
  res.at<float>(2,3) = t(2);
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

}  // namespace ORB_SLAM2
