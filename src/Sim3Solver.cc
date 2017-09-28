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

#include "Sim3Solver.h"
#include <cmath>
#include <opencv2/core/core.hpp>
#include "ORBmatcher.h"
#include "extra/utils.h"
#include "Converter.h"

using std::vector;

namespace SD_SLAM {

Sim3Solver::Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2, const vector<MapPoint *> &vpMatched12, const bool bFixScale):
  mnIterations(0), mnBestInliers(0), mbFixScale(bFixScale) {
  mpKF1 = pKF1;
  mpKF2 = pKF2;

  vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

  mN1 = vpMatched12.size();

  mvpMapPoints1.reserve(mN1);
  mvpMapPoints2.reserve(mN1);
  mvpMatches12 = vpMatched12;
  mvnIndices1.reserve(mN1);
  mvX3Dc1.reserve(mN1);
  mvX3Dc2.reserve(mN1);

  Eigen::Matrix3d Rcw1 = pKF1->GetRotation();
  Eigen::Vector3d tcw1 = pKF1->GetTranslation();
  Eigen::Matrix3d Rcw2 = pKF2->GetRotation();
  Eigen::Vector3d tcw2 = pKF2->GetTranslation();

  mvAllIndices.reserve(mN1);

  size_t idx=0;
  for (int i1=0; i1<mN1; i1++) {
    if (vpMatched12[i1]) {
      MapPoint* pMP1 = vpKeyFrameMP1[i1];
      MapPoint* pMP2 = vpMatched12[i1];

      if (!pMP1)
        continue;

      if (pMP1->isBad() || pMP2->isBad())
        continue;

      int indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);
      int indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);

      if (indexKF1<0 || indexKF2<0)
        continue;

      const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
      const cv::KeyPoint &kp2 = pKF2->mvKeysUn[indexKF2];

      const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
      const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];

      mvnMaxError1.push_back(9.210*sigmaSquare1);
      mvnMaxError2.push_back(9.210*sigmaSquare2);

      mvpMapPoints1.push_back(pMP1);
      mvpMapPoints2.push_back(pMP2);
      mvnIndices1.push_back(i1);

      Eigen::Vector3d X3D1w = pMP1->GetWorldPos();
      Eigen::Vector3d pos1 = Rcw1*X3D1w+tcw1;
      mvX3Dc1.push_back(pos1);

      Eigen::Vector3d X3D2w = pMP2->GetWorldPos();
      Eigen::Vector3d pos2 = Rcw2*X3D2w+tcw2;
      mvX3Dc2.push_back(pos2);

      mvAllIndices.push_back(idx);
      idx++;
    }
  }

  mK1 = pKF1->mK;
  mK2 = pKF2->mK;

  FromCameraToImage(mvX3Dc1, mvP1im1, mK1);
  FromCameraToImage(mvX3Dc2, mvP2im2, mK2);

  SetRansacParameters();
}

void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations) {
  mRansacProb = probability;
  mRansacMinInliers = minInliers;
  mRansacMaxIts = maxIterations;  

  N = mvpMapPoints1.size(); // number of correspondences

  mvbInliersi.resize(N);

  // Adjust Parameters according to number of correspondences
  float epsilon = (float)mRansacMinInliers/N;

  // Set RANSAC iterations according to probability, epsilon, and max iterations
  int nIterations;

  if (mRansacMinInliers==N)
    nIterations=1;
  else
    nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,3)));

  mRansacMaxIts = std::max(1, std::min(nIterations,mRansacMaxIts));

  mnIterations = 0;
}

Eigen::Matrix4d Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers) {
  bNoMore = false;
  vbInliers = vector<bool>(mN1,false);
  nInliers=0;

  if (N<mRansacMinInliers) {
    bNoMore = true;
    return Eigen::Matrix4d::Zero();
  }

  vector<size_t> vAvailableIndices;

  Eigen::Matrix3d P3Dc1i;
  Eigen::Matrix3d P3Dc2i;

  int nCurrentIterations = 0;
  while (mnIterations<mRansacMaxIts && nCurrentIterations<nIterations) {
    nCurrentIterations++;
    mnIterations++;

    vAvailableIndices = mvAllIndices;

    // Get min set of points
    for (short i = 0; i < 3; ++i) {
      int randi = Random(0, vAvailableIndices.size()-1);

      int idx = vAvailableIndices[randi];

      P3Dc1i.col(i) = mvX3Dc1[idx];
      P3Dc2i.col(i) = mvX3Dc2[idx];

      vAvailableIndices[randi] = vAvailableIndices.back();
      vAvailableIndices.pop_back();
    }

    ComputeSim3(P3Dc1i, P3Dc2i);

    CheckInliers();

    if (mnInliersi>=mnBestInliers) {
      mvbBestInliers = mvbInliersi;
      mnBestInliers = mnInliersi;
      mBestT12 = mT12i;
      mBestRotation = mR12i;
      mBestTranslation = mt12i;
      mBestScale = ms12i;

      if (mnInliersi>mRansacMinInliers) {
        nInliers = mnInliersi;
        for (int i=0; i<N; i++)
          if (mvbInliersi[i])
            vbInliers[mvnIndices1[i]] = true;
        return mBestT12;
      }
    }
  }

  if (mnIterations>=mRansacMaxIts)
    bNoMore=true;

  return Eigen::Matrix4d::Zero();
}

Eigen::Matrix4d Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers) {
  bool bFlag;
  return iterate(mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

void Sim3Solver::ComputeCentroid(const Eigen::Matrix3d &P, Eigen::Matrix3d &Pr, Eigen::Vector3d &C) {
  C(0) = P.row(0).sum();
  C(1) = P.row(1).sum();
  C(2) = P.row(2).sum();
  C = C/P.cols();

  for (int i=0; i<P.cols(); i++) {
    Pr.col(i)=P.col(i)-C;
  }
}

void Sim3Solver::ComputeSim3(const Eigen::Matrix3d &P1, const Eigen::Matrix3d &P2) {
  // Custom implementation of:
  // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

  // Step 1: Centroid and relative coordinates

  Eigen::Matrix3d Pr1; // Relative coordinates to centroid (set 1)
  Eigen::Matrix3d Pr2; // Relative coordinates to centroid (set 2)
  Eigen::Vector3d O1; // Centroid of P1
  Eigen::Vector3d O2; // Centroid of P2

  ComputeCentroid(P1, Pr1, O1);
  ComputeCentroid(P2, Pr2, O2);

  // Step 2: Compute M matrix

  Eigen::Matrix3d M = Pr2*Pr1.transpose();

  // Step 3: Compute N matrix

  double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

  Eigen::Matrix4d N;

  N11 = M(0,0)+M(1,1)+M(2,2);
  N12 = M(1,2)-M(2,1);
  N13 = M(2,0)-M(0,2);
  N14 = M(0,1)-M(1,0);
  N22 = M(0,0)-M(1,1)-M(2,2);
  N23 = M(0,1)+M(1,0);
  N24 = M(2,0)+M(0,2);
  N33 = -M(0,0)+M(1,1)-M(2,2);
  N34 = M(1,2)+M(2,1);
  N44 = -M(0,0)-M(1,1)+M(2,2);

  N << N11, N12, N13, N14,
       N12, N22, N23, N24,
       N13, N23, N33, N34,
       N14, N24, N34, N44;

  // Step 4: Eigenvector of the highest eigenvalue

  cv::Mat eval, evec;

  cv::eigen(Converter::toCvMat(N),eval,evec); //evec[0] is the quaternion of the desired rotation

  cv::Mat vec(1,3,evec.type());
  (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

  // Rotation angle. sin is the norm of the imaginary part, cos is the real part
  double ang=atan2(norm(vec),evec.at<float>(0,0));

  vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

  cv::Mat mR12i_cv(3,3,CV_32F);
  cv::Rodrigues(vec, mR12i_cv); // computes the rotation matrix from angle-axis
  mR12i = Converter::toMatrix3d(mR12i_cv);

  // Step 5: Rotate set 2

  Eigen::Matrix3d P3 = mR12i*Pr2;

  // Step 6: Scale

  if (!mbFixScale) {
    double nom = Converter::toCvMat(Pr1).dot(Converter::toCvMat(P3));
    Eigen::Matrix3d aux_P3 = P3.array().pow(2);
    double den = 0;

    for (int i=0; i<aux_P3.rows(); i++) {
      for (int j=0; j<aux_P3.cols(); j++) {
        den+=aux_P3(i,j);
      }
    }

    ms12i = nom/den;
  } else
    ms12i = 1.0f;

  // Step 7: Translation

  mt12i = O1 - ms12i*mR12i*O2;

  // Step 8: Transformation

  // Step 8.1 T12
  mT12i.setIdentity();

  Eigen::Matrix3d sR = ms12i*mR12i;

  mT12i.block<3,3>(0,0) = sR;
  mT12i.block<3,1>(0,3) = mt12i;

  // Step 8.2 T21

  mT21i.setIdentity();

  Eigen::Matrix3d sRinv = (1.0/ms12i)*mR12i.transpose();

  mT21i.block<3,3>(0,0) = sRinv;
  Eigen::Vector3d tinv = -sRinv*mt12i;
  mT21i.block<3,1>(0,3) = tinv;
}


void Sim3Solver::CheckInliers() {
  vector<Eigen::Vector2d> vP1im2, vP2im1;
  Project(mvX3Dc2, vP2im1, mT12i, mK1);
  Project(mvX3Dc1, vP1im2, mT21i, mK2);

  mnInliersi=0;

  for (size_t i=0; i<mvP1im1.size(); i++) {
    Eigen::Vector2d dist1 = mvP1im1[i]-vP2im1[i];
    Eigen::Vector2d dist2 = vP1im2[i]-mvP2im2[i];

    const float err1 = dist1.dot(dist1);
    const float err2 = dist2.dot(dist2);

    if (err1<mvnMaxError1[i] && err2<mvnMaxError2[i]) {
      mvbInliersi[i]=true;
      mnInliersi++;
    } else
      mvbInliersi[i]=false;
  }
}


Eigen::Matrix3d Sim3Solver::GetEstimatedRotation() {
  return mBestRotation;
}

Eigen::Vector3d Sim3Solver::GetEstimatedTranslation() {
  return mBestTranslation;
}

float Sim3Solver::GetEstimatedScale() {
  return mBestScale;
}

void Sim3Solver::Project(const vector<Eigen::Vector3d> &vP3Dw, vector<Eigen::Vector2d> &vP2D, const Eigen::Matrix4d &Tcw, const Eigen::Matrix3d &K) {
  Eigen::Matrix3d Rcw = Tcw.block<3,3>(0,0);
  Eigen::Vector3d tcw = Tcw.block<3,1>(0,3);
  const float fx = K(0,0);
  const float fy = K(1,1);
  const float cx = K(0,2);
  const float cy = K(1,2);

  vP2D.clear();
  vP2D.reserve(vP3Dw.size());

  for (size_t i=0, iend=vP3Dw.size(); i<iend; i++) {
    Eigen::Vector3d P3Dc = Rcw*vP3Dw[i]+tcw;
    const float invz = 1/(P3Dc(2));
    const float x = P3Dc(0)*invz;
    const float y = P3Dc(1)*invz;

    vP2D.push_back(Eigen::Vector2d(fx*x+cx, fy*y+cy));
  }
}

void Sim3Solver::FromCameraToImage(const vector<Eigen::Vector3d> &vP3Dc, vector<Eigen::Vector2d> &vP2D, const Eigen::Matrix3d &K) {
  const float fx = K(0,0);
  const float fy = K(1,1);
  const float cx = K(0,2);
  const float cy = K(1,2);

  vP2D.clear();
  vP2D.reserve(vP3Dc.size());

  for (size_t i=0, iend=vP3Dc.size(); i<iend; i++) {
    const float invz = 1/(vP3Dc[i](2));
    const float x = vP3Dc[i](0)*invz;
    const float y = vP3Dc[i](1)*invz;

    vP2D.push_back(Eigen::Vector2d(fx*x+cx, fy*y+cy));
  }
}

}  // namespace SD_SLAM
