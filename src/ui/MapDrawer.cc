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

#include "MapDrawer.h"
#include "Config.h"

using std::vector;
using std::set;
using std::mutex;
using std::unique_lock;

namespace SD_SLAM {

MapDrawer::MapDrawer(Map* pMap): mpMap(pMap) {
}

void MapDrawer::DrawMapPoints() {
  const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
  const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

  set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

  if (vpMPs.empty())
    return;

  glPointSize(Config::PointSize());
  glBegin(GL_POINTS);
  glColor3f(0.0,0.0,0.0);

  for (size_t i=0, iend=vpMPs.size(); i<iend;i++) {
    if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
      continue;
    cv::Mat pos = vpMPs[i]->GetWorldPos();
    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
  }
  glEnd();

  glPointSize(Config::PointSize());
  glBegin(GL_POINTS);
  glColor3f(0.0,0.8,0.0);

  for (set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++) {
    if ((*sit)->isBad())
      continue;
    cv::Mat pos = (*sit)->GetWorldPos();
    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
  }

  glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
  const float &w = Config::KeyFrameSize();
  const float h = w*0.75;
  const float z = w*0.6;

  const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

  if (bDrawKF) {
    double lwidth = Config::KeyFrameLineWidth();
    for (size_t i=0; i<vpKFs.size(); i++) {
      KeyFrame* pKF = vpKFs[i];
      cv::Mat Twc = Converter::toCvMat(pKF->GetPoseInverse()).t();

      glPushMatrix();

      glMultMatrixf(Twc.ptr<GLfloat>(0));

      glLineWidth(lwidth);
      glColor3f(0.0f,0.0f,0.9f);
      glBegin(GL_LINES);
      glVertex3f(0,0,0);
      glVertex3f(w,h,z);
      glVertex3f(0,0,0);
      glVertex3f(w,-h,z);
      glVertex3f(0,0,0);
      glVertex3f(-w,-h,z);
      glVertex3f(0,0,0);
      glVertex3f(-w,h,z);

      glVertex3f(w,h,z);
      glVertex3f(w,-h,z);

      glVertex3f(-w,h,z);
      glVertex3f(-w,-h,z);

      glVertex3f(-w,h,z);
      glVertex3f(w,h,z);

      glVertex3f(-w,-h,z);
      glVertex3f(w,-h,z);
      glEnd();

      glPopMatrix();
    }
  }

  if (bDrawGraph) {
    glLineWidth(Config::GraphLineWidth());
    glColor4f(0.7f,0.0f,0.7f,0.6f);
    glBegin(GL_LINES);

    for (size_t i=0; i<vpKFs.size(); i++) {
      // Covisibility Graph
      const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
      Eigen::Vector3d Ow = vpKFs[i]->GetCameraCenter();
      if (!vCovKFs.empty()) {
        for (vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++) {
          if ((*vit)->mnId<vpKFs[i]->mnId)
            continue;
          Eigen::Vector3d Ow2 = (*vit)->GetCameraCenter();
          glVertex3f(Ow(0),Ow(1),Ow(2));
          glVertex3f(Ow2(0),Ow2(1),Ow2(2));
        }
      }

      // Spanning tree
      KeyFrame* pParent = vpKFs[i]->GetParent();
      if (pParent) {
        Eigen::Vector3d Owp = pParent->GetCameraCenter();
        glVertex3f(Ow(0),Ow(1),Ow(2));
        glVertex3f(Owp(0),Owp(1),Owp(2));
      }

      // Loops
      set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
      for (set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++) {
        if ((*sit)->mnId<vpKFs[i]->mnId)
          continue;
        Eigen::Vector3d Owl = (*sit)->GetCameraCenter();
        glVertex3f(Ow(0),Ow(1),Ow(2));
        glVertex3f(Owl(0),Owl(1),Owl(2));
      }
    }

    glEnd();
  }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
  const float &w = Config::CameraSize();
  const float h = w*0.75;
  const float z = w*0.6;

  glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

  glLineWidth(Config::CameraLineWidth());
  glColor3f(0.9f,0.0f,0.0f);
  glBegin(GL_LINES);
  glVertex3f(0,0,0);
  glVertex3f(w,h,z);
  glVertex3f(0,0,0);
  glVertex3f(w,-h,z);
  glVertex3f(0,0,0);
  glVertex3f(-w,-h,z);
  glVertex3f(0,0,0);
  glVertex3f(-w,h,z);

  glVertex3f(w,h,z);
  glVertex3f(w,-h,z);

  glVertex3f(-w,h,z);
  glVertex3f(-w,-h,z);

  glVertex3f(-w,h,z);
  glVertex3f(w,h,z);

  glVertex3f(-w,-h,z);
  glVertex3f(w,-h,z);
  glEnd();

  glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw) {
  unique_lock<mutex> lock(mMutexCamera);
  mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M) {
  if (!mCameraPose.empty()) {
    cv::Mat Rwc(3,3,CV_32F);
    cv::Mat twc(3,1,CV_32F);
    {
      unique_lock<mutex> lock(mMutexCamera);
      Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
      twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
    }

    M.m[0] = Rwc.at<float>(0,0);
    M.m[1] = Rwc.at<float>(1,0);
    M.m[2] = Rwc.at<float>(2,0);
    M.m[3]  = 0.0;

    M.m[4] = Rwc.at<float>(0,1);
    M.m[5] = Rwc.at<float>(1,1);
    M.m[6] = Rwc.at<float>(2,1);
    M.m[7]  = 0.0;

    M.m[8] = Rwc.at<float>(0,2);
    M.m[9] = Rwc.at<float>(1,2);
    M.m[10] = Rwc.at<float>(2,2);
    M.m[11]  = 0.0;

    M.m[12] = twc.at<float>(0);
    M.m[13] = twc.at<float>(1);
    M.m[14] = twc.at<float>(2);
    M.m[15]  = 1.0;
  } else
    M.SetIdentity();
}

}  // namespace SD_SLAM
