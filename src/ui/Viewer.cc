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

#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <unistd.h>
#include "Config.h"

using std::mutex;
using std::unique_lock;

namespace SD_SLAM {

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking):
  mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
  mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false) {
}

void Viewer::Run() {
  int w, h, mw, iw, ih, ib;

  mbFinished = false;
  mbStopped = false;
  iw = Config::Width();
  ih = Config::Height();
  w = 1024;
  h = 768;
  mw = 175;
  ib = 10;

  pangolin::CreateWindowAndBind("SD-SLAM", w, h);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(mw));
  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
  pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
  pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
  pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
  pangolin::Var<bool> menuShowFrames("menu.Show Frames",true,true);
  pangolin::Var<double> menuImageScale("menu.Image Scale",0.5,0,1,false);
  pangolin::Var<bool> menuReset("menu.Reset",false,false);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(w,h,Config::ViewpointF(),Config::ViewpointF(),w/2,h/2,0.1,1000),
        pangolin::ModelViewLookAt(Config::ViewpointX(),Config::ViewpointY(),Config::ViewpointZ(), 0,0,0,0.0,-1.0, 0.0)
        );

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(mw), 1.0, -w/(float)h)
      .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  // Configure frame viewer
  pangolin::View& d_video = pangolin::Display("Frame").SetAspect(iw/(float)ih)
      .SetBounds(pangolin::Attach::Pix(ib), pangolin::Attach::Pix(ib+ih*menuImageScale),
                 pangolin::Attach::Pix(mw+ib), pangolin::Attach::Pix(mw+ib+iw*menuImageScale));
  pangolin::GlTexture textVideo(iw,ih,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

  bool bFollow = true;

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

    if (menuFollowCamera && bFollow) {
      s_cam.Follow(Twc);
    } else if (menuFollowCamera && !bFollow) {
      s_cam.SetModelViewMatrix(
        pangolin::ModelViewLookAt(Config::ViewpointX(),Config::ViewpointY(),Config::ViewpointZ(), 0,0,0,0.0,-1.0, 0.0));
      s_cam.Follow(Twc);
      bFollow = true;
    } else if (!menuFollowCamera && bFollow) {
      bFollow = false;
    }

    d_cam.Activate(s_cam);
    glClearColor(1.0f,1.0f,1.0f,1.0f);
    mpMapDrawer->DrawCurrentCamera(Twc);
    if (menuShowKeyFrames || menuShowGraph)
      mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
    if (menuShowPoints)
      mpMapDrawer->DrawMapPoints();

    // Get frame and show
    if (menuShowFrames) {
      cv::Mat im = mpFrameDrawer->DrawFrame();
      textVideo.Upload(im.data,GL_RGB,GL_UNSIGNED_BYTE);

      d_video.SetBounds(pangolin::Attach::Pix(ib), pangolin::Attach::Pix(ib+ih*menuImageScale),
                        pangolin::Attach::Pix(mw+ib), pangolin::Attach::Pix(mw+ib+iw*menuImageScale));
      d_video.Activate();
      glColor4f(1.0f,1.0f,1.0f,1.0f);
      textVideo.RenderToViewportFlipY();
    }

    pangolin::FinishFrame();

    if (menuReset) {
      menuShowGraph = true;
      menuShowKeyFrames = true;
      menuShowPoints = true;
      bFollow = true;
      menuFollowCamera = true;
      mpSystem->Reset();
      menuReset = false;
    }

    if (Stop()) {
      while (isStopped()) {
        usleep(3000);
      }
    }

    if (CheckFinish())
      break;
  }

  SetFinish();

	std::cout << "UI thread finished, exiting..." << std::endl;
	exit(1);
}

void Viewer::RequestFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  mbFinishRequested = true;
}

bool Viewer::CheckFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinishRequested;
}

void Viewer::SetFinish() {
  unique_lock<mutex> lock(mMutexFinish);
  mbFinished = true;
}

bool Viewer::isFinished() {
  unique_lock<mutex> lock(mMutexFinish);
  return mbFinished;
}

void Viewer::RequestStop() {
  unique_lock<mutex> lock(mMutexStop);
  if (!mbStopped)
    mbStopRequested = true;
}

bool Viewer::isStopped() {
  unique_lock<mutex> lock(mMutexStop);
  return mbStopped;
}

bool Viewer::Stop() {
  unique_lock<mutex> lock(mMutexStop);
  unique_lock<mutex> lock2(mMutexFinish);

  if (mbFinishRequested)
    return false;
  else if (mbStopRequested) {
    mbStopped = true;
    mbStopRequested = false;
    return true;
  }

  return false;

}

void Viewer::Release() {
  unique_lock<mutex> lock(mMutexStop);
  mbStopped = false;
}

}
