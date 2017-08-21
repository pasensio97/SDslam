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

#include "Map.h"
#include <mutex>

using std::mutex;
using std::unique_lock;
using std::vector;
using std::set;

namespace ORB_SLAM2 {

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0) {
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
  unique_lock<mutex> lock(mMutexMap);
  mspKeyFrames.insert(pKF);
  if (pKF->mnId>mnMaxKFid)
    mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
  unique_lock<mutex> lock(mMutexMap);
  mspMapPoints.erase(pMP);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
  unique_lock<mutex> lock(mMutexMap);
  mspKeyFrames.erase(pKF);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
  unique_lock<mutex> lock(mMutexMap);
  mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
  unique_lock<mutex> lock(mMutexMap);
  mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
  unique_lock<mutex> lock(mMutexMap);
  return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
  unique_lock<mutex> lock(mMutexMap);
  return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
  unique_lock<mutex> lock(mMutexMap);
  return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
  unique_lock<mutex> lock(mMutexMap);
  return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
  unique_lock<mutex> lock(mMutexMap);
  return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
  unique_lock<mutex> lock(mMutexMap);
  return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
  unique_lock<mutex> lock(mMutexMap);
  return mnMaxKFid;
}

void Map::clear()
{
  for (set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
    delete *sit;

  for (set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    delete *sit;

  mspMapPoints.clear();
  mspKeyFrames.clear();
  mnMaxKFid = 0;
  mvpReferenceMapPoints.clear();
  mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
