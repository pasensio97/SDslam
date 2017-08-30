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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include "System.h"
#include "Config.h"
#include "extra/timer.h"

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv) {
    if(argc != 4) {
        cerr << endl << "Usage: ./rgbd_tum path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[3]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty()) {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    } else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size()) {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Read parameters
    SD_SLAM::Config &config = SD_SLAM::Config::GetInstance();
    if (!config.ReadParameters(argv[1])) {
      cerr << "[ERROR] Config file contains errors" << endl;
      return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SD_SLAM::System SLAM(SD_SLAM::System::RGBD, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im, imD;
    for(int ni=0; ni<nImages; ni++) {
        // Read image and depthmap from file
        cout << "[INFO] Reading Frame " << string(argv[2])+"/"+vstrImageFilenamesRGB[ni] << endl;
        im = cv::imread(string(argv[2])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_GRAYSCALE);
        imD = cv::imread(string(argv[2])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(argv[2]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        SD_SLAM::Timer ttracking(true);

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(im,imD,tframe);

        ttracking.Stop();
        cout << "[INFO] Tracking time is " << ttracking.GetMsTime() << "ms" << endl;

        vTimesTrack[ni]=ttracking.GetTime();

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(vTimesTrack[ni]<T)
            usleep((T-vTimesTrack[ni])*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps) {
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof()) {
        string s;
        getline(fAssociation,s);
        if(!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
