/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/StdVector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "rovio/RovioFilter.hpp"
#include "rovio/Camera.hpp"
#include "rovio/FeatureCoordinates.hpp"
#include "rovio/FeatureDistance.hpp"
#include "rovio/ImagePyramid.hpp"

#ifdef MAKE_SCENE
#include "rovio/RovioScene.hpp"
#endif

#ifdef ROVIO_NMAXFEATURE
static constexpr int nMax_ = ROVIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
#endif

#ifdef ROVIO_NLEVELS
static constexpr int nLevels_ = ROVIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // Total number of pyramid levels considered.
#endif

#ifdef ROVIO_PATCHSIZE
static constexpr int patchSize_ = ROVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 6; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVIO_NCAM
static constexpr int nCam_ = ROVIO_NCAM;
#else
static constexpr int nCam_ = 1; // Used total number of cameras.
#endif

#ifdef ROVIO_NPOSE
static constexpr int nPose_ = ROVIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif

typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;

#ifdef MAKE_SCENE
static rovio::RovioScene<mtFilter> mRovioScene;

void idleFunc(){
    mRovioScene.drawScene(mRovioScene.mpFilter_->safe_);
}
#endif

class RovioStandalone {
public:
    RovioStandalone(const std::string& config_file) {
        mpFilter = std::make_shared<mtFilter>();
        mpFilter->readFromInfo(config_file);
        mpFilter->refreshProperties();
    }

    void processImage(const cv::Mat& image, double timestamp) {
        // TODO: Implement image processing pipeline
        // This would replace the ROS subscriber callbacks
    }

    void reset() {
        mpFilter->reset();
    }

private:
    std::shared_ptr<mtFilter> mpFilter;
};

void printUsage(const char* progName) {
    std::cout << "Usage: " << progName << " <config_file> [options]\n"
              << "Options:\n"
              << "  --camera <index>    Camera index (default: 0)\n"
              << "  --video <path>      Video file path\n"
              << "  --help              Show this help message\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string config_file = argv[1];
    std::string video_source = "";
    int camera_index = 0;

    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--camera" && i + 1 < argc) {
            camera_index = std::stoi(argv[++i]);
        } else if (arg == "--video" && i + 1 < argc) {
            video_source = argv[++i];
        } else if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
    }

    try {
        RovioStandalone rovio(config_file);
        cv::VideoCapture cap;

        if (!video_source.empty()) {
            cap.open(video_source);
        } else {
            cap.open(camera_index);
        }

        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open video source\n";
            return 1;
        }

        cv::Mat frame;
        double timestamp = 0.0;
        
#ifdef MAKE_SCENE
        // Initialize visualization if enabled
        std::string mVSFileName = "shaders/shader.vs";
        std::string mFSFileName = "shaders/shader.fs";
        mRovioScene.initScene(argc, argv, mVSFileName, mFSFileName, rovio.mpFilter);
        mRovioScene.setIdleFunction(idleFunc);
        glutMainLoop();
#else
        while (cap.read(frame)) {
            timestamp = cv::getTickCount() / cv::getTickFrequency();
            rovio.processImage(frame, timestamp);
            
            cv::imshow("ROVIO", frame);
            char key = cv::waitKey(1);
            if (key == 27) // ESC key
                break;
            else if (key == 'r')
                rovio.reset();
        }
#endif

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
