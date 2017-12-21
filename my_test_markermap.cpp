/**
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/

#include <aruco/aruco.h>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "pcdwriter.h"

class CmdLineParser {
  int argc;
  char **argv;
 public:
  CmdLineParser(int _argc, char **_argv) : argc(_argc), argv(_argv) {}

  //is the param?
  bool operator[](std::string param) {
    int idx = -1;
    for (int i = 0; i < argc && idx == -1; i++)if (std::string(argv[i]) == param) idx = i;
    return (idx != -1);
  }

  //return the value of a param using a default value if it is not present
  std::string operator()(std::string param, std::string defvalue = "-1") {
    int idx = -1;
    for (int i = 0; i < argc && idx == -1; i++)if (std::string(argv[i]) == param) idx = i;
    if (idx == -1) return defvalue; else return (argv[idx + 1]);
  }
};

int main(int argc, char **argv) {
  std::string TheMarkerMapConfigFile;
  float TheMarkerSize = -1;
  cv::VideoCapture TheVideoCapturer;
  cv::Mat TheInputImage, AnnotateImage;
  aruco::CameraParameters TheCameraParameters;
  aruco::MarkerMap TheMarkerMapConfig;
  aruco::MarkerDetector TheMarkerDetector;
  aruco::MarkerMapPoseTracker TheMSPoseTracker;
  std::map<int, cv::Mat> frame_pose_map;  // set of poses and the frames they were detected

  try {
    CmdLineParser cml(argc, argv);
    if (argc < 4 || cml["-h"]) {
      std::cerr << "Invalid number of arguments" << std::endl;
      std::cerr
          << "Usage: (in.avi|live[:camera_index(e.g 0 or 1)])) marksetconfig.yml camera_intrinsics.yml [optional_arguments]  "
              "\n\t[-s marker_size] \n\t[-pcd out_pcd_file_with_camera_poses] \n\t[-poses out_file_with_poses] "
              "\n\t[-mti value: minimum value in range (0,1) for the size of the detected markers. If 0, ] "
          << std::endl;
      return false;
    }
    TheMarkerMapConfig.readFromFile(argv[2]);

    TheMarkerMapConfigFile = argv[2];
    TheMarkerSize = stof(cml("-s", "1"));

    // read from camera or from  file
    std::string TheInputVideo = std::string(argv[1]);
    TheVideoCapturer.open(argv[1]);        // check video is open

    if (!TheVideoCapturer.isOpened()) {
      throw std::runtime_error("Could not open video");
    }

    // read first image to get the dimensions
    TheVideoCapturer >> TheInputImage;

    // read camera parameters if passed
    TheCameraParameters.readFromXMLFile(argv[3]);
    TheCameraParameters.resize(TheInputImage.size());

    // prepare the detector
    TheMarkerDetector.setDictionary(TheMarkerMapConfig.getDictionary());

    if (TheMarkerMapConfig.isExpressedInPixels() && TheMarkerSize > 0) {
      TheMarkerMapConfig = TheMarkerMapConfig.convertToMeters(TheMarkerSize);
    }

    if (!TheCameraParameters.isValid()) {
      std::cerr << "Invalid camera parameters" << std::endl;
    }
    else {
      TheMSPoseTracker.setParams(TheCameraParameters, TheMarkerMapConfig);
    }

    // Create gui
    char key = 0;
    int index = 0;

    do {
      TheVideoCapturer.retrieve(TheInputImage);
      TheInputImage.copyTo(AnnotateImage);
      index++;  // number of images captured

      // Detection of the markers
      std::vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(TheInputImage);

      // estimate 3d camera pose if possible
      if (TheMSPoseTracker.isValid()) {
        if (TheMSPoseTracker.estimatePose(detected_markers)) {
          frame_pose_map.insert(std::make_pair(index, TheMSPoseTracker.getRTMatrix()));
        }
      }

      // print the markers detected that belongs to the markerset
      for (int idx : TheMarkerMapConfig.getIndices(detected_markers)) {
        detected_markers[idx].draw(AnnotateImage, cv::Scalar(0, 0, 255), 1);
      }


      // show information
//      cv::imshow("annotated frames", AnnotateImage);
//      cv::waitKey(66);

    } while (TheVideoCapturer.grab());

    savePCDFile(cml("-pcd", "poses.pcd"), TheMarkerMapConfig, frame_pose_map);
  }
  catch (std::exception &ex) {
    std::cout << "Exception :" << ex.what() << std::endl;
  }
}
