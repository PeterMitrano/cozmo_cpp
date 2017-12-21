#include <cerrno>
#include <iostream>
#include <fstream>

#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include "pcdwriter.h"

void show_help();

int main(int argc, char **argv) {

  if (argc < 5) {
    show_help();
    return 0;
  }

  unsigned int num_maps = (unsigned int) argc - 4;
  std::string video_filename(argv[1]);
  std::string camera_params_filename(argv[2]);
  std::string pcd_filename(argv[argc - 1]);
  std::map<int, cv::Mat> frame_pose_map;
  std::vector<aruco::MarkerMap> maps(num_maps, aruco::MarkerMap());

  cv::VideoCapture capture(video_filename);

  if (!capture.isOpened()) {
    std::cerr << "failed to open file: [" << strerror(errno) << "]" << std::endl;
    return -1;
  }

  aruco::CameraParameters camera_params;
  camera_params.readFromXMLFile(camera_params_filename);
  camera_params.resize(cv::Size(320, 240));  // I'm so bad

  std::ofstream pcd_file(pcd_filename);
  if (!pcd_file.good()) {
    std::cerr << strerror(errno) << std::endl;
    return -1;
  }

  // open all the marker maps
  for (int i = 0; i < num_maps; i++) {
    std::string map_filename(argv[i + 3]);
    // construct and call read from file
    maps[i].readFromFile(map_filename);
  }

  // check the camera configuration file
  if (!camera_params.isValid()) {
    std::cerr << "Invalid camera parameters" << std::endl;
    return -1;
  }

  std::vector<aruco::MarkerMapPoseTracker> trackers(num_maps, aruco::MarkerMapPoseTracker());
  for (size_t i = 0; i < num_maps; i++) {
    aruco::MarkerMapPoseTracker &tracker = trackers[i];
    aruco::MarkerMap &map = maps[i];

    // convert to meters if necessary
    if (map.isExpressedInPixels()) {
      map = map.convertToMeters(0.02);
    }

    tracker.setParams(camera_params, map);
  }

  cv::Mat frame, annotated_frame;
  aruco::MarkerDetector detector;
  detector.setDictionary(maps[0].getDictionary()); // this assumes they all have the same dictionary

  int frame_index = 0;
  while (true) {
    capture >> frame;

    if (frame.empty()) {
      break;
    }

    frame.copyTo(annotated_frame);

    // Detection of the markers
    std::vector<aruco::Marker> detected_markers = detector.detect(frame);

    // estimate 3d camera pose if possible
    for (size_t i = 0; i < num_maps; i++) {
      aruco::MarkerMapPoseTracker tracker = trackers[i];
      aruco::MarkerMap map = maps[i];

      if (tracker.isValid()) {
        if (tracker.estimatePose(detected_markers)) {
          frame_pose_map.insert(std::make_pair(frame_index, tracker.getRTMatrix()));
        }
      }

      // annotate the video feed
      for (int idx : map.getIndices(detected_markers)) {
        detected_markers[idx].draw(annotated_frame, cv::Scalar(0, 0, 255), 1);
      }
    }

    // show information
//    cv::imshow("annotated frames", annotated_frame);
//    cv::waitKey(66);

    ++frame_index;
  }

  savePCDFile(pcd_filename, maps, frame_pose_map);

  std::cout << "Writing " << pcd_filename << std::endl;

  return 0;
}

void show_help() {
  std::cout << "USAGE: extract_poses_from_video video.avi output_poses.pcd"
            << std::endl
            << "    video.avi - the video file recorded on cozmo"
            << std::endl
            << "    map1 map2 map3 - a bunch of map config yml files"
            << std::endl
            << "    camera_params.yml - camera calibration file. run aruco_calibration program to get one."
            << std::endl
            << "    output_poses.pcd - the pcd (point cloud data) file to fill with poses"
            << std::endl
            << std::endl;

}
