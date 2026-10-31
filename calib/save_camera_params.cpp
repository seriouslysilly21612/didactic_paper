/*****************************************************************************
*	Name: save_camera_params.cpp
*	Description: The intrinsic parameters of the color camera of a RealSense
*	(paper, Sec. V-C), read from the device with ViSP and saved as a camera
*	XML of ViSP, for ViSP tools such as its AprilTag pose estimation. The
*	calibration of this repository does not need it: calib/calib_grab.py
*	saves the factory intrinsics that the camera pipeline publishes
*	(camera_info.yaml), and calib/calib_cpo.py uses those.
*
*	    ./save_camera_params [camera.xml] [name] [width] [height]
*	    (defaults: camera.xml, RealSense_color, 640, 480; make VISP_DIR=...)
*****************************************************************************/
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    const std::string output  = (argc > 1) ? argv[1] : "camera.xml";
    const std::string camName = (argc > 2) ? argv[2] : "RealSense_color";
    const int width           = (argc > 3) ? std::stoi(argv[3]) : 640;
    const int height          = (argc > 4) ? std::stoi(argv[4]) : 480;

    // the intrinsics belong to a resolution: open the color stream at it
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, 30);

    vpRealSense2 rs;
    rs.open(cfg);

    // the resolution the device has actually opened, read back for the file
    const rs2_intrinsics intr = rs.getIntrinsics(RS2_STREAM_COLOR);
    const int actualWidth  = intr.width;
    const int actualHeight = intr.height;

    vpCameraParameters cam = rs.getCameraParameters(
        RS2_STREAM_COLOR,
        vpCameraParameters::perspectiveProjWithDistortion);

    std::cout << "[save_camera_params] camera: " << camName
              << "  " << actualWidth << "x" << actualHeight << "\n"
              << cam << "\n";

    vpXmlParserCamera parser;
    if (parser.save(cam, output, camName, actualWidth, actualHeight) != vpXmlParserCamera::SEQUENCE_OK)
    {
        std::cerr << "[save_camera_params] cannot save " << output << "\n";
        return 1;
    }

    std::cout << "[save_camera_params] saved " << output << "\n";
    rs.close();
    return 0;
}
