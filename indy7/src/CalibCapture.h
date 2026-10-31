/*****************************************************************************
*	Name: CalibCapture.h
*	Description: The poses of the tool for the hand-eye calibration (paper,
*	Sec. V-C, eq. (10)): 'w' arms the capture, then every 's' writes
*	pose_rPe_<N>.yaml into CALIB_DIR. Rotation matrix -> theta-u (axis-angle)
*	with Eigen::AngleAxisd; the output has the layout of ViSP
*	vpPoseVector::saveYAML, which calib/eye_to_hand_calib.py reads.
*****************************************************************************/
#ifndef G2T_CALIB_CAPTURE_H
#define G2T_CALIB_CAPTURE_H

#include <string>
#include "FullDynControllerRT.h"

class CalibCapture
{
public:
    explicit CalibCapture(const std::string& outDir = "calib_data");

    // directory of the dataset (created at the first capture)
    void               SetDir(const std::string& outDir) { m_outDir = outDir; }
    const std::string& GetDir() const { return m_outDir; }

    // Save current pose as <outDir>/pose_rPe_<N>.yaml
    // (6x1 pose vector: [tx ty tz, thetau_x thetau_y thetau_z])
    void Capture(const CControllerFullDynamicsRT::Pose& pose);

    void Reset();
    int  GetCount() const { return m_count; }

private:
    std::string m_outDir;
    int         m_count;
};

#endif // G2T_CALIB_CAPTURE_H
