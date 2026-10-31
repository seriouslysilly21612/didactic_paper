/*****************************************************************************
*	Name: CalibCapture.cpp
*	Description: The poses of the tool for the hand-eye calibration. See
*	CalibCapture.h.
*****************************************************************************/
#include "CalibCapture.h"
#include <sstream>
#include <cstdio>
#include <sys/stat.h>
#include <Eigen/Geometry>

CalibCapture::CalibCapture(const std::string& outDir)
    : m_outDir(outDir), m_count(1)
{
}

void CalibCapture::Capture(const CControllerFullDynamicsRT::Pose& pose)
{
    // rotation matrix -> theta-u (axis-angle) vector, ViSP/vpPoseVector convention
    Eigen::AngleAxisd aa(pose.m_rotation);
    const Eigen::Vector3d tu = aa.axis() * aa.angle();

    mkdir(m_outDir.c_str(), 0777);
    std::stringstream ss;
    ss << m_outDir << "/pose_rPe_" << m_count << ".yaml";

    FILE* fp = fopen(ss.str().c_str(), "w");
    if (!fp) {
        printf("[CalibCapture] ERROR: cannot open %s\n", ss.str().c_str());
        return;
    }
    // Same layout ViSP vpPoseVector::saveYAML writes (rows/cols/data)
    fprintf(fp, "rows: 6\n");
    fprintf(fp, "cols: 1\n");
    fprintf(fp, "data: \n");
    fprintf(fp, "  - [%.9f]\n", pose.m_position[0]);
    fprintf(fp, "  - [%.9f]\n", pose.m_position[1]);
    fprintf(fp, "  - [%.9f]\n", pose.m_position[2]);
    fprintf(fp, "  - [%.9f]\n", tu[0]);
    fprintf(fp, "  - [%.9f]\n", tu[1]);
    fprintf(fp, "  - [%.9f]\n", tu[2]);
    fclose(fp);

    printf("[CalibCapture] #%d saved: t=[%.4f, %.4f, %.4f] -> %s\n",
           m_count,
           pose.m_position[0], pose.m_position[1], pose.m_position[2],
           ss.str().c_str());
    m_count++;
}

void CalibCapture::Reset()
{
    m_count = 1;    // the first pose is pose_rPe_1, like image0001
}
