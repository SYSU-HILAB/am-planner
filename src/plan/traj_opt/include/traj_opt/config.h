#ifndef CONFIG_H
#define CONFIG_H

#include <ros/package.h>
#include <ros/ros.h>
#include <se3gcopter/trajectory.h>

#include <Eigen/Core>  // Assuming Eigen library is used
#include <iostream>
#include <string>
#include <vector>

enum Mode
{
  MODE_GRASP,
  MODE_STRIKE,
  MODE_OTHER
};

struct Config
{
  std::string odomFrame;

  // Params
  double scale;
  double mapHeight;
  Eigen::Vector3d polyhedronBox;
  double rho;
  double totalT;
  int qdIntervals;
  double horizHalfLen;
  double vertHalfLen;
  double safeMargin;
  double velMax;
  double thrustAccMin;
  double thrustAccMax;
  double bodyRateMax;
  double gravAcc;
  double armVelMax;
  Eigen::Vector4d penaltyPVTB;
  Eigen::Vector3d penaltyArm;
  bool useC2Diffeo;
  double optRelTol1;
  double optRelTol2;
  double trajVizWidth;
  Eigen::Vector3d trajVizRGB;
  std::string ellipsoidPath;
  Eigen::Vector4d ellipsoidVizRGBA;
  Eigen::Vector4d quadEllipsoidVizRGBA;
  Eigen::Vector4d eeEllipsoidVizRGBA;
  std::string quadrotorPath;
  Eigen::Vector4d PenaltyGraspWeight;
  double taskArmZ;
  double orientationR;
  double approachR;
  double platformR;
  double smoothR;
  double boundArmZ;
  double maxLengthLine;
  double initVel;
  double biasArmToBody;
  double upperArm;
  double lowerArm;
  double staticRadius;
  double endEffectorRadius;
  double objectHeight;
  double graspR;
  bool multiLayerOpt;
  bool printCost;
  double EEDownVel;
  bool collectData;
  double sCurvCoeff;
  bool restart;
  int coordinateNum;
  Eigen::Vector3d Fext;
  void loadParameters(const ros::NodeHandle &nh_priv)
  {
    std::vector<double> vecPolyhedronBox, vecPenaltyPVTB, vecTrajVizRGB,
        vecEllipsoidVizRGBA, vecPenaltyGraspWeight, vecPenaltyArm,
        vecQuadEllipsoidVizRGBA, vecEEEllipsoidVizRGBA;
    std::string packagePath = ros::package::getPath("plan_manage");
    packagePath += packagePath.back() == '/' ? "" : "/";
    std::string packageUrl = "package://plan_manage/";
    nh_priv.getParam("OdomFrame", odomFrame);
    nh_priv.getParam("/scale", scale);
    nh_priv.getParam("MapHeight", mapHeight);
    nh_priv.getParam("PolyhedronBox", vecPolyhedronBox);
    polyhedronBox << vecPolyhedronBox[0], vecPolyhedronBox[1], vecPolyhedronBox[2];
    nh_priv.getParam("Rho", rho);
    nh_priv.getParam("TotalT", totalT);
    nh_priv.getParam("QdIntervals", qdIntervals);
    nh_priv.getParam("HorizHalfLen", horizHalfLen);
    nh_priv.getParam("VertHalfLen", vertHalfLen);
    nh_priv.getParam("SafeMargin", safeMargin);
    nh_priv.getParam("VelMax", velMax);
    nh_priv.getParam("ThrustAccMin", thrustAccMin);
    nh_priv.getParam("ThrustAccMax", thrustAccMax);
    nh_priv.getParam("BodyRateMax", bodyRateMax);
    nh_priv.getParam("GravAcc", gravAcc);
    nh_priv.getParam("PenaltyPVTB", vecPenaltyPVTB);
    penaltyPVTB << vecPenaltyPVTB[0], vecPenaltyPVTB[1], vecPenaltyPVTB[2],
        vecPenaltyPVTB[3];
    nh_priv.getParam("PenaltyArm", vecPenaltyArm);
    penaltyArm << vecPenaltyArm[0], vecPenaltyArm[1], vecPenaltyArm[2];
    nh_priv.getParam("UseC2Diffeo", useC2Diffeo);
    nh_priv.getParam("OptRelTol1", optRelTol1);
    nh_priv.getParam("OptRelTol2", optRelTol2);
    nh_priv.getParam("TrajVizWidth", trajVizWidth);
    nh_priv.getParam("TrajVizRGB", vecTrajVizRGB);
    trajVizRGB << vecTrajVizRGB[0], vecTrajVizRGB[1], vecTrajVizRGB[2];
    nh_priv.getParam("EllipsoidPath", ellipsoidPath);
    ellipsoidPath = packageUrl + ellipsoidPath;
    nh_priv.getParam("QuadEllipsoidVizRGBA", vecQuadEllipsoidVizRGBA);
    quadEllipsoidVizRGBA << vecQuadEllipsoidVizRGBA[0], vecQuadEllipsoidVizRGBA[1],
        vecQuadEllipsoidVizRGBA[2], vecQuadEllipsoidVizRGBA[3];
    nh_priv.getParam("EEEllipsoidVizRGBA", vecEEEllipsoidVizRGBA);
    eeEllipsoidVizRGBA << vecEEEllipsoidVizRGBA[0], vecEEEllipsoidVizRGBA[1],
        vecEEEllipsoidVizRGBA[2], vecEEEllipsoidVizRGBA[3];
    nh_priv.getParam("QuadrotorPath", quadrotorPath);
    quadrotorPath = packageUrl + quadrotorPath;
    nh_priv.getParam("PenaltyGraspWeight", vecPenaltyGraspWeight);
    PenaltyGraspWeight << vecPenaltyGraspWeight[0], vecPenaltyGraspWeight[1],
        vecPenaltyGraspWeight[2], vecPenaltyGraspWeight[3];
    nh_priv.getParam("TaskArmZ", taskArmZ);
    nh_priv.getParam("OrientationR", orientationR);
    nh_priv.getParam("ApproachR", approachR);
    nh_priv.getParam("PlatformR", platformR);
    nh_priv.getParam("ArmVelMax", armVelMax);
    nh_priv.getParam("SmoothR", smoothR);
    nh_priv.getParam("BoundArmZ", boundArmZ);
    nh_priv.getParam("MaxLengthLine", maxLengthLine);
    nh_priv.getParam("InitVel", initVel);
    nh_priv.getParam("BiasArmToBody", biasArmToBody);
    nh_priv.getParam("UpperArm", upperArm);
    nh_priv.getParam("LowerArm", lowerArm);
    nh_priv.getParam("StaticRadius", staticRadius);
    nh_priv.getParam("EndEffectorRadius", endEffectorRadius);
    nh_priv.getParam("ObjectHeight", objectHeight);
    nh_priv.getParam("GraspR", graspR);
    nh_priv.getParam("MultiLayerOpt", multiLayerOpt);
    nh_priv.getParam("PrintCost", printCost);
    nh_priv.getParam("EEDownVel", EEDownVel);
    nh_priv.getParam("CollectData", collectData);
    nh_priv.getParam("S_CurvCoeff", sCurvCoeff);
    nh_priv.getParam("Restart", restart);
    nh_priv.getParam("CoordinateNum", coordinateNum);
    std::vector<double> vecFext;
    if (!nh_priv.getParam("Fext", vecFext))
    {
      vecFext = {0.0, 0.0, 0.0};
    }
    Fext << vecFext[0], vecFext[1], vecFext[2];
  };
};

#endif  // PLAN_MANAGE_CONFIG_H_
