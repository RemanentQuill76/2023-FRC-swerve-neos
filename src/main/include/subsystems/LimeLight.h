// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>




class LimeLight : public frc2::SubsystemBase {
 public:
  LimeLight();
  bool IsAutoLimeLightOn();
  double GetTX(); 
  double GetTY();
  double GetDistance();
  double GetTV();
  double GetVertical();
  double GetArea();
  void BallPipeline();
  void LeftPipeline();
  void RightPipeline();
  void MidPipeline();
  bool IsMidPipeline();
  void LightOn();
  void LightOff();
  void ToggleLight();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");//added nt:: in 2022
  bool tv;
  double tx;
  double ty;
  double tvert;
  double ta;
  double theight=66.;  //28.75
  double limelightheight=28.25;
  double limelightangle=0.;  // degrees L---->  =0 up=positive
  double distance;
  bool lighton;
};
