/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <numbers>
#include "subsystems/DriveSubsystem.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/GenericEntry.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveDistance
    : public frc2::CommandHelper<frc2::CommandBase, DriveDistance> {
 public:
  DriveDistance(double AutoDistance, DriveSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  
  private:
  DriveSubsystem* m_swerve;
 // double m_distance;
 // double m_speed;
 // frc::Timer timer;
  bool EndSimple;
  
  double m_AutoEntry;
  double WheelPos1;
  //double WheelPos2;
  double WheelDistance;
 // double InitialAngle;
  double Heading;
  double error;
  double roterror;
  //double RunAngle;
  double AutoDist;
  
  nt::GenericEntry* e_AutoDistance;
  
};
