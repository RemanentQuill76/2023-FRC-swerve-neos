/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"
#include <networktables/GenericEntry.h>
#include <networktables/NetworkTableEntry.h>

#include <frc/shuffleboard/Shuffleboard.h>//delete later if not used
#include <frc/shuffleboard/ShuffleboardTab.h>//delete later if not used

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetHeading
    : public frc2::CommandHelper<frc2::CommandBase, SetHeading> {
 public:
  SetHeading(double setheading, DriveSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  double m_setheading;
  double initheading;
  double currentheading;
  double error;
  double SetHead;
 
  //nt::NetworkTableEntry e_setheading;
  
  nt::GenericEntry* e_setheading;
  
  //nt::NetworkTableEntry myBoolean = frc::Shuffleboard.GetTab("Autonomous").Add("SetHeading", 180).GetEntry();
  
  DriveSubsystem* m_swerve;

};
