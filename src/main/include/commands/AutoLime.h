// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/LimeLight.h"
#include "subsystems/DistanceSensor.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoLime
    : public frc2::CommandHelper<frc2::CommandBase, AutoLime> {
 public:
  
  AutoLime(LimeLight* limelight, DriveSubsystem* swerve, DistanceSensor* distancesense);


  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  LimeLight* m_limelight;
  DriveSubsystem* m_swerve;
  DistanceSensor* m_distanceSense;

  bool AutoLimeOn;
  double LimeX;
  double LimeArea;
  double LimeAreaPrev;
  double rotate;
  double drive;
  double distancesensor;
  double counter;
  double prevdrive;
  double prevrotate;
  
};
