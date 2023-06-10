/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once


#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandHelper.h>

#include "commands/DriveDistance.h"
#include "commands/SetHeading.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ComplexAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, ComplexAuto> {
 public:
  ComplexAuto(double drive, double turn, DriveSubsystem* Drive);

 private:
  double m_drive;
  double m_turn;
  DriveSubsystem* m_swerve;
  /*void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;*/
};
