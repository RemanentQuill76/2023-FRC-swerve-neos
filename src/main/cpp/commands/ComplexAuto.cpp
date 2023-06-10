/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ComplexAuto.h"

ComplexAuto::ComplexAuto(double drive, double turn, DriveSubsystem* Drive): /*builds without this*/m_drive(drive), m_turn(turn), m_swerve(Drive){
  AddCommands(
      // Drive forward the specified distance
      DriveDistance(m_drive, m_swerve),
      // Release the hatch
      SetHeading(m_turn, m_swerve));
  //I wonder if using m_drive makes that veriable public for all functions.  That is why it is not needed here.
}

  // Use addRequirements() here to declare subsystem dependencies.
/*

// Called when the command is initially scheduled.
void ComplexAuto::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ComplexAuto::Execute() {}

// Called once the command ends or is interrupted.
void ComplexAuto::End(bool interrupted) {}

// Returns true when the command should end.
bool ComplexAuto::IsFinished() { return true; }
*/