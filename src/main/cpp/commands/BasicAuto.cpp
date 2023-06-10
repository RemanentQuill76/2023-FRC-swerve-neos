/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/BasicAuto.h"
#include <frc/smartdashboard/SmartDashboard.h>

BasicAuto::BasicAuto() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void BasicAuto::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void BasicAuto::Execute() {frc::SmartDashboard::PutBoolean("basicauto",true);}

// Called once the command ends or is interrupted.
void BasicAuto::End(bool interrupted) {}

// Returns true when the command should end.
bool BasicAuto::IsFinished() { return false; }
