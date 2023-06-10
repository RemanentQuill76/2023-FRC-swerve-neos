// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeUpDown.h"

IntakeUpDown::IntakeUpDown(IntakeSubsystem* subsystem) 
: m_intakeUD(subsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void IntakeUpDown::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeUpDown::Execute() {
  m_intakeUD->togglecylinder();
}

// Called once the command ends or is interrupted.
void IntakeUpDown::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeUpDown::IsFinished() {
  return true;
}
