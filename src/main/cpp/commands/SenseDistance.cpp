// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SenseDistance.h"

SenseDistance::SenseDistance(DistanceSensor* distancesubsystem, DriveSubsystem* drivesubsystem):m_distanceSense(distancesubsystem), m_swerve(drivesubsystem) {
 AddRequirements({distancesubsystem});
 AddRequirements({drivesubsystem});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SenseDistance::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SenseDistance::Execute() {
  currentdistance = m_distanceSense->GetDistance();
  if (currentdistance>40){i++;}
  if (currentdistance<40){i--;}
  if (i>5){
    i=5;
    m_swerve->Drive(-.5,0.0,0.0,false);
  }
  else if (i<0){
    i=0;
    m_swerve->Drive(0.0,0.0,0.0,false);
  }
  
}

// Called once the command ends or is interrupted.
void SenseDistance::End(bool interrupted) {
    m_swerve->Drive(0.0,0.0,0.0,false);
}

// Returns true when the command should end.
bool SenseDistance::IsFinished() {
  return false;
}
