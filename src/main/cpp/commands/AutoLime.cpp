// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoLime.h"
#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>

AutoLime::AutoLime(LimeLight* limelight, DriveSubsystem* swerve, DistanceSensor* distancesense): 
    m_limelight(limelight), m_swerve(swerve), m_distanceSense(distancesense) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({limelight});
  AddRequirements({swerve});
  AddRequirements({distancesense});


}
// Called when the command is initially scheduled.
void AutoLime::Initialize() {
  m_limelight->ToggleLight();
  AutoLimeOn=m_limelight->IsAutoLimeLightOn();
  
  

}

// Called repeatedly when this Command is scheduled to run
void AutoLime::Execute() {
    LimeX=m_limelight->GetTX();
    distancesensor=m_distanceSense->GetDistance();

counter=counter + 1;

if (counter>=10){

  if (LimeX>0.0){rotate = fmin(pow(LimeX/20,1.0),0.5);}
  else if (LimeX<0.0){rotate = -fmin(pow(-LimeX/20,1.0),0.5);}
  else {rotate = 0.0;}

  if (distancesensor>18 && distancesensor<144) {drive = -fmin(distancesensor/90, 0.5);}
  else {drive = 0.0;}

  if (prevdrive==0.0) {prevdrive=drive;}
  if (prevrotate==0.0) {prevrotate=rotate;}
  drive = (drive+prevdrive)/2;
  rotate = (rotate+prevrotate)/2;

  prevdrive=drive;
  prevrotate=rotate;

  if (counter>=11) {counter = 0.0;}
}
/*
if (fabs(rotate)<0.05){
    if(LimeArea>0.5 && LimeArea<5){drive = -0.4;}
    else {drive = 0.0;}
}*/
  frc::SmartDashboard::PutNumber("LimX", LimeX);
  frc::SmartDashboard::PutNumber("autodrivespeed", drive);
  m_swerve->Drive(drive,0.0,rotate,false);//add drive if it works.

}

// Called once the command ends or is interrupted.
void AutoLime::End(bool interrupted) {

  m_limelight->ToggleLight();
  AutoLimeOn=m_limelight->IsAutoLimeLightOn();
}

// Returns true when the command should end.
bool AutoLime::IsFinished() {
  return false;
}
