/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetHeading.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

SetHeading::SetHeading(double setheading, DriveSubsystem* subsystem): m_setheading(setheading), m_swerve(subsystem) {
  AddRequirements({subsystem});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SetHeading::Initialize() {
 e_setheading = frc::Shuffleboard::GetTab("Autonomous").Add("setheading",180.0).GetEntry();
 
 
 
 SetHead = e_setheading->GetDouble(180.0);

 if (SetHead != m_setheading) { m_setheading = SetHead; }
//  SetHead=180.0;
}

// Called repeatedly when this Command is scheduled to run
void SetHeading::Execute() {
  currentheading=fmod(m_swerve->GetAngleAsDouble(),360);
  if (currentheading<0.0) {currentheading=currentheading+360.0;}
  frc::SmartDashboard::PutNumber("SetHeading",m_setheading);
  frc::SmartDashboard::PutNumber("headmod",fmod(currentheading,360));
  if ((currentheading<m_setheading && (m_setheading-currentheading)<=180) || (currentheading>m_setheading && currentheading-m_setheading>180)){
    m_swerve->Drive(0.0,0.0,0.4,false);}
  else {m_swerve->Drive(0.0,0.0,-0.4,false);}
}

// Called once the command ends or is interrupted.
void SetHeading::End(bool interrupted) {m_swerve->Drive(0.0,0.0,0.0,false);}

// Returns true when the command should end.
bool SetHeading::IsFinished() { 
  if (fabs(fmod(currentheading,360)-m_setheading)<0.5) {return true;}
  else {return false; }
  }
