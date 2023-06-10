/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveDistance.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "RobotContainer.h"


DriveDistance::DriveDistance(double AutoDistance, DriveSubsystem* subsystem): m_swerve(subsystem), m_AutoEntry(AutoDistance)
{
  AddRequirements({subsystem});

  
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void DriveDistance::Initialize() {
 // timer.Start();
 // timer.Stop();
 // timer.Reset();
  
  WheelPos1 = m_swerve->GetWheelPosition();
  Heading = m_swerve->GetAngleAsDouble();
  //AutoRunTime = 0.0;
  e_AutoDistance = frc::Shuffleboard::GetTab("Autonomous").Add("AutoDist",100.0).GetEntry();
  AutoDist = e_AutoDistance->GetDouble(100.0);
  if (AutoDist != m_AutoEntry) { m_AutoEntry = AutoDist; }
  //AutoDist=100.0;
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute() {
 
 
 error=Heading-(*m_swerve).GetAngleAsDouble();
 roterror=copysign(fabs(error)+0.10,error);
 WheelDistance = ((*m_swerve).GetWheelPosition() - WheelPos1)*2.0;//approximate distance in inches
 frc::SmartDashboard::PutNumber("initialhead",Heading);
 frc::SmartDashboard::PutNumber("currenthead",(*m_swerve).GetAngleAsDouble());
 frc::SmartDashboard::PutNumber("error",error);
 frc::SmartDashboard::PutNumber("Distance",WheelDistance);
 
 if (abs(WheelDistance)<m_AutoEntry){
 
    (*m_swerve).Drive(-0.30, 0.0, roterror/10.0, false);
     
     EndSimple = false;
    }
  else {EndSimple=true;
    m_swerve->Drive(0.0, 0.0, 0.0, false);
  //  timer.Stop();
  //  timer.Reset();
  }
 // RunAngle=m_swerve->GetAngleAsDouble();//test
 // frc::SmartDashboard::PutNumber("gyro",RunAngle);//test
}

// Called once the command ends or is interrupted.
void DriveDistance::End(bool interrupted) {m_swerve->Drive(0.0, 0.0, 0.0, false);}

// Returns true when the command should end.
bool DriveDistance::IsFinished() { 
  
  if(EndSimple){return true;}
  else{return false; }
}
