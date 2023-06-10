/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace SwerveConstants;

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel, double encoderoffset, bool invertturn, bool sensorset)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
    m_turningMotor(turningMotorChannel),m_driveEncoder(m_driveMotor.GetEncoder()) //changed in 2022
    //constructor is called the first time an obect of this class is created.
     {
    
    m_driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

   // m_driveEncoder = m_driveMotor.GetEncoder(rev::CANEncoder::EncoderType::kQuadrature, 4096);//added 2022
    m_driveEncoder.SetVelocityConversionFactor(kmperrev/60);

    m_turningMotor.ConfigFactoryDefault();
		

		/* choose the sensor and sensor direction */
		m_turningMotor.ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Absolute,0,0);//Getting all this from positionclosedloop (phoenix)
		m_turningMotor.SetSensorPhase(sensorset);
    
		/* set the peak and nominal outputs, 12V means full */
		m_turningMotor.ConfigNominalOutputForward(0, 0);
		m_turningMotor.ConfigNominalOutputReverse(0, 0);
		m_turningMotor.ConfigPeakOutputForward(1.0, 0);
		m_turningMotor.ConfigPeakOutputReverse(-1.0, 0);

		/* set closed loop gains in slot0 */
		m_turningMotor.Config_kF(0.0, 0.0, 0.0);//middle number is the FPID
		m_turningMotor.Config_kP(0.0, 5.0, 0.0);
		m_turningMotor.Config_kI(0.0, 0.0, 0.0);
		m_turningMotor.Config_kD(0.0, 20.0, 0.0);
    

    m_turningMotor.SetInverted(invertturn);

    adjust = encoderoffset;
    
			
}

double SwerveModule::npitopi(double adjustangle){
      adjustangle=fmod(adjustangle,4096);//can get rid of????
      if (adjustangle<=-2048){adjustangle+=4096;};
      if (adjustangle>2048){adjustangle-=4096;};
      return adjustangle;
  }
double SwerveModule::GetEncoderPos(){
  return m_driveEncoder.GetPosition();
  }


frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()}/*units/100ms.  need radius of wheel in meters*/
          ,frc::Rotation2d(units::radian_t((m_turningMotor.GetSelectedSensorPosition(0)-encoderoffset)/4096*2*M_PI))};//Will need to adjust this for zero point
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{m_driveEncoder .GetPosition()/4096*2*M_PI*.0508},
          units::radian_t{(m_turningMotor.GetSelectedSensorPosition(0)-encoderoffset)/4096*2*M_PI}};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state, bool m_sbwrite, int wheel) 

{
  
  absolutePosition = fmod((m_turningMotor.GetSelectedSensorPosition(0)-adjust),4096);
  
  double setangle=state.angle.Radians().to<double>()/2/M_PI*4096;
  mult=1.;
 

  if (fabs(npitopi(setangle-absolutePosition))>fabs(npitopi(setangle-absolutePosition +2048))){
    mult=-1.;
    setangle+=2048;
  };//checks to see if it needs to turn less than pi/2 or switch motor direction

  double error=npitopi(setangle-absolutePosition);
 
  if (m_sbwrite) {
 
    frc::SmartDashboard::PutNumber("turnerror", error);
    frc::SmartDashboard::PutNumber("setangle", setangle);
    frc::SmartDashboard::PutNumber("WheelAngle",m_turningMotor.GetSelectedSensorPosition(0)-adjust);
    frc::SmartDashboard::PutNumber("Target",m_turningMotor.GetSelectedSensorPosition(0)-adjust+error);
    frc::SmartDashboard::PutNumber("absolutePos",absolutePosition);
    frc::SmartDashboard::PutNumber("DriveinputSpeed",copysign(fmin(fabs(state.speed.to<double>()),1.0),state.speed.to<double>()*mult));
    
  }

  if (wheel==1){frc::SmartDashboard::PutNumber("FL",m_turningMotor.GetSelectedSensorPosition(0));}
  if (wheel==2){frc::SmartDashboard::PutNumber("FR",m_turningMotor.GetSelectedSensorPosition(0));}
  if (wheel==3){frc::SmartDashboard::PutNumber("BL",m_turningMotor.GetSelectedSensorPosition(0));}
  if (wheel==4){frc::SmartDashboard::PutNumber("BR",m_turningMotor.GetSelectedSensorPosition(0));}
  
  double drive_speed=copysign(fmin(fabs(state.speed.to<double>()),1.0),state.speed.to<double>()*mult);
  
  m_driveMotor.Set(drive_speed);
  
  if (fabs(drive_speed)>.01){
     m_turningMotor.Set(ControlMode::Position, m_turningMotor.GetSelectedSensorPosition(0)+error);//(multiple+crossover)*4096+setangle+adjust);
  }
  else {m_turningMotor.Set(0.0);}
}
// This method will be called once per scheduler run
void SwerveModule::Periodic() {}
