// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DistanceSensor.h"
#include <frc/smartdashboard/SmartDashboard.h>

DistanceSensor::DistanceSensor() = default;

double DistanceSensor::GetDistance(){
    double rawValue = ultrasonic.GetValue();
    double currentDistance = rawValue * 0.125;//in inches
    frc::SmartDashboard::PutNumber("ultrasonic", currentDistance);
    return currentDistance;

}
// This method will be called once per scheduler run
void DistanceSensor::Periodic() {}
