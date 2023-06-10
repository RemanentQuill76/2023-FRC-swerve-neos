// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimeLight.h"
#include <frc/smartdashboard/SmartDashboard.h>

LimeLight::LimeLight() {
    lighton=true;
    ToggleLight();
}

// This method will be called once per scheduler run
void LimeLight::Periodic() {}


bool LimeLight::IsAutoLimeLightOn(){
    
    frc::SmartDashboard::PutNumber("AutoLimeOn", lighton);
    return lighton;
}

double LimeLight::GetTX(){ 
    double tx = table->GetNumber("tx",0.0);
    
    return (tx);

}

double LimeLight::GetTY(){
    ty = table->GetNumber("ty",0.0);
    
    return (ty);
}

double LimeLight::GetDistance(){
    //inches::Have not adjusted from Anne's program
    return (.11437167718137*pow(GetVertical(),2.)+-8.4560021555243*GetVertical()+197.84917270283);
}

double LimeLight::GetTV(){
    tv = table->GetBoolean("tv",0);
    return (tv);
}

double LimeLight::GetVertical(){
    tvert = table->GetNumber("tvert",0.0);
    return (tvert);
}

double LimeLight::GetArea(){
    ta = table->GetNumber("ta", 0.0);
    return (ta);
    }
/*Anne's code
void LimeLight::BallPipeline(){
    table=inst.GetTable("limelight");
    table->GetEntry("pipeline").SetDouble(0);
}

void LimeLight::LeftPipeline(){
    table=inst.GetTable("limelight");
    table->GetEntry("pipeline").SetDouble(1);
}

void LimeLight::RightPipeline(){
    table=inst.GetTable("limelight");
    table->GetEntry("pipeline").SetDouble(1);
}

void LimeLight::MidPipeline(){
    table=inst.GetTable("limelight");
    table->GetEntry("pipeline").SetDouble(1);  
}

bool LimeLight::IsMidPipeline(){
    table=inst.GetTable("limelight");
    return (table->GetEntry("getpipe").GetDouble(10)==0);
}
*/
void LimeLight::LightOff(){
    if (lighton){
   //     table=inst.GetTable("limelight");
        table->GetEntry("ledMode").SetDouble(1);
        lighton=false;
    }  
}  //check code on limelight's website if this doesn't work.

void LimeLight::LightOn(){
    if (!lighton){
 //       table=inst.GetTable("limelight");
        table->GetEntry("ledMode").SetDouble(3);
        lighton=true;
    }
}  

void LimeLight::ToggleLight(){
    if (lighton){
        LightOff();
    }
    else{
        LightOn();
    };
}
