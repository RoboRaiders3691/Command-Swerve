#include "Mechanism Modules/SwerveModule.h"

SwerveModule::SwerveModule(
    const std::string name,
    const int driveMotorPort,
    const int turnMotorPort,
    const int absoluteEncoderPort,
    ctre::phoenix6::configs::TalonFXConfiguration driveConfigs, 
    ctre::phoenix6::configs::TalonFXConfiguration rotationConfigs,
    units::angle::turn_t arbitraryOffset) : turnMotor{turnMotorPort}, driveMotor{driveMotorPort}, CANcoder{absoluteEncoderPort}, m_request{arbitraryOffset}{

    m_name = name;

    turnMotor.GetConfigurator().Apply(rotationConfigs);
    driveMotor.GetConfigurator().Apply(driveConfigs);
}

void SwerveModule::ZeroModuleRotation(){
    turnMotor.SetPosition(CANcoder.GetPosition().GetValue());
}

void SwerveModule::ResetDrive(){
    driveMotor.Set(0);
}

units::angle::turn_t SwerveModule::GetRotation(){
    return turnMotor.GetPosition().GetValue();
}

units::angle::turn_t SwerveModule::GetRotationAbs(){
    return CANcoder.GetPosition().GetValue();
}

units::angle::turn_t SwerveModule::GetDriveRotation(){
    return driveMotor.GetPosition().GetValue();
}

//units::angle::turn_t SwerveModule::GetDriveDistance(); Implement Later

void SwerveModule::SetTargetRotation(units::angle::turn_t targetRotation){
    turnMotor.SetControl(m_request.WithPosition(targetRotation));
}

//void SwerveModule::SetDriveDistance(units::length::foot_t targetDistnace); Implement Later

void SwerveModule::SetDriveOutput(double percentOutput){

    if(percentOutput > 1){
        percentOutput = 1;
    }
    else if(percentOutput < -1){
        percentOutput = -1;
    }

    driveMotor.Set(percentOutput);

}
