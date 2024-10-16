 #include "subsystems/DriveSubsystem.h"
 #include "Constants.h"

 using namespace SwerveConstants;

 //Determine how to implement the constants for the configs into Constants.h 
 DriveSubsystem::DriveSubsystem() : DriveMotorConfigs{}, RotationMotorConfigs{}, driveGyro{kDriveGyroPort} {

    // set slot 0 (Drive) gains
    auto& slot0ConfigsDrive = DriveMotorConfigs.Slot0;
    slot0ConfigsDrive.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsDrive.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsDrive.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsDrive.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0ConfigsDrive.kI = 0; // no output for integrated error
    slot0ConfigsDrive.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    auto& motionMagicConfigsDrive = DriveMotorConfigs.MotionMagic;
    motionMagicConfigsDrive.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigsDrive.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsDrive.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // set slot 0 (Rot) gains
    auto& slot0ConfigsRot = RotationMotorConfigs.Slot0;
    slot0ConfigsRot.kS = 0.15; // Add 0.25 V output to overcome static friction
    slot0ConfigsRot.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsRot.kA = 0.04; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsRot.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0ConfigsRot.kI = 0; // no output for integrated error
    slot0ConfigsRot.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output


    auto& motionMagicConfigsRot = RotationMotorConfigs.MotionMagic;
    motionMagicConfigsRot.MotionMagicCruiseVelocity = 90; // Target cruise velocity of 80 rps
    motionMagicConfigsRot.MotionMagicAcceleration = 110; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsRot.MotionMagicJerk = 1100; // Target jerk of 1600 rps/s/s (0.1 seconds)

    RotationMotorConfigs.Feedback.SensorToMechanismRatio = 13.3714;
    RotationMotorConfigs.ClosedLoopGeneral.ContinuousWrap = true;


    m_frontRightSwerveModule = std::make_unique<SwerveModule>(
        "Front Right",
        kFrontRightDriveMotorPort,
        kFrontRightTurnMotorPort,
        kFrontRightAbsoluteEncoderPort,
        DriveMotorConfigs,
        RotationMotorConfigs,
        kDriveArbitraryOffset
    );
    m_frontLeftSwerveModule = std::make_unique<SwerveModule>(
        "Front Left",
        kFrontLeftDriveMotorPort,
        kFrontLeftTurnMotorPort,
        kFrontLeftAbsoluteEncoderPort,
        DriveMotorConfigs,
        RotationMotorConfigs,
        kDriveArbitraryOffset
    );
    m_backLeftSwerveModule = std::make_unique<SwerveModule>(
        "Back Left",
        kBackLeftDriveMotorPort,
        kBackLeftTurnMotorPort,
        kBackLeftAbsoluteEncoderPort,
        DriveMotorConfigs,
        RotationMotorConfigs,
        kDriveArbitraryOffset
    );
    m_backRightSwerveModule = std::make_unique<SwerveModule>(
        "Back Right",
        kBackRightDriveMotorPort,
        kBackRightTurnMotorPort,
        kBackRightAbsoluteEncoderPort,
        DriveMotorConfigs,
        RotationMotorConfigs,
        kDriveArbitraryOffset
    ); 
 }

 void DriveSubsystem::Periodic(){

 //For right now do nothing

 }

 //Drive should later be rewriten to support field relative control as well as inputs in coordinates to move to in a straight line
 //Currently only supports basic drive for controller use
 //Maybe rework the calculations to use exlusively units library rather than converting to doubles then back to units
 void DriveSubsystem::Drive(
            units::velocity::feet_per_second_t xSpeed,
            units::velocity::feet_per_second_t ySpeed, 
            units::angular_velocity::turns_per_second_t rot,
            bool fieldRelative){

        if(hypot(xSpeed.value(), ySpeed.value()) <= .05){
            xSpeed == 0_fps;
            ySpeed == 0_fps;
        }
        if(abs(rot.value()) <= .05){
            rot = 0_rpm;
        }       


        radiusToWheels = hypot(kWheelBase.value(), kTrackWidth.value());

        //Assign the transition variable A, B, C, and D their values
        A = xSpeed.value() - (rot.value() * kWheelBase.value() / radiusToWheels);
        B = xSpeed.value() + (rot.value() * kWheelBase.value() / radiusToWheels);
        C = ySpeed.value() - (rot.value() * kTrackWidth.value() / radiusToWheels);
        D = ySpeed.value() + (rot.value() * kTrackWidth.value() / radiusToWheels);

        FRMovement.wheelSpeed = hypot(B, C);
        FRMovement.wheelAngle = fmod(((((atan2(C, B) - (.5 * pi)) / -pi) * 180) + 360), 360);

        FLMovement.wheelSpeed = hypot(B, D);
        FLMovement.wheelAngle = fmod(((((atan2(D, B) - (.5 * pi)) / -pi) * 180) + 360), 360);

        BLMovement.wheelSpeed = hypot(A, D);
        BLMovement.wheelAngle = fmod(((((atan2(D, A) - (.5 * pi)) / -pi) * 180) + 360), 360);

        BRMovement.wheelSpeed = hypot(A, C);
        BRMovement.wheelAngle = fmod(((((atan2(C, A) - (.5 * pi)) / -pi) * 180) + 360), 360);

        maxSpeed = std::max({ FRMovement.wheelSpeed, FLMovement.wheelSpeed, BLMovement.wheelSpeed, BRMovement.wheelSpeed });

        if (maxSpeed > 1) {
            FRMovement.wheelSpeed /= maxSpeed;
            FLMovement.wheelSpeed /= maxSpeed;
            BLMovement.wheelSpeed /= maxSpeed;
            BRMovement.wheelSpeed /= maxSpeed;
        }


        m_frontRightSwerveModule->SetTargetRotation(units::angle::turn_t(FRMovement.wheelAngle / 360));
        m_frontRightSwerveModule->SetDriveOutput(FRMovement.wheelSpeed);

        m_frontLeftSwerveModule->SetTargetRotation(units::angle::turn_t(FLMovement.wheelAngle / 360));
        m_frontLeftSwerveModule->SetDriveOutput(-FLMovement.wheelSpeed);

        m_backLeftSwerveModule->SetTargetRotation(units::angle::turn_t(BLMovement.wheelAngle / 360));
        m_backLeftSwerveModule->SetDriveOutput(-BLMovement.wheelSpeed);

        m_backRightSwerveModule->SetTargetRotation(units::angle::turn_t(BRMovement.wheelAngle / 360));
        m_backRightSwerveModule->SetDriveOutput(BRMovement.wheelSpeed);


        frc::SmartDashboard::PutNumber("FR rotationTarget", FRMovement.wheelAngle);
        frc::SmartDashboard::PutNumber("FL rotationTarget", FLMovement.wheelAngle);
        frc::SmartDashboard::PutNumber("BL rotationTarget", BLMovement.wheelAngle);
        frc::SmartDashboard::PutNumber("BR rotationTarget", BRMovement.wheelAngle);

 }

 void DriveSubsystem::ResetMotorEncoders(){
    m_frontRightSwerveModule->ZeroModuleRotation();
    m_frontLeftSwerveModule->ZeroModuleRotation();
    m_backLeftSwerveModule->ZeroModuleRotation();
    m_backRightSwerveModule->ZeroModuleRotation();
 }