package frc.robot.subsystems.SwerveParts.Drive;

//[BEGIN DRIVE IMPORTS]
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import com.revrobotics.CANSparkLowLevel.MotorType;
//[END DRIVE IMPORTS]


public class RevNeoDriveMotor {
    //[BEGIN DRIVE PRIVATE VARIABLES]
    private final CANSparkMax mDriveMotor;
    private final RelativeEncoder mDriveEncoder;
    private final SparkPIDController mDriveController;
    //[END DRIVE PRIVATE VARIABLES]
    public RevNeoDriveMotor(SwerveModuleConstants moduleConstant) {
        //[BEGIN DRIVE INITIALIZATION]
        mDriveMotor = new CANSparkMax(moduleConstant.driveMotorID, MotorType.kBrushless);

        mDriveEncoder = mDriveMotor.getEncoder();
        
        mDriveEncoder.setPositionConversionFactor((Constants.Swerve.wheelCircumference) / Constants.Swerve.driveGearRatio);
        mDriveEncoder.setVelocityConversionFactor(((Constants.Swerve.wheelCircumference) / Constants.Swerve.driveGearRatio) / 60.0);

        mDriveController = mDriveMotor.getPIDController();

        mDriveController.setP(Constants.Swerve.driveKP);
        mDriveController.setI(Constants.Swerve.driveKI);
        mDriveController.setD(Constants.Swerve.driveKD);
        mDriveController.setFF(Constants.Swerve.driveKF);
        mDriveController.setOutputRange(-1, 1);

        mDriveMotor.setIdleMode(IdleMode.kBrake);
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveCurrentThreshold);

        mDriveMotor.setOpenLoopRampRate(1.0 / Constants.Swerve.openLoopRamp);
        mDriveMotor.setClosedLoopRampRate(1.0 / Constants.Swerve.closedLoopRamp);

        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);

        mDriveMotor.burnFlash();
        //[END DRIVE INITIALIZATION]
    }

    //[BEGIN SETSPEED]
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop){
            double speed = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(speed);
        }
        else {
            mDriveController.setReference(desiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        }
    }
    //[END SETSPEED]

    //[BEGIN GETVELOCITY]
    public double getVelocityMPS(){
        return Conversions.RPSToMPS(mDriveEncoder.getVelocity(), Constants.Swerve.wheelCircumference);
    }
    //[END GETVELOCITY]
    
    //[BEGIN GETPOSITION]
    public double getDrivePositionMeters(){
        return Conversions.rotationsToMeters(mDriveEncoder.getPosition(), Constants.Swerve.wheelCircumference);
    }
    //[END GETPOSITION]
}
