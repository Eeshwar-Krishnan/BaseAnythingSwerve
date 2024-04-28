package frc.robot.subsystems.SwerveParts.Drive;

//[BEGIN DRIVE IMPORTS]
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
//[END DRIVE IMPORTS]


public class CTRETalonFXDriveMotor {
    //[BEGIN DRIVE PRIVATE VARIABLES]
    private final TalonFX mDriveMotor;
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    //[END DRIVE PRIVATE VARIABLES]
    public CTRETalonFXDriveMotor(SwerveModuleConstants moduleConstant) {
        //[BEGIN DRIVE INITIALIZATION]
        //Sets up a TalonFX on the "rio" bus.
        //TODO: Change to a CANivore ID if needed.
        mDriveMotor = new TalonFX(moduleConstant.driveMotorID, "rio");

        TalonFXConfiguration mDriveConfiguration = new TalonFXConfiguration();

        //Enable current limits
        mDriveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        //Set the supply current limit. This is the absolute limit before the motor hard limits current.
        mDriveConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        //Sets the supply current threshold. The motor will allow brief spikes over this threshold.
        mDriveConfiguration.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
        //Sets how long the motor will allow current over the threshold before ramping down the motor.
        mDriveConfiguration.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;

        //Sets the gear ratio between the wheel and the motor
        mDriveConfiguration.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        //Sets a slight ramp rate for duty cycle and voltage. This prevents the motor from ramping too fast, improving control.
        mDriveConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        mDriveConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        //PID gain setup
        mDriveConfiguration.Slot0.kP = Constants.Swerve.driveKP;
        mDriveConfiguration.Slot0.kI = Constants.Swerve.driveKI;
        mDriveConfiguration.Slot0.kD = Constants.Swerve.driveKD;

        //Sets the motor inversion and neutral mode. The drive motor is on BRAKE by default.
        mDriveConfiguration.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        mDriveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //Sets ramp rates for closed loop control. This improves control stability by preventing the motor from ramping too fast.
        mDriveConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        mDriveConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;

        mDriveMotor.getConfigurator().apply(mDriveConfiguration);
        //[END DRIVE INITIALIZATION]
    }

    //[BEGIN SETSPEED]
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop){
            double speed = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(speed);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }
    //[END SETSPEED]

    //[BEGIN GETVELOCITY]
    private double getVelocityMPS(){
        return Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference);
    }
    //[END GETVELOCITY]
    
    //[BEGIN GETPOSITION]
    private double getDrivePositionMeters(){
        return Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference);
    }
    //[END GETPOSITION]
}
