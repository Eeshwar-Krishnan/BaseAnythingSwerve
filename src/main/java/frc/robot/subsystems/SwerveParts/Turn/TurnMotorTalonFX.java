package frc.robot.subsystems.SwerveParts.Turn;

//[BEGIN TURN IMPORTS]
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//[END TURN IMPORTS]

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class TurnMotorTalonFX {
    //[BEGIN TURN PRIVATE VARIABLES]
    private final TalonFX mTurnMotor;
    private final PositionVoltage anglePosition = new PositionVoltage(0);
    //[END TURN PRIVATE VARIABLES]
    public TurnMotorTalonFX(SwerveModuleConstants moduleConstant) {
        //[BEGIN TURN INITIALIZATION]
        mTurnMotor = new TalonFX(moduleConstant.angleMotorID, "rio");

        TalonFXConfiguration mTurnConfiguration = new TalonFXConfiguration();

        mTurnConfiguration.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        mTurnConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        mTurnConfiguration.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.angleCurrentThreshold;
        mTurnConfiguration.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;

        mTurnConfiguration.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;

        mTurnConfiguration.Slot0.kP = Constants.Swerve.angleKP;
        mTurnConfiguration.Slot0.kI = Constants.Swerve.angleKI;
        mTurnConfiguration.Slot0.kD = Constants.Swerve.angleKD;

        mTurnConfiguration.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        mTurnConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        mTurnMotor.getConfigurator().apply(mTurnConfiguration);
        //[END TURN INITIALIZATION]
    }

    //[BEGIN HELPER FUNCTION IGNORE]
    public Rotation2d getAbsoluteAngle(){
        return Rotation2d.fromDegrees(0);
    }

    public void setSpeed(SwerveModuleState state, boolean isOpenLoop) {
        
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            0, 
            getAbsoluteAngle()
        );
    }
    //[END HELPER FUNCTION IGNORE]
    //[BEGIN RESETTOABSOLUTE]
    public void resetToAbsolute() {
        double absolutePosition = getAbsoluteAngle().getRotations();
        mTurnMotor.setPosition(absolutePosition);
    }
    //[END RESETTOABSOLUTE]
    //[BEGIN SETDESIREDSTATE]
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mTurnMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }
    //[END SETDESIREDSTATE]
}
