package frc.robot.subsystems.SwerveParts.Turn;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

//[BEGIN TURN IMPORTS]
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
//[END TURN IMPORTS]

import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class TurnMotorNeo {
    //[BEGIN TURN PRIVATE VARIABLES]
    private final CANSparkMax mTurnMotor;
    private final RelativeEncoder mTurnMotorEncoder;
    private final SparkPIDController mTurnController;
    //[END TURN PRIVATE VARIABLES]
    public TurnMotorNeo(SwerveModuleConstants moduleConstant) {
        //[BEGIN TURN INITIALIZATION]
        mTurnMotor = new CANSparkMax(moduleConstant.angleMotorID, MotorType.kBrushless);

        mTurnMotorEncoder = mTurnMotor.getEncoder();
        
        mTurnMotorEncoder.setPositionConversionFactor((2 * Math.PI) / Constants.Swerve.angleGearRatio);
        mTurnMotorEncoder.setVelocityConversionFactor(((2 * Math.PI) / Constants.Swerve.angleGearRatio) / 60.0);

        mTurnController = mTurnMotor.getPIDController();

        mTurnController.setP(Constants.Swerve.angleKP);
        mTurnController.setI(Constants.Swerve.angleKI);
        mTurnController.setD(Constants.Swerve.angleKD);
        mTurnController.setOutputRange(-1, 1);

        mTurnMotor.setIdleMode(IdleMode.kCoast);
        mTurnMotor.setSmartCurrentLimit(Constants.Swerve.angleCurrentThreshold);
        mTurnMotor.setInverted(Constants.Swerve.angleMotorInvert);

        mTurnMotor.burnFlash();
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
        mTurnMotorEncoder.setPosition(absolutePosition);
    }
    //[END RESETTOABSOLUTE]
    
    //[BEGIN SETDESIREDSTATE]
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mTurnController.setReference(desiredState.angle.getRotations(), ControlType.kPosition);
        setSpeed(desiredState, isOpenLoop);
    }
    //[END SETDESIREDSTATE]
}
