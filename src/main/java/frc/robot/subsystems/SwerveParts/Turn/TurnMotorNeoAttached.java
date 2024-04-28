package frc.robot.subsystems.SwerveParts.Turn;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

//[BEGIN TURN IMPORTS]
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
//[END TURN IMPORTS]

import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class TurnMotorNeoAttached {
    //[BEGIN TURN PRIVATE VARIABLES]
    private final CANSparkMax mTurnMotor;
    private final SparkAbsoluteEncoder mTurnMotorEncoder;
    private final SparkPIDController mTurnController;
    //[END TURN PRIVATE VARIABLES]
    public TurnMotorNeoAttached(SwerveModuleConstants moduleConstant) {
        //[BEGIN TURN INITIALIZATION]
        mTurnMotor = new CANSparkMax(moduleConstant.angleMotorID, MotorType.kBrushless);

        mTurnMotorEncoder = mTurnMotor.getAbsoluteEncoder(Type.kDutyCycle);

        mTurnMotorEncoder.setPositionConversionFactor((2 * Math.PI));
        mTurnMotorEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0);

        mTurnController = mTurnMotor.getPIDController();

        mTurnController.setFeedbackDevice(mTurnMotorEncoder);

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

    public void setSpeed(SwerveModuleState state, boolean isOpenLoop) {
        
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            0, 
            Rotation2d.fromDegrees(0)
        );
    }
    //[END HELPER FUNCTION IGNORE]
    //[BEGIN RESETTOABSOLUTE]
    public void resetToAbsolute() {
        //We don't need to do anything, since we are following the encoder directly
    }
    //[END RESETTOABSOLUTE]
    //[BEGIN SETDESIREDSTATE]
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mTurnController.setReference(desiredState.angle.getRotations(), ControlType.kPosition);
        setSpeed(desiredState, isOpenLoop);
    }
    //[END SETDESIREDSTATE]
    //[BEGIN GETABSOLUTEANGLE]
    public Rotation2d getAbsoluteAngle(){
        return Rotation2d.fromRotations(mTurnMotorEncoder.getPosition());
    }
    //[END GETABSOLUTEANGLE]
}
