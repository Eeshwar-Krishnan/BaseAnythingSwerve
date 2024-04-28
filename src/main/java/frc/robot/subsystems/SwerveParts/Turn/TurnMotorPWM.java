package frc.robot.subsystems.SwerveParts.Turn;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

//[BEGIN TURN IMPORTS]
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
//[END TURN IMPORTS]

import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class TurnMotorPWM {
    //[BEGIN TURN PRIVATE VARIABLES]
    private final PWMTalonSRX mTurnMotor;
    private final PIDController turnController;
    //[END TURN PRIVATE VARIABLES]
    public TurnMotorPWM(SwerveModuleConstants moduleConstant) {
        //[BEGIN TURN INITIALIZATION]
        mTurnMotor = new PWMTalonSRX(moduleConstant.angleMotorID);

        turnController = new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);

        mTurnMotor.setInverted(Constants.Swerve.angleMotorInvert);
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
        //We don't need to do anything, since the WPILIB PID is doing that for us
    }
    //[END RESETTOABSOLUTE]
    //[BEGIN SETDESIREDSTATE]
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mTurnMotor.set(turnController.calculate(getAbsoluteAngle().getRotations(), desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }
    //[END SETDESIREDSTATE]
}
