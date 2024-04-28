package frc.robot.subsystems.SwerveParts;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.SwerveModuleConstants;
//[TEMPLATE DRIVE IMPORTS]
//[TEMPLATE TURN IMPORTS]
//[TEMPLATE ENCODER IMPORTS]


public class SwerveModule {
    public int moduleNumber;
    //[TEMPLATE DRIVE PRIVATE VARIABLES]

    //[TEMPLATE TURN PRIVATE VARIABLES]

    //[TEMPLATE ENCODER PRIVATE VARIABLES]

    public SwerveModule(int moduleIndex, SwerveModuleConstants moduleConstant){
        this.moduleNumber = moduleIndex;

        //[TEMPLATE DRIVE INITIALIZATION]

        //[TEMPLATE TURN INITIALIZATION]

        //[TEMPLATE ENCODER INITIALIZATION]
    }

    //[TEMPLATE SETDESIREDSTATE]
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){}

    //[TEMPLATE RESETTOABSOLUTE]
    public void resetToAbsolute(){}

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            getVelocityMPS(), 
            getAbsoluteAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePositionMeters(),
            getAbsoluteAngle()
        );
    }

    //[TEMPLATE GETVELOCITY]
    public double getVelocityMPS() {return 0;}

    //[TEMPLATE GETABSOLUTEANGLE]
    public Rotation2d getAbsoluteAngle() {return Rotation2d.fromDegrees(0);}

    //[TEMPLATE GETPOSITION]
    public double getDrivePositionMeters() {return 0;}
}

