package frc.robot.subsystems.SwerveParts.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;
//[BEGIN ENCODER IMPORTS]
import edu.wpi.first.wpilibj.DutyCycleEncoder;
//[END ENCODER IMPORTS]
import frc.lib.util.SwerveModuleConstants;

public class PWMEncoder {
    //[BEGIN ENCODER PRIVATE VARIABLES]
    private final DutyCycleEncoder encoder;
    private final Rotation2d offset;
    //[END ENCODER PRIVATE VARIABLES]

    public PWMEncoder(SwerveModuleConstants moduleConstant) {
        //[BEGIN ENCODER INITIALIZATION]
        encoder = new DutyCycleEncoder(moduleConstant.encoderID);
        offset = moduleConstant.angleOffset;
        //[END ENCODER INITIALIZATION]
    }

    //[BEGIN GETABSOLUTEANGLE]
    public Rotation2d getAbsoluteAngle(){
        return Rotation2d.fromRotations(encoder.getAbsolutePosition() - offset.getRotations());
    }
    //[END GETABSOLUTEANGLE]
}
