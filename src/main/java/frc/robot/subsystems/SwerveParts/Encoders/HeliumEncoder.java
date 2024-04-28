package frc.robot.subsystems.SwerveParts.Encoders;

//[BEGIN ENCODER IMPORTS]
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
//[END ENCODER IMPORTS]

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.SwerveModuleConstants;

public class HeliumEncoder {
    //[BEGIN ENCODER PRIVATE VARIABLES]
    private final Canandcoder encoder;
    //[END ENCODER PRIVATE VARIABLES]

    public HeliumEncoder(SwerveModuleConstants moduleConstant) {
        //[BEGIN ENCODER INITIALIZATION]
        encoder = new Canandcoder(moduleConstant.encoderID);

        Canandcoder.Settings settings = new Canandcoder.Settings();
        settings.setZeroOffset(moduleConstant.angleOffset.getRotations());

        encoder.setSettings(settings);
        //[END ENCODER INITIALIZATION]
    }

    //[BEGIN GETABSOLUTEANGLE]
    public Rotation2d getAbsoluteAngle(){
        return Rotation2d.fromRotations(encoder.getAbsPosition());
    }
    //[END GETABSOLUTEANGLE]
}
