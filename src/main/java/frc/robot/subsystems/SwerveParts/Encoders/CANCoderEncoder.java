package frc.robot.subsystems.SwerveParts.Encoders;

//[BEGIN ENCODER IMPORTS]
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
//[END ENCODER IMPORTS]

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.SwerveModuleConstants;

public class CANCoderEncoder {
    //[BEGIN ENCODER PRIVATE VARIABLES]
    private final CANcoder cancoder;
    private final Rotation2d offset;
    //[END ENCODER PRIVATE VARIABLES]

    public CANCoderEncoder(SwerveModuleConstants moduleConstant) {
        //[BEGIN ENCODER INITIALIZATION]
        cancoder = new CANcoder(moduleConstant.encoderID);

        CANcoderConfiguration configuration = new CANcoderConfiguration();
        cancoder.getConfigurator().apply(configuration);

        offset = moduleConstant.angleOffset;
        //[END ENCODER INITIALIZATION]
    }

    //[BEGIN GETABSOLUTEANGLE]
    public Rotation2d getAbsoluteAngle(){
        return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValue() - offset.getRotations());
    }
    //[END GETABSOLUTEANGLE]
}
