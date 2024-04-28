package frc.robot.subsystems.SwerveParts.Gyro;

//[BEGIN GYRO IMPORTS]
import com.kauailabs.navx.frc.AHRS;
//[END GYRO IMPORTS]

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;

public class NavXGyro {
    //[BEGIN GYRO PRIVATE VARIABLES]
    private final AHRS gyro;
    //[END GYRO PRIVATE VARIABLES]

    public NavXGyro() {
        //[BEGIN GYRO INITIALIZATION]
        gyro = new AHRS(SerialPort.Port.kMXP);

        gyro.zeroYaw();
        //[END GYRO INITIALIZATION]
    }

    //[BEGIN GETYAW]
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }
    //[END GETYAW]
}
