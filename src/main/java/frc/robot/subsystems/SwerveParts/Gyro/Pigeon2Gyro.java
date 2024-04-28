package frc.robot.subsystems.SwerveParts.Gyro;

//[BEGIN GYRO IMPORTS]
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
//[END GYRO IMPORTS]

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2Gyro {
    //[BEGIN GYRO PRIVATE VARIABLES]
    private Pigeon2 pigeon;
    //[END GYRO PRIVATE VARIABLES]

    public Pigeon2Gyro(int id) {
        //[BEGIN GYRO INITIALIZATION]
        pigeon = new Pigeon2(id, "rio");

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.setYaw(0);
        //[END GYRO INITIALIZATION]
    }

    //[BEGIN GETYAW]
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValue());
    }
    //[END GETYAW]
}
