package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.TunerConstants;

public class RobotConstants
{
    public static class DriveConstants
    {
        public static final double kPPDriveTP = 10.0;
        public static final double kPPDriveTI = 0.0;
        public static final double kPPDriveTD = 0.0;

        public static final double kPPDriveRP = 7.0;
        public static final double kPPDriveRI = 0.0;
        public static final double kPPDriveRD = 0.0;

        public static final Distance kDriveRadius = Inches.of(12.3645);

        public static final AngularVelocity kMaxAngularVelocity = RadiansPerSecond
                .of(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / kDriveRadius.in(Meters));
    }
}
