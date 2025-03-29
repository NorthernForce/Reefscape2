package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public class RobotConstants
{
    public static class DriveConstants
    {
        public static final double kPPDriveTP = 10.0;
        public static final double kPPDriveTI = 0.0;
        public static final double kPPDriveTD = 0.0;
        public static final PIDConstants kPPDrivePID = new PIDConstants(kPPDriveTP, kPPDriveTI, kPPDriveTD);

        public static final double kPPDriveRP = 7.0;
        public static final double kPPDriveRI = 0.0;
        public static final double kPPDriveRD = 0.0;
        public static final PIDConstants kPPDriveRPID = new PIDConstants(kPPDriveRP, kPPDriveRI, kPPDriveRD);

        public static final Distance kDriveRadius = Inches.of(12.3645);

        public static final AngularVelocity kMaxAngularVelocity = RadiansPerSecond
                .of(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / kDriveRadius.in(Meters));

        public static final LinearVelocity kPPMaxVelocity = MetersPerSecond.of(3.0);
        public static final LinearAcceleration kPPMaxAcceleration = MetersPerSecondPerSecond.of(3.0);
        public static final AngularVelocity kPPMaxAngularVelocity = RadiansPerSecond.of(3.0);
        public static final AngularAcceleration kPPMaxAngularAcceleration = RadiansPerSecondPerSecond.of(3.0);
        public static final Voltage kNominalVoltage = Volts.of(12.0);
        public static final PathConstraints kPPConstraints = new PathConstraints(kPPMaxVelocity, kPPMaxAcceleration,
                kPPMaxAngularVelocity, kPPMaxAngularAcceleration);

        public static final double kCloseDriveTP = 2.9;
        public static final double kCloseDriveTI = 0.0;
        public static final double kCloseDriveTD = 0.0;

        public static final double kCloseDriveRP = 5.0;
        public static final double kCloseDriveRI = 0.0;
        public static final double kCloseDriveRD = 0.0;

        public static final Translation2d kPlacingOffset = new Translation2d(Inches.of(2.5), Inches.of(-9.75));
    }
}
