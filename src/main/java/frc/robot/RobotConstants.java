package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorConfig;

public class RobotConstants
{
    public static class DriveConstants
    {
        public static final double kPPDriveTP = 2.9;
        public static final double kPPDriveTI = 0.0;
        public static final double kPPDriveTD = 0.0;
        public static final PIDConstants kPPDrivePID = new PIDConstants(kPPDriveTP, kPPDriveTI, kPPDriveTD);

        public static final double kPPDriveRP = 5.0;
        public static final double kPPDriveRI = 0.0;
        public static final double kPPDriveRD = 0.0;
        public static final PIDConstants kPPDriveRPID = new PIDConstants(kPPDriveRP, kPPDriveRI, kPPDriveRD);

        public static final Distance kDriveRadius = Inches.of(12.3645);

        public static final AngularVelocity kMaxAngularVelocity = RadiansPerSecond
                .of(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / kDriveRadius.in(Meters));

        public static final LinearVelocity kPPMaxVelocity = MetersPerSecond.of(3.5);
        public static final LinearAcceleration kPPMaxAcceleration = MetersPerSecondPerSecond.of(3.0);
        public static final AngularVelocity kPPMaxAngularVelocity = RadiansPerSecond.of(3.0);
        public static final AngularAcceleration kPPMaxAngularAcceleration = RadiansPerSecondPerSecond.of(3.0);
        public static final Voltage kNominalVoltage = Volts.of(12.0);
        public static final PathConstraints kPPConstraints = new PathConstraints(kPPMaxVelocity, kPPMaxAcceleration,
                kPPMaxAngularVelocity, kPPMaxAngularAcceleration);

        public static final double kCloseDriveTP = 2.9;
        public static final double kCloseDriveTI = 0.0;
        public static final double kCloseDriveTD = 0.0;

        public static final double kCloseDriveRP = 5;
        public static final double kCloseDriveRI = 0.0;
        public static final double kCloseDriveRD = 0.0;

        // TODO: maybe tune a bit
        public static final double kCloseDriveVP = 5;
        public static final double kCloseDriveVI = 0.0;
        public static final double kCloseDriveVD = 0.0;

        public static final Translation2d kPlacingOffset = new Translation2d(Inches.of(2.5), Inches.of(-12.25));
        public static final Angle[] SWERVE_MODULE_OFFSETS =
        { Rotations.of(Preferences.getDouble("kSwerveOffsetFrontLeft", TunerConstants.FrontLeft.EncoderOffset)),
                Rotations.of(Preferences.getDouble("kSwerveOffsetFrontRight", TunerConstants.FrontRight.EncoderOffset)),
                Rotations.of(Preferences.getDouble("kSwerveOffsetBackLeft", TunerConstants.BackLeft.EncoderOffset)),
                Rotations.of(Preferences.getDouble("kSwerveOffsetBackRight", TunerConstants.BackRight.EncoderOffset)) };

    }

    public static class CameraConstants
    {
        public static final Transform3d kFrontRightCameraTransform = new Transform3d(
                new Translation3d(Inches.of(15.0 - 3.0), Inches.of(-(15.0 - 7.5)), Inches.of(8.5)),
                new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.zero()));

        public static final Transform3d kFrontLeftCameraTransform = new Transform3d(
                new Translation3d(Inches.of(15.0 - 3.0), Inches.of(15.0 - 7.75), Inches.of(8.5)),
                new Rotation3d(Degrees.zero(), Degrees.of(-22.1), Degrees.of(53.4)));

        public static final Transform3d kCenterCameraTransform = new Transform3d(
                new Translation3d(Inches.of(15.0 - 2.5), Inches.zero(), Inches.of(9.5)),
                new Rotation3d(Degrees.zero(), Degrees.of(-22.4), Degrees.zero()));
    }

    public static class ManipulatorConstants
    {
        public static final double kIntakeSpeed = 0.6;
        public static final double kOuttakeSpeed = 0.89;
        public static final double kSlowOuttakeSpeed = 0.4;
        public static final double kPurgeSpeed = 0.2;
        public static final int kMotorId = 19;
        public static final boolean kMotorInverted = false;
        public static final int kSensorId = 2;

    }

    public static class ClimberConstants
    {
        public static final int kId = 17;
        public static final boolean kInverted = true;
        public static final int kEncoderId = 23;
        public static final Angle kLowerLimit = Rotations.of(-0.03);
        public static final Angle kUpperLimit = Rotations.of(0.22);
        public static final Angle kSweetAngle = Degrees.of(70.0);
        public static final double kClimbSpeed = 1;
    }

    public static class ElevatorConstants
    {
        public static final double kHomingSpeed = 0.25;
        public static final Distance kTolerance = Inches.of(0.5);
    }

    public static class InnerElevatorConstants
    {
        // outer ratios
        public static final double kGearBoxRatio = 12.0;
        public static final double kSprocketTeeth = 16.0;
        public static final Distance kSproketPitch = Inches.of(0.25);
        public static final Distance kSprocketCircumference = kSproketPitch.times(kSprocketTeeth);

        // talon configs
        public static final double kS = 0.017384;
        public static final double kV = Units.inchesToMeters(28.59);
        public static final double kA = 0.015;
        public static final double kP = 18;
        public static final double kI = 0.0;
        public static final double kD = 0;
        public static final double kG = 0.21;
        public static final double kCruiseVelocity = 160;
        public static final double kAcceleration = 0;
        public static final double kJerk = 0;
        public static final Distance kLowerLimit = Inches.of(0.0);
        public static final Distance kUpperLimit = Inches.of(24.8);

        public static final Mass kInnerElevatorMass = Pounds.of(6.0);

        public static final ElevatorConfig kConfig = new ElevatorConfig(kS, kV, kA, kP, kI, kD, kG, kCruiseVelocity,
                kAcceleration, kJerk, kSprocketCircumference, kGearBoxRatio, true, kLowerLimit, kUpperLimit,
                kInnerElevatorMass);
    }

    public static class OuterElevatorConstants
    {
        // outer ratios
        public static final double kGearBoxRatio = 16.0;
        public static final double kSprocketTeeth = 22.0;
        public static final Distance kSprocketPitch = Inches.of(0.25);
        public static final Distance kSprocketCircumference = kSprocketPitch.times(kSprocketTeeth);

        // talon configs
        public static final double kS = 0.052289;
        public static final double kV = Units.inchesToMeters(19.868);
        public static final double kA = 0.015;
        public static final double kP = 10;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0.31;
        public static final double kCruiseVelocity = 160;
        public static final double kAcceleration = 0;
        public static final double kJerk = 0;
        public static final Distance kLowerLimit = Inches.of(0.0);
        public static final Distance kUpperLimit = Inches.of(26.8);

        public static final Mass kOuterElevatorMass = Pounds.of(14.0);

        public static final ElevatorConfig kConfig = new ElevatorConfig(kS, kV, kA, kP, kI, kD, kG, kCruiseVelocity,
                kAcceleration, kJerk, kSprocketCircumference, kGearBoxRatio, false, kLowerLimit, kUpperLimit,
                kOuterElevatorMass);
    }

    public static enum SuperstructureGoal
    {
        L1(Inches.of(0), Inches.of(2)), L2(Inches.of(0), Inches.of(11.38)), L3(Inches.of(0), Inches.of(26.3)),
        L4(InnerElevatorConstants.kUpperLimit, OuterElevatorConstants.kUpperLimit),
        CORAL_STATION(Inches.of(0), Inches.of(0)), START(Inches.of(0), Inches.of(0));

        private final SuperstructureState state;

        /**
         * Superstructure state constructor
         * 
         * @param innerHeight height of the inner elevator to go to
         * @param outerHeight height of the outer elevator to go to
         */

        private SuperstructureGoal(Distance innerHeight, Distance outerHeight)
        {
            this.state = new SuperstructureState(innerHeight, outerHeight);
        }

        public SuperstructureState getState()
        {
            return state;
        }
    }

    public static class AlgaeRemoverConstants
    {
        public static final double kRemovingSpeed = 0.8;
        public static final double kReturningSpeed = 0.2;
        public static final double kGearRatio = 10.0;
        public static final int kMotorId = 18;
        public static final int kSensorId = 3;
        public static final boolean kInverted = false;
    }

}
