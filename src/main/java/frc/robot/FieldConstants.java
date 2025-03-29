package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import static edu.wpi.first.units.Units.*;

import java.util.HashMap;

public class FieldConstants
{
    public static final Distance FIELD_LENGTH = Meters.of(17.55);
    public static final Distance FIELD_WIDTH = Meters.of(8.05);

    /**
     * Enum for the reef locations
     */
    public static enum ReefLocations
    {
        A, B, C, D, E, F, G, H, I, J, K, L, AB_ALGAE, CD_ALGAE, EF_ALGAE, GH_ALGAE, IJ_ALGAE, KL_ALGAE, AB_TROUGH,
        CD_TROUGH, EF_TROUGH, GH_TROUGH, IJ_TROUGH, KL_TROUGH, LEFT_CORAL_STATION, RIGHT_CORAL_STATION,
        PROCESSOR_STATION
    }

    public static record ReefSide(Pose2d left, Pose2d right, Pose2d center, Pose2d trough) {
    }

    public static class ReefRotations
    {
        public static final Rotation2d AB_ROTATION = Rotation2d.fromDegrees(0);
        public static final Rotation2d CD_ROTATION = Rotation2d.fromDegrees(60);
        public static final Rotation2d EF_ROTATION = Rotation2d.fromDegrees(120);
        public static final Rotation2d GH_ROTATION = Rotation2d.fromDegrees(180);
        public static final Rotation2d IJ_ROTATION = Rotation2d.fromDegrees(240);
        public static final Rotation2d KL_ROTATION = Rotation2d.fromDegrees(300);
    }

    public static Pose2d getReefBackupPosition(Pose2d original, Distance metersBackup)
    {
        double originalX = original.getX();
        double originalY = original.getY();
        Angle parallelAngle = Degrees.of(original.getRotation().getDegrees());
        double[] result = new double[2];
        result[0] = originalX + (-metersBackup.in(Meters)) * Math.cos((parallelAngle).in(Radians));
        result[1] = originalY + (-metersBackup.in(Meters)) * Math.sin((parallelAngle).in(Radians));
        return new Pose2d(result[0], result[1], original.getRotation());
    }

    /**
     * All poses are BLUE relative
     */
    public static class ReefPositions
    {
        public static final Translation2d REEF_CENTER = new Translation2d(4.489323, 4.0259);
        public static final Pose2d A = new Pose2d(3.15, 4.18, ReefRotations.AB_ROTATION);
        public static final Pose2d AB_ALGAE = new Pose2d(3.15, 4.02, ReefRotations.AB_ROTATION);
        public static final Pose2d B = new Pose2d(3.15, 3.85, ReefRotations.AB_ROTATION);
        public static final Pose2d C = new Pose2d(3.69, 2.96, ReefRotations.CD_ROTATION);
        public static final Pose2d CD_ALGAE = new Pose2d(3.79, 2.86, ReefRotations.CD_ROTATION);
        public static final Pose2d D = new Pose2d(3.94, 2.79, ReefRotations.CD_ROTATION);
        public static final Pose2d E = new Pose2d(5.01, 2.81, ReefRotations.EF_ROTATION);
        public static final Pose2d EF_ALGAE = new Pose2d(5.14, 2.87, ReefRotations.EF_ROTATION);
        public static final Pose2d F = new Pose2d(5.29, 2.96, ReefRotations.EF_ROTATION);
        public static final Pose2d G = new Pose2d(5.85, 3.87, ReefRotations.GH_ROTATION);
        public static final Pose2d GH_ALGAE = new Pose2d(5.85, 4.02, ReefRotations.GH_ROTATION);
        public static final Pose2d H = new Pose2d(5.85, 4.18, ReefRotations.GH_ROTATION);
        public static final Pose2d I = new Pose2d(5.29, 5.12, ReefRotations.IJ_ROTATION);
        public static final Pose2d IJ_ALGAE = new Pose2d(5.14, 5.19, ReefRotations.IJ_ROTATION);
        public static final Pose2d J = new Pose2d(5.01, 5.29, ReefRotations.IJ_ROTATION);
        public static final Pose2d K = new Pose2d(3.95, 5.29, ReefRotations.KL_ROTATION);
        public static final Pose2d KL_ALGAE = new Pose2d(3.82, 5.19, ReefRotations.KL_ROTATION);
        public static final Pose2d L = new Pose2d(3.65, 5.12, ReefRotations.KL_ROTATION);
        public static final Pose2d AB_TROUGH = new Pose2d(3.531, 5.203,
                ReefRotations.AB_ROTATION.plus(Rotation2d.kCW_90deg));
        public static final Pose2d CD_TROUGH = AB_TROUGH.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(60));
        public static final Pose2d EF_TROUGH = AB_TROUGH.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(120));
        public static final Pose2d GH_TROUGH = AB_TROUGH.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(180));
        public static final Pose2d IJ_TROUGH = AB_TROUGH.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(240));
        public static final Pose2d KL_TROUGH = AB_TROUGH.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(300));
        public static final ReefSide AB_SIDE = new ReefSide(A, B, AB_ALGAE, CD_TROUGH);
        public static final ReefSide CD_SIDE = new ReefSide(C, D, CD_ALGAE, EF_TROUGH);
        public static final ReefSide EF_SIDE = new ReefSide(E, F, EF_ALGAE, GH_TROUGH);
        public static final ReefSide GH_SIDE = new ReefSide(G, H, GH_ALGAE, IJ_TROUGH);
        public static final ReefSide IJ_SIDE = new ReefSide(I, J, IJ_ALGAE, KL_TROUGH);
        public static final ReefSide KL_SIDE = new ReefSide(K, L, KL_ALGAE, AB_TROUGH);

        public static final ReefSide[] REEF_SIDES =
        { AB_SIDE, CD_SIDE, EF_SIDE, GH_SIDE, IJ_SIDE, KL_SIDE };
    }

    public static final HashMap<ReefLocations, Pose2d> REEF_POSITIONS = new HashMap<>();
    static
    {
        REEF_POSITIONS.put(ReefLocations.A, ReefPositions.A);
        REEF_POSITIONS.put(ReefLocations.B, ReefPositions.B);
        REEF_POSITIONS.put(ReefLocations.C, ReefPositions.C);
        REEF_POSITIONS.put(ReefLocations.D, ReefPositions.D);
        REEF_POSITIONS.put(ReefLocations.E, ReefPositions.E);
        REEF_POSITIONS.put(ReefLocations.F, ReefPositions.F);
        REEF_POSITIONS.put(ReefLocations.G, ReefPositions.G);
        REEF_POSITIONS.put(ReefLocations.H, ReefPositions.H);
        REEF_POSITIONS.put(ReefLocations.I, ReefPositions.I);
        REEF_POSITIONS.put(ReefLocations.J, ReefPositions.J);
        REEF_POSITIONS.put(ReefLocations.K, ReefPositions.K);
        REEF_POSITIONS.put(ReefLocations.L, ReefPositions.L);
        REEF_POSITIONS.put(ReefLocations.AB_ALGAE, ReefPositions.AB_ALGAE);
        REEF_POSITIONS.put(ReefLocations.CD_ALGAE, ReefPositions.CD_ALGAE);
        REEF_POSITIONS.put(ReefLocations.EF_ALGAE, ReefPositions.EF_ALGAE);
        REEF_POSITIONS.put(ReefLocations.GH_ALGAE, ReefPositions.GH_ALGAE);
        REEF_POSITIONS.put(ReefLocations.IJ_ALGAE, ReefPositions.IJ_ALGAE);
        REEF_POSITIONS.put(ReefLocations.KL_ALGAE, ReefPositions.KL_ALGAE);
        REEF_POSITIONS.put(ReefLocations.AB_TROUGH, ReefPositions.AB_TROUGH);
        REEF_POSITIONS.put(ReefLocations.CD_TROUGH, ReefPositions.CD_TROUGH);
        REEF_POSITIONS.put(ReefLocations.EF_TROUGH, ReefPositions.EF_TROUGH);
        REEF_POSITIONS.put(ReefLocations.GH_TROUGH, ReefPositions.GH_TROUGH);
        REEF_POSITIONS.put(ReefLocations.IJ_TROUGH, ReefPositions.IJ_TROUGH);
        REEF_POSITIONS.put(ReefLocations.KL_TROUGH, ReefPositions.KL_TROUGH);
        REEF_POSITIONS.put(ReefLocations.LEFT_CORAL_STATION, CoralStations.LEFT);
        REEF_POSITIONS.put(ReefLocations.RIGHT_CORAL_STATION, CoralStations.RIGHT);
        REEF_POSITIONS.put(ReefLocations.PROCESSOR_STATION, ProcessorStations.PROCESSOR_STATION);
    }

    /**
     * All poses are BLUE relative, and left and right are from the perspective of
     * the drive station
     */
    public static class CoralStations
    {
        public static final Pose2d LEFT = new Pose2d(1.18, 7.07, Rotation2d.fromDegrees(307.5));
        public static final Pose2d RIGHT = new Pose2d(1.11, 1.00, Rotation2d.fromDegrees(52.5));
    }

    /**
     * All poses are BLUE relative
     */
    public static class ProcessorStations
    {
        public static final Pose2d PROCESSOR_STATION = new Pose2d(6.34, 0.44, Rotation2d.fromDegrees(90));
    }

    /**
     * Gets a rotation based on the alliance
     * 
     * @param alliance The alliance color
     * @return The rotation (0 for blue, 180 for red)
     */
    public static Rotation2d getFieldRotation(Alliance alliance)
    {
        if (alliance == Alliance.Blue)
        {
            return Rotation2d.fromDegrees(0);
        } else
        {
            return Rotation2d.fromDegrees(180);
        }
    }

    public static Pose2d convertPoseByAlliance(Pose2d pose, Alliance alliance)
    {
        if (alliance == Alliance.Blue)
        {
            return pose;
        } else
        {
            return new Pose2d(FIELD_LENGTH.in(Meters) - pose.getX(), FIELD_WIDTH.in(Meters) - pose.getY(),
                    pose.getRotation().plus(Rotation2d.fromDegrees(180)));
        }
    }

    public static Pose2d convertPoseByAlliance(Pose2d pose)
    {
        return convertPoseByAlliance(pose, getAlliance());
    }

    public static Pose2d applyOffset(Pose2d pose, Translation2d offset)
    {
        Translation2d translation = offset.rotateBy(pose.getRotation());
        return new Pose2d(pose.getTranslation().plus(translation), pose.getRotation());
    }

    public static Translation2d convertTranslationByAlliance(Translation2d pose, Alliance alliance)
    {
        if (alliance == Alliance.Blue)
        {
            return pose;
        } else
        {
            return new Translation2d(FIELD_LENGTH.in(Meters) - pose.getX(), FIELD_WIDTH.in(Meters) - pose.getY());
        }
    }

    public static Translation2d convertTranslationByAlliance(Translation2d pose)
    {
        return convertTranslationByAlliance(pose, getAlliance());
    }

    public static ReefSide convertReefSideByAlliance(ReefSide side, Alliance alliance)
    {
        if (alliance == Alliance.Blue)
        {
            return side;
        } else
        {
            return new ReefSide(convertPoseByAlliance(side.left), convertPoseByAlliance(side.right),
                    convertPoseByAlliance(side.center), convertPoseByAlliance(side.trough));
        }
    }

    public static ReefSide convertReefSideByAlliance(ReefSide side)
    {
        return convertReefSideByAlliance(side, getAlliance());
    }

    public static Alliance getAlliance()
    {
        return DriverStation.getAlliance().orElse(Alliance.Red);
    }

    public static class CoralRotations
    {
        public static final Rotation2d BLUE_LEFT = Rotation2d.fromDegrees(120);
        public static final Rotation2d BLUE_RIGHT = Rotation2d.fromDegrees(240);
        public static final Rotation2d RED_LEFT = Rotation2d.fromDegrees(60);
        public static final Rotation2d RED_RIGHT = Rotation2d.fromDegrees(300);
    }

    public static boolean isAtCoralRotation(Rotation2d rotation)
    {
        double degrees = rotation.getDegrees();
        degrees = MathUtil.inputModulus(degrees, 0, 360);
        if (getAlliance() == Alliance.Red)
        {
            return Math.abs(degrees - CoralRotations.RED_LEFT.getDegrees()) <= 10
                    || Math.abs(degrees - CoralRotations.RED_RIGHT.getDegrees()) <= 10;
        } else
        {
            return Math.abs(degrees - CoralRotations.BLUE_LEFT.getDegrees()) <= 10
                    || Math.abs(degrees - CoralRotations.BLUE_RIGHT.getDegrees()) <= 10;
        }
    }

    public static final AprilTagFieldLayout kField = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeAndyMark);
}