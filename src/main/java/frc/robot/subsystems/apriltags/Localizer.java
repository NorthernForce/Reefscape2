package frc.robot.subsystems.apriltags;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotConstants.CameraConstants;

@Logged
public class Localizer extends SubsystemBase
{
    private final PhotonCamera frontLeftCamera = new PhotonCamera("front_left_camera");
    private final PhotonCamera centerCamera = new PhotonCamera("center_camera");
    private final PhotonPoseEstimator frontLeftPoseEstimator = new PhotonPoseEstimator(FieldConstants.kField,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.kFrontLeftCameraTransform);
    private final PhotonPoseEstimator centerPoseEstimator = new PhotonPoseEstimator(FieldConstants.kField,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.kCenterCameraTransform);
    private EPOse[] estimatedPoses = new EPOse[0];
    private Pose2d[] frontLeftPoses = new Pose2d[0];
    private Pose2d[] frontRightPoses = new Pose2d[0];
    private Pose2d[] centerPoses = new Pose2d[0];
    private final VisionSystemSim sim;
    private final Timer timeSinceLastEstimate;

    public Localizer()
    {
        frontLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        centerPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        timeSinceLastEstimate = new Timer();
        if (RobotBase.isSimulation())
        {
            sim = new VisionSystemSim("main-vision");
            sim.addCamera(new PhotonCameraSim(centerCamera), CameraConstants.kCenterCameraTransform);
            sim.addCamera(new PhotonCameraSim(frontLeftCamera), CameraConstants.kFrontLeftCameraTransform);
            sim.addAprilTags(FieldConstants.kField);
        } else
        {
            sim = null;
        }
    }

    public void updateWithReferencePose(Pose2d pose)
    {
        double timestampSeconds = Timer.getFPGATimestamp();
        LimelightHelpers.SetIMUMode("limelight-fl", 0);
        LimelightHelpers.SetRobotOrientation("limelight-fl", pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        frontLeftPoseEstimator.addHeadingData(timestampSeconds, pose.getRotation());
        centerPoseEstimator.addHeadingData(timestampSeconds, pose.getRotation());
        frontLeftPoseEstimator.setReferencePose(pose);
        centerPoseEstimator.setReferencePose(pose);
        if (RobotBase.isSimulation())
        {
            sim.update(pose);
        }
    }

    public record EPOse(Pose2d pose, double timestamp) {
    }

    @Override
    public void periodic()
    {
        ArrayList<EPOse> poses = new ArrayList<>();
        ArrayList<Pose2d> frontLeftPoses = new ArrayList<>();
        ArrayList<Pose2d> frontRightPoses = new ArrayList<>();
        ArrayList<Pose2d> centerPoses = new ArrayList<>();
        var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-fl");
        if (LimelightHelpers.validPoseEstimate(poseEstimate))
        {
            var p = new EPOse(poseEstimate.pose, poseEstimate.timestampSeconds);
            frontRightPoses.add(p.pose);
            poses.add(p);
        }
        if (frontLeftCamera.isConnected())
            frontLeftCamera.getAllUnreadResults().forEach(result ->
            {
                frontLeftPoseEstimator.update(result).ifPresent(pose ->
                {
                    poses.add(new EPOse(pose.estimatedPose.toPose2d(), pose.timestampSeconds));
                    frontLeftPoses.add(pose.estimatedPose.toPose2d());
                });
            });
        if (centerCamera.isConnected())
            centerCamera.getAllUnreadResults().forEach(result ->
            {
                centerPoseEstimator.update(result).ifPresent(pose ->
                {
                    poses.add(new EPOse(pose.estimatedPose.toPose2d(), pose.timestampSeconds));
                    centerPoses.add(pose.estimatedPose.toPose2d());
                });
            });
        estimatedPoses = poses.toArray(new EPOse[0]);
        this.frontLeftPoses = frontLeftPoses.toArray(new Pose2d[0]);
        this.frontRightPoses = frontRightPoses.toArray(new Pose2d[0]);
        this.centerPoses = centerPoses.toArray(new Pose2d[0]);
        if (estimatedPoses.length > 0)
        {
            timeSinceLastEstimate.restart();
        }
    }

    public EPOse[] getEstimatedPoses()
    {
        return estimatedPoses;
    }

    public Pose2d[] getFrontLeftPoses()
    {
        return frontLeftPoses;
    }

    public Pose2d[] getFrontRightPoses()
    {
        return frontRightPoses;
    }

    public Pose2d[] getCenterPoses()
    {
        return centerPoses;
    }

    public double getTimeSinceLastEstimate()
    {
        return timeSinceLastEstimate.get();
    }

    public boolean hasHadRecentEstimate()
    {
        return timeSinceLastEstimate.hasElapsed(0.5) && timeSinceLastEstimate.isRunning();
    }
}
