package frc.robot.subsystems.apriltags;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
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
    private Pose2d[] centerPoses = new Pose2d[0];
    private Pose2d[] frontRightPoses = new Pose2d[0];
    private Pose2d[] centerBackPoses = new Pose2d[0];
    private final Timer timeSinceLastEstimate;

    public Localizer()
    {
        frontLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        centerPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        timeSinceLastEstimate = new Timer();
    }

    public void updateWithReferencePose(Pose2d pose)
    {
        double timestampSeconds = Timer.getFPGATimestamp();
        LimelightHelpers.SetIMUMode("limelight-fl", 0);
        LimelightHelpers.SetRobotOrientation("limelight-fl", pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode("limelight-ctr", 0);
        LimelightHelpers.SetRobotOrientation("limelight-ctr", pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        frontLeftPoseEstimator.addHeadingData(timestampSeconds, pose.getRotation());
        centerPoseEstimator.addHeadingData(timestampSeconds, pose.getRotation());
        frontLeftPoseEstimator.setReferencePose(pose);
        centerPoseEstimator.setReferencePose(pose);
    }

    public record EPOse(Pose2d pose, double timestamp) {
    }

    @Override
    public void periodic()
    {
        ArrayList<EPOse> poses = new ArrayList<>();
        ArrayList<Pose2d> frontRightPoses = new ArrayList<>();
        ArrayList<Pose2d> centerBackPoses = new ArrayList<>();
        ArrayList<Pose2d> frontLeftPoses = new ArrayList<>();
        ArrayList<Pose2d> centerPoses = new ArrayList<>();
        var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-fl");
        if (LimelightHelpers.validPoseEstimate(poseEstimate))
        {
            var p = new EPOse(poseEstimate.pose, poseEstimate.timestampSeconds);
            frontRightPoses.add(p.pose);
            poses.add(p);
        }
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-ctr");
        if (LimelightHelpers.validPoseEstimate(poseEstimate))
        {
            var p = new EPOse(poseEstimate.pose, poseEstimate.timestampSeconds);
            centerBackPoses.add(p.pose);
            poses.add(p);
        }
        for (PhotonPipelineResult result : frontLeftCamera.getAllUnreadResults())
        {
            var estimate = frontLeftPoseEstimator.update(result);
            if (estimate.isPresent())
            {
                var p = new EPOse(estimate.get().estimatedPose.toPose2d(), estimate.get().timestampSeconds);
                frontLeftPoses.add(p.pose);
                poses.add(p);
            }
        }
        for (PhotonPipelineResult result : centerCamera.getAllUnreadResults())
        {
            var estimate = centerPoseEstimator.update(result);
            if (estimate.isPresent())
            {
                var p = new EPOse(estimate.get().estimatedPose.toPose2d(), estimate.get().timestampSeconds);
                centerPoses.add(p.pose);
                poses.add(p);
            }
        }
        estimatedPoses = poses.toArray(new EPOse[0]);
        this.frontRightPoses = frontRightPoses.toArray(new Pose2d[0]);
        this.centerBackPoses = centerBackPoses.toArray(new Pose2d[0]);
        this.frontLeftPoses = frontLeftPoses.toArray(new Pose2d[0]);
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

    public Pose2d[] getFrontRightPoses()
    {
        return frontRightPoses;
    }

    public Pose2d[] getCenterPoses()
    {
        return centerPoses;
    }

    public Pose2d[] getFrontLeft()
    {
        return frontLeftPoses;
    }

    public Pose2d[] getCenterBackPoses()
    {
        return centerBackPoses;
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
