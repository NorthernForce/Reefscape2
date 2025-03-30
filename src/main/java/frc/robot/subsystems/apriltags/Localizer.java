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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants.CameraConstants;

@Logged
public class Localizer extends SubsystemBase
{
    private final PhotonCamera frontLeftCamera = new PhotonCamera("front_right_camera");
    private final PhotonCamera frontRightCamera = new PhotonCamera("front_left_camera");
    private final PhotonCamera centerCamera = new PhotonCamera("center_camera");
    private final PhotonPoseEstimator frontLeftPoseEstimator = new PhotonPoseEstimator(FieldConstants.kField,
            PoseStrategy.CONSTRAINED_SOLVEPNP, CameraConstants.kFrontLeftCameraTransform);
    private final PhotonPoseEstimator frontRightPoseEstimator = new PhotonPoseEstimator(FieldConstants.kField,
            PoseStrategy.CONSTRAINED_SOLVEPNP, CameraConstants.kFrontRightCameraTransform);
    private final PhotonPoseEstimator centerPoseEstimator = new PhotonPoseEstimator(FieldConstants.kField,
            PoseStrategy.CONSTRAINED_SOLVEPNP, CameraConstants.kCenterCameraTransform);
    private EstimatedRobotPose[] estimatedPoses = new EstimatedRobotPose[0];
    private Pose2d[] frontLeftPoses = new Pose2d[0];
    private Pose2d[] frontRightPoses = new Pose2d[0];
    private Pose2d[] centerPoses = new Pose2d[0];
    private final VisionSystemSim sim;
    private final Timer timeSinceLastEstimate;

    public Localizer()
    {
        frontLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        frontRightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        centerPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        timeSinceLastEstimate = new Timer();
        if (RobotBase.isSimulation())
        {
            sim = new VisionSystemSim("main-vision");
            sim.addCamera(new PhotonCameraSim(centerCamera), CameraConstants.kCenterCameraTransform);
            sim.addCamera(new PhotonCameraSim(frontLeftCamera), CameraConstants.kFrontLeftCameraTransform);
            sim.addCamera(new PhotonCameraSim(frontRightCamera), CameraConstants.kFrontRightCameraTransform);
            sim.addAprilTags(FieldConstants.kField);
        } else
        {
            sim = null;
        }
    }

    public void updateWithReferencePose(Pose2d pose)
    {
        double timestampSeconds = Timer.getFPGATimestamp();
        frontLeftPoseEstimator.addHeadingData(timestampSeconds, pose.getRotation());
        frontRightPoseEstimator.addHeadingData(timestampSeconds, pose.getRotation());
        centerPoseEstimator.addHeadingData(timestampSeconds, pose.getRotation());
        frontLeftPoseEstimator.setReferencePose(pose);
        frontRightPoseEstimator.setReferencePose(pose);
        centerPoseEstimator.setReferencePose(pose);
        if (RobotBase.isSimulation())
        {
            sim.update(pose);
        }
    }

    @Override
    public void periodic()
    {
        ArrayList<EstimatedRobotPose> poses = new ArrayList<>();
        ArrayList<Pose2d> frontLeftPoses = new ArrayList<>();
        ArrayList<Pose2d> frontRightPoses = new ArrayList<>();
        ArrayList<Pose2d> centerPoses = new ArrayList<>();
        frontRightCamera.getAllUnreadResults().forEach(result ->
        {
            frontRightPoseEstimator.update(result).ifPresent(pose ->
            {
                poses.add(pose);
                frontRightPoses.add(pose.estimatedPose.toPose2d());
            });
        });
        frontLeftCamera.getAllUnreadResults().forEach(result ->
        {
            frontLeftPoseEstimator.update(result).ifPresent(pose ->
            {
                poses.add(pose);
                frontLeftPoses.add(pose.estimatedPose.toPose2d());
            });
        });
        centerCamera.getAllUnreadResults().forEach(result ->
        {
            centerPoseEstimator.update(result).ifPresent(pose ->
            {
                poses.add(pose);
                centerPoses.add(pose.estimatedPose.toPose2d());
            });
        });
        estimatedPoses = poses.toArray(new EstimatedRobotPose[0]);
        this.frontLeftPoses = frontLeftPoses.toArray(new Pose2d[0]);
        this.frontRightPoses = frontRightPoses.toArray(new Pose2d[0]);
        this.centerPoses = centerPoses.toArray(new Pose2d[0]);
        if (estimatedPoses.length > 0)
        {
            timeSinceLastEstimate.restart();
        }
    }

    public EstimatedRobotPose[] getEstimatedPoses()
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
