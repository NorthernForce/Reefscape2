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
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.kFrontLeftCameraTransform);
    private final PhotonPoseEstimator frontRightPoseEstimator = new PhotonPoseEstimator(FieldConstants.kField,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.kFrontRightCameraTransform);
    private final PhotonPoseEstimator centerPoseEstimator = new PhotonPoseEstimator(FieldConstants.kField,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.kCenterCameraTransform);
    private EstimatedRobotPose[] estimatedPoses = new EstimatedRobotPose[0];
    private final VisionSystemSim sim;

    public Localizer()
    {
        frontLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        frontRightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        centerPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
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
        frontRightCamera.getAllUnreadResults().forEach(result ->
        {
            frontRightPoseEstimator.update(result).ifPresent(pose -> poses.add(pose));
        });
        frontLeftCamera.getAllUnreadResults().forEach(result ->
        {
            frontLeftPoseEstimator.update(result).ifPresent(pose -> poses.add(pose));
        });
        centerCamera.getAllUnreadResults().forEach(result ->
        {
            centerPoseEstimator.update(result).ifPresent(pose -> poses.add(pose));
        });
        estimatedPoses = poses.toArray(new EstimatedRobotPose[0]);
    }

    public EstimatedRobotPose[] getEstimatedPoses()
    {
        return estimatedPoses;
    }
}
