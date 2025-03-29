package frc.robot.subsystems.drive.requests;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CloseDriveToPoseRequest implements SwerveRequest
{
    private final FieldCentricFacingAngle facingAngle;
    private final PIDController xPID;
    private final PIDController yPID;
    private final LinearVelocity maxVelocity;

    public CloseDriveToPoseRequest(Pose2d pose, double tP, double tI, double tD, double rP, double rI, double rD,
            LinearVelocity maxVelocity)
    {
        this.xPID = new PIDController(tP, tI, tD);
        this.yPID = new PIDController(tP, tI, tD);
        xPID.setTolerance(0.02);
        yPID.setTolerance(0.02);
        xPID.setSetpoint(pose.getX());
        yPID.setSetpoint(pose.getY());
        this.facingAngle = new FieldCentricFacingAngle();
        facingAngle.HeadingController.setPID(rP, rI, rD);
        facingAngle.HeadingController.enableContinuousInput(0, Math.PI * 2);
        facingAngle.HeadingController.setTolerance(Math.toRadians(2));
        facingAngle.withTargetDirection(pose.getRotation());
        facingAngle.withDriveRequestType(DriveRequestType.Velocity);
        this.maxVelocity = maxVelocity;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply)
    {
        var vx = xPID.calculate(parameters.currentPose.getX());
        var vy = yPID.calculate(parameters.currentPose.getY());
        facingAngle
                .withVelocityX(MathUtil.clamp(vx, -maxVelocity.in(MetersPerSecond), maxVelocity.in(MetersPerSecond)));
        facingAngle
                .withVelocityY(MathUtil.clamp(vy, -maxVelocity.in(MetersPerSecond), maxVelocity.in(MetersPerSecond)));
        facingAngle.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
        return facingAngle.apply(parameters, modulesToApply);
    }

    public boolean isFinished()
    {
        return xPID.atSetpoint() && yPID.atSetpoint() && facingAngle.HeadingController.atSetpoint();
    }

}