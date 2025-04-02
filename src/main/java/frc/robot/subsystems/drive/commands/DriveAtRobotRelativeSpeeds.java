package frc.robot.subsystems.drive.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class DriveAtRobotRelativeSpeeds extends Command
{
    private final ChassisSpeeds speeds;
    private Rotation2d targetAngle;
    private final CommandSwerveDrivetrain drive;
    private final SwerveRequest.RobotCentricFacingAngle robotCentric = new SwerveRequest.RobotCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withHeadingPID(5, 0, 0)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public DriveAtRobotRelativeSpeeds(CommandSwerveDrivetrain drive, ChassisSpeeds speeds)
    {
        addRequirements(drive);
        this.speeds = speeds;
        this.targetAngle = Rotation2d.kZero;
        this.drive = drive;
    }

    @Override
    public void initialize()
    {
        targetAngle = drive.getPose().getRotation();
    }

    @Override
    public void execute()
    {
        drive.setControl(robotCentric.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)
                .withTargetDirection(targetAngle));
    }
}
