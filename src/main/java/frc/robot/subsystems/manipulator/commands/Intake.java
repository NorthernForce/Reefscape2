package frc.robot.subsystems.manipulator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.manipulator.Manipulator;

public class Intake extends Command
{
    private final double m_speed;
    private final Manipulator m_manipulator;

    public Intake(Manipulator manipulator, double speed)
    {
        addRequirements(manipulator);
        m_speed = speed;
        m_manipulator = manipulator;
    }

    public Intake(Manipulator manipulator)
    {
        this(manipulator, RobotConstants.ManipulatorConstants.kIntakeSpeed);
    }

    @Override
    public void initialize()
    {
        m_manipulator.set(m_speed);
    }

    @Override
    public boolean isFinished()
    {
        return m_manipulator.hasCoral();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_manipulator.stop();
    }
}
