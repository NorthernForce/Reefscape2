package frc.robot.subsystems.manipulator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.manipulator.Manipulator;

public class IntakeWhileWaiting extends Command {
    private final double m_speed;
    private final Manipulator m_manipulator;

    public IntakeWhileWaiting(Manipulator manipulator, double speed)
    {
        addRequirements(manipulator);
        m_speed = speed;
        m_manipulator = manipulator;
    }

    public IntakeWhileWaiting(Manipulator manipulator)
    {
        this(manipulator, RobotConstants.ManipulatorConstants.kIntakeSpeed);
    }

    @Override
    public void execute()
    {
        if (m_manipulator.hasCoral())
        {
            m_manipulator.stop();
        } else {
            m_manipulator.set(m_speed);
        }
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
