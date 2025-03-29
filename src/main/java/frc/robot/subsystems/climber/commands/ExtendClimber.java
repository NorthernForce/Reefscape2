package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class ExtendClimber extends Command
{
    private final Climber m_climber;

    public ExtendClimber(Climber climber)
    {
        m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize()
    {
        m_climber.extend();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_climber.stop();
    }
}
