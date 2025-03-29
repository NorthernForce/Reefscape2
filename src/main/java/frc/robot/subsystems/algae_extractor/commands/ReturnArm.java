package frc.robot.subsystems.algae_extractor.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae_extractor.AlgaeExtractor;

public class ReturnArm extends Command
{
    private final AlgaeExtractor m_extractor;

    public ReturnArm(AlgaeExtractor extractor)
    {
        m_extractor = extractor;
        addRequirements(extractor);
    }

    @Override
    public void execute()
    {
        if (!m_extractor.hasReachedTop())
        {
            m_extractor.returnExtractor();
        } else
        {
            m_extractor.stop();
        }
    }

    @Override
    public boolean isFinished()
    {
        return m_extractor.hasReachedTop();
    }

    @Override
    public void end(boolean interrupted)
    {
    }
}