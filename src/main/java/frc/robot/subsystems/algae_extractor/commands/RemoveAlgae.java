package frc.robot.subsystems.algae_extractor.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae_extractor.AlgaeExtractor;

public class RemoveAlgae extends Command
{
    private final AlgaeExtractor m_extractor;

    public RemoveAlgae(AlgaeExtractor extractor)
    {
        m_extractor = extractor;
        addRequirements(extractor);
    }

    @Override
    public void execute()
    {
        m_extractor.extractAlgae();
    }
}