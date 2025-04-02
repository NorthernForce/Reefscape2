package frc.robot.subsystems.manipulator.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.Manipulator;

public class Intake extends Command
{
    private final Manipulator manipulator;

    public Intake(Manipulator manipulator)
    {
        addRequirements(manipulator);
        this.manipulator = manipulator;
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
    }

    @Override
    public boolean isFinished()
    {
        return this.manipulator.hasCoral();
    }
}
