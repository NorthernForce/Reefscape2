package frc.robot.subsystems.manipulator.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;

public class Outtake extends Command
{
    private final Manipulator manip;

    public Outtake(Manipulator m)
    {
        addRequirements(m);
        this.manip = m;
    }

    @Override
    public void initialize()
    {
        this.manip.setState(ManipulatorState.OUTTAKING);
    }

    @Override
    public boolean isFinished()
    {
        return !this.manip.hasCoralInSensor();
    }

    @Override
    public void execute()
    {
    }
}
