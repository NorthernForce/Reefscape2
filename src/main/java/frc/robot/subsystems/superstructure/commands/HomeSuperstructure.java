package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.superstructure.Superstructure;

public class HomeSuperstructure extends ParallelCommandGroup
{

    public HomeSuperstructure(Superstructure superstructure)
    {
        addRequirements(superstructure);
        addCommands(superstructure.getInnerElevator().home(), superstructure.getOuterElevator().home());
    }
}
