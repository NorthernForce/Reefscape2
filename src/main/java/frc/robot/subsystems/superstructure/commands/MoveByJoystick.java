package frc.robot.subsystems.superstructure.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.superstructure.Superstructure;

public class MoveByJoystick extends ParallelCommandGroup {
    public MoveByJoystick(Superstructure superstructure, DoubleSupplier inner, DoubleSupplier outer)
    {
        addRequirements(superstructure);
        addCommands(
            superstructure.getInnerElevator().controlByJoystick(inner),
            superstructure.getOuterElevator().controlByJoystick(outer)
        );
    }
}
