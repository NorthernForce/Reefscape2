package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotConstants.SuperstructureGoal;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.commands.HoldElevatorAtPosition;

public class HoldAtState extends ParallelCommandGroup
{
    public HoldAtState(Superstructure superstructure, SuperstructureGoal goal)
    {
        addRequirements(superstructure);
        addCommands(
                new HoldElevatorAtPosition(superstructure.getInnerElevator(), goal.getState().innerElevatorHeight()),
                new HoldElevatorAtPosition(superstructure.getOuterElevator(), goal.getState().outerElevatorHeight()));
    }
}