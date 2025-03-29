package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotConstants.SuperstructureGoal;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.commands.MoveElevatorToPosition;

public class MoveToState extends ParallelCommandGroup {
    public MoveToState(Superstructure superstructure, SuperstructureGoal goal)
    {
        addRequirements(superstructure);
        addCommands(new MoveElevatorToPosition(superstructure.getInnerElevator(), goal.getState().innerElevatorHeight()),
            new MoveElevatorToPosition(superstructure.getOuterElevator(), goal.getState().outerElevatorHeight()));
    }
}
