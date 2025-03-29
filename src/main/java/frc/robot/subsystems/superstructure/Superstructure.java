package frc.robot.subsystems.superstructure;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.InnerElevatorConstants;
import frc.robot.RobotConstants.OuterElevatorConstants;
import frc.robot.RobotConstants.SuperstructureGoal;
import frc.robot.subsystems.superstructure.commands.MoveByJoystick;
import frc.robot.subsystems.superstructure.commands.MoveToState;
import frc.robot.subsystems.superstructure.elevator.Elevator;

@Logged
public class Superstructure extends SubsystemBase
{
    public static record SuperstructureState(Distance innerElevatorHeight, Distance outerElevatorHeight) {
    }

    private final Elevator innerElevator;
    private final Elevator outerElevator;
    private SuperstructureGoal goal;

    public Superstructure()
    {
        innerElevator = new Elevator(15, 0, InnerElevatorConstants.kConfig);
        outerElevator = new Elevator(14, 1, OuterElevatorConstants.kConfig);
        goal = SuperstructureGoal.START;
    }

    public SuperstructureState getState()
    {
        return new SuperstructureState(innerElevator.getHeight(), outerElevator.getHeight());
    }

    public boolean isAtHeight(Distance innerElevatorHeight, Distance outerElevatorHeight)
    {
        return innerElevator.getHeight().isNear(innerElevatorHeight, ElevatorConstants.kTolerance)
                && outerElevator.getHeight().isNear(outerElevatorHeight, ElevatorConstants.kTolerance);
    }

    public boolean isAtHeight(SuperstructureState state)
    {
        return isAtHeight(state.innerElevatorHeight(), state.outerElevatorHeight());
    }

    public boolean isAtTargetState()
    {
        return isAtHeight(goal.getState());
    }

    public void setTargetState(SuperstructureGoal goal)
    {
        this.goal = goal;
    }

    public Command moveToGoal(SuperstructureGoal goal)
    {
        return runOnce(() -> setTargetState(goal)).andThen(new MoveToState(this, goal));
    }

    @NotLogged
    public Elevator getInnerElevator()
    {
        return innerElevator;
    }

    @NotLogged
    public Elevator getOuterElevator()
    {
        return outerElevator;
    }

    public Command moveToL1()
    {
        return moveToGoal(SuperstructureGoal.L1);
    }

    public Command moveToL2()
    {
        return moveToGoal(SuperstructureGoal.L2);
    }

    public Command moveToL3()
    {
        return moveToGoal(SuperstructureGoal.L3);
    }

    public Command moveToL4()
    {
        return moveToGoal(SuperstructureGoal.L4);
    }

    public Command moveToIntake()
    {
        return moveToGoal(SuperstructureGoal.CORAL_STATION);
    }

    public Command moveByJoystick(DoubleSupplier inner, DoubleSupplier outer)
    {
        return new MoveByJoystick(this, inner, outer);
    }

    public Command getHomingCommand()
    {
        return outerElevator.home().alongWith(innerElevator.home());
    }
}
