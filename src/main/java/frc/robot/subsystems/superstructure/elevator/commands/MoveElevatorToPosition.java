package frc.robot.subsystems.superstructure.elevator.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class MoveElevatorToPosition extends Command {
    private final Elevator elevator;
    private final Distance targetPosition;
    public MoveElevatorToPosition(Elevator elevator, Distance targetPosition)
    {
        addRequirements(elevator);
        this.elevator = elevator;
        this.targetPosition = targetPosition;
    }
    @Override
    public void initialize()
    {
        elevator.setTargetHeight(targetPosition);
    }
    @Override
    public void end(boolean interrupted)
    {
        elevator.stop();
    }
    @Override
    public boolean isFinished()
    {
        return elevator.getHeight().isNear(targetPosition, ElevatorConstants.kTolerance);
    }
}
