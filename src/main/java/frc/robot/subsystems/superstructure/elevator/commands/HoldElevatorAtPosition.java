package frc.robot.subsystems.superstructure.elevator.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class HoldElevatorAtPosition extends Command
{
    private final Elevator elevator;
    private final Distance targetPosition;

    public HoldElevatorAtPosition(Elevator elevator, Distance targetPosition)
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
        return false;
    }
}
