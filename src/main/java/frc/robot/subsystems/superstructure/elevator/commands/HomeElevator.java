package frc.robot.subsystems.superstructure.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class HomeElevator extends Command
{
    private final Elevator elevator;

    public HomeElevator(Elevator elevator)
    {
        addRequirements(elevator);
        this.elevator = elevator;
    }

    @Override
    public void initialize()
    {
        elevator.ignoreBottomLimit();
    }

    @Override
    public void execute()
    {
        elevator.set(-ElevatorConstants.kHomingSpeed);
    }

    @Override
    public void end(boolean interrupted)
    {
        elevator.stop();
        // elevator.resetPosition();
        elevator.useBottomLimit();
    }

    @Override
    public boolean isFinished()
    {
        return elevator.isAtBottomLimit();
    }
}
