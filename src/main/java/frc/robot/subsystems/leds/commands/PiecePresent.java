package frc.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.leds.LEDS;

public class PiecePresent extends Command
{
    private final LEDS leds;

    public PiecePresent(LEDS leds)
    {
        addRequirements(leds);
        this.leds = leds;
    }

    @Override
    public void initialize()
    {
        leds.setColour(RobotConstants.LEDConstants.kTeamColor);
    }

    @Override
    public void end(boolean interrupted)
    {
        leds.clearAnimationBuffer();
    }

    @Override
    public boolean runsWhenDisabled()
    {
        return true;
    }
}
