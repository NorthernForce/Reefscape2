package frc.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDS;

public class NoAllianceLEDs extends Command
{
    private final LEDS leds;

    public NoAllianceLEDs(LEDS leds)
    {
        addRequirements(leds);
        this.leds = leds;
    }

    @Override
    public void initialize()
    {
        leds.fire();
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
