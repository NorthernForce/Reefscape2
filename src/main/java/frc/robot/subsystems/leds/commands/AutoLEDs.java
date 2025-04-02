package frc.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDS;

public class AutoLEDs extends Command
{
    private final LEDS leds;

    public AutoLEDs(LEDS leds)
    {
        addRequirements(leds);
        this.leds = leds;
    }

    @Override
    public void initialize()
    {
        leds.rainbowAnimation(0.75, 0.1);
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
