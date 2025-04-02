package frc.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDS;

public class EveryOther extends Command
{
    private final LEDS leds;
    private final Color color;

    public EveryOther(LEDS leds, Color color)
    {
        addRequirements(leds);
        this.leds = leds;
        this.color = color;
    }

    @Override
    public void initialize()
    {
        leds.everyOther(color);
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
