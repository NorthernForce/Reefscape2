package frc.robot.subsystems.leds.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDS;

public class Runway extends Command
{
    private final LEDS leds;
    private int tick = 0;

    public Runway(LEDS leds)
    {
        addRequirements(leds);
        this.leds = leds;
    }

    @Override
    public void execute()
    {
        leds.feedParticalEffect(0.5, tick);
        tick = (int) MathUtil.inputModulus(tick + 1, 0, 32);
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
