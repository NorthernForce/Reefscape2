package frc.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.leds.LEDS;

public class Blinking extends Command
{
    private final LEDS leds;
    private final Timer timer = new Timer();
    private boolean ledState = false;

    public Blinking(LEDS leds)
    {
        addRequirements(leds);
        this.leds = leds;
    }

    @Override
    public void initialize()
    {
        timer.restart();
    }

    @Override
    public void execute()
    {
        if (timer.advanceIfElapsed(RobotConstants.LEDConstants.ledRate))
        {
            ledState = !ledState;
            if (ledState)
            {
                leds.setColour(RobotConstants.LEDConstants.kTeamColor);
            } else
            {
                leds.resetLEDS();
            }
        }
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
