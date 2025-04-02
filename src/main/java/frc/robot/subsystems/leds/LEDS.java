package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.leds.commands.AutoLEDs;
import frc.robot.subsystems.leds.commands.Blinking;
import frc.robot.subsystems.leds.commands.EndgameLEDs;
import frc.robot.subsystems.leds.commands.EveryOther;
import frc.robot.subsystems.leds.commands.NoAllianceLEDs;
import frc.robot.subsystems.leds.commands.PiecePresent;
import frc.robot.subsystems.leds.commands.Runway;

@Logged
public class LEDS extends SubsystemBase
{
    private CANdle candle;
    private CANdleConfiguration config;
    private int ledCount;

    public static enum GameState
    {
        NONE, NO_ALLIANCE, BLUE_ALLIANCE, RED_ALLIANCE, AUTO, TELEOP, ENDGAME, HASPIECE, WANTSPIECE, READYPLACE
    };

    private int currentAnimationTick = 0;

    public void resetLEDS()
    {
        candle.setLEDs(0, 0, 0, 0, 0, ledCount);
    }

    public LEDS(int CANid, int ledCount)
    {
        candle = new CANdle(CANid);
        config = new CANdleConfiguration();
        this.ledCount = ledCount;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.75;
        candle.configAllSettings(config);
    }

    public void rainbowAnimation(double brightness, double animationSpeed)
    {
        RainbowAnimation rainbowAnim = new RainbowAnimation(brightness, animationSpeed, ledCount);
        candle.animate(rainbowAnim);
    }

    public void setColour(Color color, int start, int length)
    {
        candle.setLEDs((int) (255 * color.red), (int) (255 * color.green), (int) (255 * color.blue), 0, start, length);
    }

    public void setColour(Color colour)
    {
        setColour(colour, 0, ledCount);
    }

    public int[] particalTranslation(int tick)
    {
        if (tick == 0)
        {
            int[] returnArray =
            { (62 - 6) };
            return returnArray;
        } else if (tick > 0 && tick < 31)
        {
            int[] returnArray =
            { (int) MathUtil.inputModulus((62 - 6) - tick, 0, 62),
                    (int) MathUtil.inputModulus((62 - 6) + tick, 0, 62) };
            return returnArray;
        } else if (tick == 31)
        {
            int[] returnArray =
            { 31 + 6 };
            return returnArray;
        }

        return new int[0];
    }

    public void feedParticalEffect(double brightness, int tick)
    {
        candle.setLEDs(0, 0, 0, 0, 0, ledCount);
        int[] particalReturn = particalTranslation(tick);
        for (int i = 0; i < particalReturn.length; i++)
        {
            setColour(RobotConstants.LEDConstants.kTeamColor, particalReturn[i] + 8, 1);
        }
        int[] nextParticle0 = particalTranslation((int) MathUtil.inputModulus(currentAnimationTick + 4, 0, 32));
        for (int i = 0; i < nextParticle0.length; i++)
        {
            setColour(RobotConstants.LEDConstants.kTeamColor, particalReturn[i] + 8, 1);
        }
        int[] nextParticle1 = particalTranslation((int) MathUtil.inputModulus(currentAnimationTick + 8, 0, 32));
        for (int i = 0; i < nextParticle1.length; i++)
        {
            setColour(RobotConstants.LEDConstants.kTeamColor, particalReturn[i] + 8, 1);
        }
        int[] nextParticle2 = particalTranslation((int) MathUtil.inputModulus(currentAnimationTick + 12, 0, 32));
        for (int i = 0; i < nextParticle2.length; i++)
        {
            setColour(RobotConstants.LEDConstants.kTeamColor, particalReturn[i] + 8, 1);
        }
    }

    public void hasPiece()
    {
        candle.setLEDs(255, 0, 255, 0, 0, ledCount);
    }

    public void setEveryOtherColour(Color inputColour1, Color inputColour2)
    {
        for (int i = 0; i < ledCount; i++)
        {
            if (i % 2 == 0)
            {
                setColour(inputColour1, i, 1);
            } else
            {
                setColour(inputColour2, i, 1);
            }
        }
    }

    public void everyOther(Color allianceColour)
    {
        setEveryOtherColour(allianceColour, RobotConstants.LEDConstants.kTeamColor);
    }

    public void clearAnimationBuffer()
    {
        for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++)
        {
            candle.clearAnimation(i);
        }
    }

    public void strobe(Color inputColour)
    {
        StrobeAnimation strobeAnimation = new StrobeAnimation((int) (255 * inputColour.red),
                (int) (255 * inputColour.green), (int) (255 * inputColour.blue), 0, 0.5, ledCount);
        candle.animate(strobeAnimation);
    }

    public void fire()
    {
        FireAnimation fireAnimation = new FireAnimation(0.5, 0.5, ledCount, 0.5, 0.5);
        candle.animate(fireAnimation);
    }

    public Command readyToPlace()
    {
        return new Blinking(this);
    }

    public Command hungry()
    {
        return new Runway(this);
    }

    public Command happy()
    {
        return new PiecePresent(this);
    }

    public Command noAlliance()
    {
        return new NoAllianceLEDs(this);
    }

    public Command redAlliance()
    {
        return new EveryOther(this, Color.kRed);
    }

    public Command blueAlliance()
    {
        return new EveryOther(this, Color.kBlue);
    }

    public Command auto()
    {
        return new AutoLEDs(this);
    }

    public Command endgame()
    {
        return new EndgameLEDs(this);
    }
}
