package frc.robot.subsystems.leds;

import java.util.Optional;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase
{
    private final Notifier periodicNotifier;
    private CANdle candle;
    private CANdleConfiguration config;
    private int ledCount;
    private Optional<Alliance> alliance;
    private boolean isAnimating;

    private class RGB
    {
        int r;
        int b;
        int g;

        RGB(int r, int g, int b)
        {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public static enum GameState
    {
        NONE, AUTO, TELEOP, ENDGAME, HASPIECE, WANTSPIECE, READYPLACE
    };

    private RGB teamColour = new RGB(238, 10, 154);

    private int currentAnimationTick = 0;

    private GameState currenState = GameState.ENDGAME;

    public void resetLEDS()
    {
        candle.setLEDs(0, 0, 0, 0, 0, ledCount);
    }

    public void setLEDState(GameState state)
    {
        currenState = state;
    }

    public LEDS(int CANid, int ledCount)
    {
        candle = new CANdle(CANid);
        config = new CANdleConfiguration();
        this.ledCount = ledCount;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.75;
        candle.configAllSettings(config);
        alliance = DriverStation.getAlliance();
        periodicNotifier = new Notifier(this::periodic);
    }

    public void rainbowAnimation(double brightness, double animationSpeed)
    {
        RainbowAnimation rainbowAnim = new RainbowAnimation(brightness, animationSpeed, ledCount);
        candle.animate(rainbowAnim);
    }

    public void setColour(RGB colour)
    {
        candle.setLEDs(colour.r, colour.g, colour.b, 0, 0, ledCount);
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
        }

        return new int[0];
    }

    public void feedParticalEffect(double brightness)
    {
        candle.setLEDs(0, 0, 0, 0, 0, ledCount);
        int[] particalReturn = particalTranslation(currentAnimationTick);
        for (int i = 0; i < particalReturn.length; i++)
        {
            candle.setLEDs(teamColour.r, teamColour.g, teamColour.b, 0, particalReturn[i] + 8, 1);
        }
        int[] nextParticle0 = particalTranslation((int) MathUtil.inputModulus(currentAnimationTick + 4, 0, 32));
        for (int i = 0; i < nextParticle0.length; i++)
        {
            candle.setLEDs(teamColour.r, teamColour.g, teamColour.b, 0, nextParticle0[i] + 8, 1);
        }
        int[] nextParticle1 = particalTranslation((int) MathUtil.inputModulus(currentAnimationTick + 8, 0, 32));
        for (int i = 0; i < nextParticle1.length; i++)
        {
            candle.setLEDs(teamColour.r, teamColour.g, teamColour.b, 0, nextParticle1[i] + 8, 1);
        }
        int[] nextParticle2 = particalTranslation((int) MathUtil.inputModulus(currentAnimationTick + 12, 0, 32));
        for (int i = 0; i < nextParticle2.length; i++)
        {
            candle.setLEDs(teamColour.r, teamColour.g, teamColour.b, 0, nextParticle2[i] + 8, 1);
        }
    }

    public void hasPiece()
    {
        candle.setLEDs(255, 0, 255, 0, 0, ledCount);
    }

    public void setEveryOtherColour(RGB inputColour1, RGB inputColour2)
    {
        for (int i = 0; i < ledCount; i++)
        {
            if (i % 2 == 0)
            {
                candle.setLEDs(inputColour1.r, inputColour1.g, inputColour1.b, 0, i, 1);
            } else
            {
                candle.setLEDs(inputColour2.r, inputColour2.g, inputColour2.b, 0, i, 1);
            }
        }
    }

    public void everyOther(RGB allianceColour)
    {
        setEveryOtherColour(allianceColour, teamColour);
    }

    public void readyPlace()
    {
        if (currentAnimationTick % 2 == 0)
        {
            candle.setLEDs(teamColour.r, teamColour.g, teamColour.b, 0, 0, ledCount);
        } else
        {
            candle.setLEDs(0, 0, 0, 0, 0, ledCount);
        }
    }

    public void clearAnimationBuffer()
    {
        for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++)
        {
            candle.clearAnimation(i);
        }
        isAnimating = false;
    }

    public void strobe(RGB inputColour)
    {
        StrobeAnimation strobeAnimation = new StrobeAnimation(inputColour.r, inputColour.g, inputColour.b, 0, 0.5,
                ledCount);
        candle.animate(strobeAnimation);
    }

    public RGB determineRGBAlliance(Optional<Alliance> allianceTemp)
    {
        if (allianceTemp.isPresent())
        {
            if (allianceTemp.get() == Alliance.Blue)
            {
                return new RGB(0, 0, 255);
            } else if (allianceTemp.get() == Alliance.Red)
            {
                return new RGB(255, 0, 0);
            } else
            {
                setColour(new RGB(238, 10, 154));
            }
        } else
        {
            setColour(new RGB(238, 10, 154));
        }
        return new RGB(238, 10, 154);
    }

    public void startPeriodic()
    {
        periodicNotifier.startPeriodic(0.02);
    }

    public void stopPeriodic()
    {
        periodicNotifier.stop();
    }

    public void periodic()
    {
        alliance = DriverStation.getAlliance();
        clearAnimationBuffer();
        currentAnimationTick = (int) MathUtil.inputModulus((double) (currentAnimationTick + 1), 0, 32);
        switch (currenState)
        {
        case NONE:
            if (!isAnimating)
            {
                rainbowAnimation(0.5, 0.5);
                isAnimating = true;
            }
            break;
        case HASPIECE:
            clearAnimationBuffer();
            hasPiece();
            break;
        case WANTSPIECE:
            clearAnimationBuffer();
            feedParticalEffect(0.5);
            break;
        case READYPLACE:
            clearAnimationBuffer();
            readyPlace();
            break;
        case AUTO:
            clearAnimationBuffer();
            everyOther(determineRGBAlliance(alliance));
            break;
        case ENDGAME:
            if (!isAnimating)
            {
                resetLEDS();
                strobe(determineRGBAlliance(alliance));
                isAnimating = true;
            }
            break;
        default:
            break;
        }

    }
}
