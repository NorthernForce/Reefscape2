package frc.robot.subsystems.leds;

import java.util.Optional;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase
{
    private CANdle candle;
    private CANdleConfiguration config;
    private int ledCount;
    private Optional<Alliance> alliance;

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

    enum GameState
    {
        NONE, AUTO, TELEOP, ENDGAME, HASPIECE, WANTSPIECE
    };

    private RGB teamColour = new RGB(238, 10, 154);

    private int currentAnimationTick = 0;

    private GameState currenState = GameState.WANTSPIECE;

    public LEDS(int CANid, int ledCount)
    {
        candle = new CANdle(CANid);
        config = new CANdleConfiguration();
        this.ledCount = ledCount;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.75;
        candle.configAllSettings(config);
        alliance = DriverStation.getAlliance();
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

    public int belt(int input, int number)
    {
        if (input < 0)
        {
            belt(number + input, number);
        } else if (input > number)
        {
            belt(input - number, number);
        } else
        {
            return input;
        }
        return 0;
    }

    public int[] particalTranslation(int tick)
    {
        if (tick == 0)
        {
            int[] returnArray =
            { 6 };
            return returnArray;
        } else if (tick > 0 && tick < 32)
        {
            int[] returnArray =
            { belt(6 - tick, 63), belt(6 + tick, 63) };
            return returnArray;
        } else if (tick == 32)
        {
            int[] returnArray =
            { 32 };
        }

        return new int[0];
    }

    public void feedParticalEffect(double brightness)
    {
        candle.setLEDs(0, 0, 0, 0, 0, ledCount);
        int[] particalReturn = particalTranslation(currentAnimationTick);
        for (int i = 0; i < particalReturn.length; i++)
        {
            candle.setLEDs(teamColour.r, teamColour.g, teamColour.b, 0, particalReturn[i], 1);
        }
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

    public void everyOther(Optional<Alliance> allianceTemp)
    {
        if (allianceTemp.isPresent())
        {
            if (allianceTemp.get() == Alliance.Blue)
            {
                setEveryOtherColour(new RGB(0, 0, 255), new RGB(238, 10, 154));
            } else if (allianceTemp.get() == Alliance.Red)
            {
                setEveryOtherColour(new RGB(255, 0, 0), new RGB(238, 10, 154));
            } else
            {
                setColour(new RGB(238, 10, 154));
            }
        } else
        {
            setColour(new RGB(238, 10, 154));
        }
    }

    public void clearAnimationBuffer()
    {
        for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++)
        {
            candle.clearAnimation(i);
        }
    }

    @Override
    public void periodic()
    {
        alliance = DriverStation.getAlliance();
        clearAnimationBuffer();
        currentAnimationTick = belt(currentAnimationTick + 1, 32);
        switch (currenState)
        {
        case NONE:
            rainbowAnimation(0.5, 0.5);
            break;
        case AUTO:
            everyOther(DriverStation.getAlliance());
            break;
        case HASPIECE:
            setColour(new RGB(238, 10, 154));
            break;
        case WANTSPIECE:
            feedParticalEffect(0.5);
            break;
        default:
            break;
        }

    }
}
