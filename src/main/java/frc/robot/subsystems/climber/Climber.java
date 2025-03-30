package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.commands.ExtendClimber;
import frc.robot.subsystems.climber.commands.RetractClimber;

@Logged
public class Climber extends SubsystemBase
{
    private final int m_motorId;
    private final TalonFX m_motor;
    private final double m_climbSpeed;

    public Climber(int motorId, double climbSpeed, boolean inverted)
    {
        m_motorId = motorId;
        m_motor = new TalonFX(m_motorId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = (inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motor.getConfigurator().apply(config);
        this.m_climbSpeed = climbSpeed;
    }

    public void extend()
    {
        m_motor.set(m_climbSpeed);
    }

    public void retract()
    {
        m_motor.set(-m_climbSpeed);
    }

    public void stop()
    {
        m_motor.stopMotor();
    }

    public Command getExtendCommand()
    {
        return new ExtendClimber(this);
    }

    public Command getRetractCommand()
    {
        return new RetractClimber(this);
    }
}
