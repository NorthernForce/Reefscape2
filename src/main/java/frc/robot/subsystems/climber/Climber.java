package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

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
    private final int m_encoderId;
    private final CANcoder m_encoder;

    public Climber(int motorId, int encoderId, double climbSpeed)
    {
        m_motorId = motorId;
        m_motor = new TalonFX(m_motorId);
        m_encoder = new CANcoder(encoderId);

        this.m_climbSpeed = climbSpeed;
        this.m_encoderId = encoderId;
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
