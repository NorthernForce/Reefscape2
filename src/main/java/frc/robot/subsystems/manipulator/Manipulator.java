package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.manipulator.command.Intake;
import frc.robot.subsystems.manipulator.command.Outtake;

@Logged
public class Manipulator extends SubsystemBase
{
    private TalonFXS m_motor;
    private DigitalInput m_sensor;
    private ManipulatorState m_state = ManipulatorState.HUNGRY;
    private boolean m_canIntake = true;
    private Timer m_timer = new Timer();

    public Manipulator()
    {
        m_motor = new TalonFXS(RobotConstants.ManipulatorConstants.kMotorId);
        m_sensor = new DigitalInput(RobotConstants.ManipulatorConstants.kSensorId);
    }

    public void set(double speed)
    {
        m_motor.set(speed);
    }

    public void stop()
    {
        m_motor.set(0);
    }

    public boolean hasCoralInSensor()
    {
        return !m_sensor.get();
    }

    public boolean hasCoral()
    {
        return m_state == ManipulatorState.HAPPY;
    }

    public void setCanIntake(boolean canIntake)
    {
        m_canIntake = canIntake;
    }

    public Command intake()
    {
        return new Intake(this);
    }

    public Command outtake()
    {
        return new Outtake(this);
    }

    public Command slowOuttake()
    {
        return runOnce(() ->
        {
            m_state = ManipulatorState.SLOW_OUTTAKING;
        }).alongWith(Commands.waitUntil(() -> !hasCoralInSensor()));
    }

    public ManipulatorState getState()
    {
        return m_state;
    }

    public void setState(ManipulatorState state)
    {
        m_state = state;
        if (state == ManipulatorState.BRUTE_OUTTAKING)
        {
            m_timer.restart();
        }
    }

    @Override
    public void periodic()
    {
        switch (m_state)
        {
        case HUNGRY:
            set(m_canIntake ? RobotConstants.ManipulatorConstants.kIntakeSpeed : 0);
            if (hasCoralInSensor())
            {
                m_state = ManipulatorState.PURGING;
            }
            break;
        case PURGING:
            set(-RobotConstants.ManipulatorConstants.kPurgeSpeed);
            if (!hasCoralInSensor())
            {
                m_state = ManipulatorState.REINTAKING;
                m_timer.restart();
            }
            break;
        case REINTAKING:
            set(0.2);
            if (m_timer.hasElapsed(0.5))
            {
                m_state = ManipulatorState.HUNGRY;
            }
            if (hasCoralInSensor())
            {
                m_state = ManipulatorState.HAPPY;
            }
            break;
        case HAPPY:
            stop();
            break;
        case OUTTAKING:
            set(RobotConstants.ManipulatorConstants.kOuttakeSpeed);
            if (!hasCoralInSensor())
            {
                m_state = ManipulatorState.HUNGRY;
            }
            break;
        case SLOW_OUTTAKING:
            set(RobotConstants.ManipulatorConstants.kSlowOuttakeSpeed);
            if (!hasCoralInSensor())
            {
                m_state = ManipulatorState.HUNGRY;
            }
            break;
        case BRUTE_OUTTAKING:
            set(RobotConstants.ManipulatorConstants.kOuttakeSpeed);
            if (m_timer.hasElapsed(1.0))
            {
                m_state = ManipulatorState.HUNGRY;
            }
        }
    }

    public enum ManipulatorState
    {
        HUNGRY, PURGING, REINTAKING, OUTTAKING, HAPPY, SLOW_OUTTAKING, BRUTE_OUTTAKING
    }
}
