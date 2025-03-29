package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.manipulator.commands.Intake;
import frc.robot.subsystems.manipulator.commands.IntakeWhileWaiting;
import frc.robot.subsystems.manipulator.commands.Outtake;
import frc.robot.subsystems.manipulator.commands.Purge;

public class Manipulator extends SubsystemBase
{
    private TalonFXS m_motor;
    private DigitalInput m_sensor;

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

    public boolean hasCoral()
    {
        return !m_sensor.get();
    }

    public Command intake()
    {
        return new Intake(this);
    }

    public Command intake(double speed)
    {
        return new Intake(this, speed);
    }

    public Command intakeWhileWaiting()
    {
        return new IntakeWhileWaiting(this);
    }

    public Command intakeWhileWaiting(double speed)
    {
        return new IntakeWhileWaiting(this, speed);
    }

    public Command outtake()
    {
        return new Outtake(this);
    }

    public Command outtake(double speed)
    {
        return new Outtake(this, speed);
    }

    public Command purge()
    {
        return new Purge(this);
    }

    public Command purge(double speed)
    {
        return new Purge(this, speed);
    }
}
