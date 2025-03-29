package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

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
}
