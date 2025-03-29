package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private TalonFXS m_motor;
    private DigitalInput m_sensor;

    public Manipulator(int motorId, int sensorId)
    {
        m_motor = new TalonFXS(motorId);
        m_sensor = new DigitalInput(sensorId);
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
