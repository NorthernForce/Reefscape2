package frc.robot.subsystems.algae_extractor;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algae_extractor.commands.RemoveAlgae;
import frc.robot.subsystems.algae_extractor.commands.ReturnArm;

@Logged
public class AlgaeExtractor extends SubsystemBase
{
    private final TalonFXS m_motor;
    private final double extractSpeed;
    private final double returnSpeed;
    private final DigitalInput m_limitSwitch;

    public AlgaeExtractor(int id, int sensorId, boolean inverted, double gearRatio, double extractSpeed,
            double returnSpeed)
    {
        m_motor = new TalonFXS(id);
        this.extractSpeed = extractSpeed;
        this.returnSpeed = returnSpeed;
        m_limitSwitch = new DigitalInput(sensorId);
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.ExternalFeedback.SensorToMechanismRatio = gearRatio;
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        config.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motor.getConfigurator().apply(config);
    }

    public void returnExtractor()
    {
        m_motor.set(-returnSpeed);
    }

    public void stop()
    {
        m_motor.stopMotor();
    }

    public void extractAlgae()
    {
        m_motor.set(extractSpeed);
    }

    public boolean hasReachedTop()
    {
        return !m_limitSwitch.get();
    }

    public Command getExtractCommand()
    {
        return new RemoveAlgae(this);
    }

    public Command getReturnCommand()
    {
        return new ReturnArm(this);
    }
}