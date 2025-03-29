package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.commands.HomeElevator;
import frc.robot.subsystems.superstructure.elevator.commands.MoveElevatorToPosition;
import frc.robot.util.PhoenixUtil;

@Logged
public class Elevator extends SubsystemBase
{
    private final TalonFX motor;
    private final ElevatorConfig constants;
    private final MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
    private final StatusSignal<Angle> position;
    private final DigitalInput lowerLimitSwitch;
    private final ElevatorSim sim;
    private final TalonFXSimState simState;
    private boolean hasResetLowerLimit = false;

    public static record ElevatorConfig(double kS, double kV, double kA, double kP, double kI, double kD, double kG,
            double cruiseVelocity, double acceleration, double jerk, Distance sprocketCircumference, double gearRatio,
            boolean inverted, Distance lowerLimit, Distance upperLimit, Mass mass) {
    }

    public Elevator(int canID, int limitSwitchPin, ElevatorConfig constants)
    {
        motor = new TalonFX(canID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = constants.kS;
        config.Slot0.kV = constants.kV;
        config.Slot0.kA = constants.kA;
        config.Slot0.kP = constants.kP;
        config.Slot0.kI = constants.kI;
        config.Slot0.kD = constants.kD;
        config.Slot0.kG = constants.kG;
        config.MotionMagic.MotionMagicCruiseVelocity = constants.cruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = constants.acceleration;
        config.MotionMagic.MotionMagicJerk = constants.jerk;
        config.MotorOutput.Inverted = constants.inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.RotorToSensorRatio = 1;
        config.Feedback.SensorToMechanismRatio = constants.gearRatio / constants.sprocketCircumference.in(Inches);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimit.in(Inches);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimit.in(Inches);
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
        this.constants = constants;
        position = motor.getPosition();
        lowerLimitSwitch = new DigitalInput(limitSwitchPin);
        if (RobotBase.isSimulation())
        {
            sim = new ElevatorSim(DCMotor.getKrakenX60Foc(1), constants.gearRatio, constants.mass.in(Kilograms),
                    constants.sprocketCircumference.in(Meters) / (2 * Math.PI), constants.lowerLimit.in(Meters),
                    constants.upperLimit.in(Meters), true, constants.lowerLimit.in(Meters));
            simState = motor.getSimState();
        } else
        {
            sim = null;
            simState = null;
        }
    }

    public void setTargetHeight(Distance height)
    {
        motor.setControl(motionMagicExpoVoltage.withPosition(height.in(Inches)));
    }

    public Distance getHeight()
    {
        return Inches.of(position.getValue().in(Rotations));
    }

    public void set(double speed)
    {
        motor.set(speed + (getHeight().isNear(constants.lowerLimit, Inches.of(0.5)) ? 0 : constants.kG / 12.0));
    }

    public void stop()
    {
        motor.stopMotor();
    }

    public void resetPosition()
    {
        motor.setPosition(0);
    }

    public boolean isAtBottomLimit()
    {
        return !lowerLimitSwitch.get();
    }

    public boolean isPresent()
    {
        return motor.isConnected();
    }

    public void ignoreBottomLimit()
    {
        SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
        motor.getConfigurator().refresh(config);
        config.ReverseSoftLimitEnable = false;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    }

    public void useBottomLimit()
    {
        SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
        motor.getConfigurator().refresh(config);
        config.ReverseSoftLimitEnable = true;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    }

    public Command home()
    {
        return new HomeElevator(this);
    }

    public Command moveToPosition(Distance position)
    {
        return new MoveElevatorToPosition(this, position);
    }

    public Command controlByJoystick(DoubleSupplier input)
    {
        return run(() -> set(input.getAsDouble()));
    }

    @Override
    public void periodic()
    {
        BaseStatusSignal.refreshAll(position);
        if (isAtBottomLimit())
        {
            if (!hasResetLowerLimit)
            {
                resetPosition();
                hasResetLowerLimit = true;
            }
        } else
        {
            hasResetLowerLimit = false;
        }
    }

    @Override
    public void simulationPeriodic()
    {
        sim.setInput(simState.getMotorVoltage());
        sim.update(0.02);
        var velocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());
        simState.setRotorVelocity(
                velocity.in(InchesPerSecond) * constants.gearRatio / constants.sprocketCircumference.in(Inches));
        var dx = velocity.times(Seconds.of(0.02));
        simState.addRotorPosition(dx.in(Inches) * constants.gearRatio / constants.sprocketCircumference.in(Inches));
    }
}
