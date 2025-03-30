package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX>
{
    public TalonFXLogger()
    {
        super(TalonFX.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFX object)
    {
        backend.log("is_present", object.isConnected());
    }
}
