package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase
{
    DoubleSubscriber xOffsetSubscriber;
    DoubleArraySubscriber postsSubscriber;
    double xOffset;
    double[] posts;
    boolean connected;
    String deviceName;

    public Vision(String deviceName, String tableName)
    {
        var table = NetworkTableInstance.getDefault().getTable(tableName);
        this.postsSubscriber = table.getDoubleArrayTopic("Posts").subscribe(new double[]
        {});
        this.xOffsetSubscriber = table.getDoubleTopic("CandidateMetersX").subscribe(Double.NaN);
        this.deviceName = deviceName;
    }

    @Override
    public void periodic()
    {
        xOffset = xOffsetSubscriber.get();
        posts = postsSubscriber.get();
        connected = false;
        for (var connection : NetworkTableInstance.getDefault().getConnections())
        {
            if (connection.remote_id.startsWith(deviceName))
            {
                connected = true;
                break;
            }
        }
    }

    public Optional<Double> getXOffset()
    {
        return Double.isNaN(xOffset) ? Optional.empty() : Optional.of(xOffset);
    }

    @Logged(name = "XOffset")
    public double getRawXOffset()
    {
        return xOffset;
    }

    @Logged(name = "RawPosts")
    public double[] getRawPosts()
    {
        return posts;
    }

    @Logged(name = "Connected")
    public boolean connected()
    {
        return connected;
    }

}
