package frc.robot;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class NetworkTablesReceiver {
    private final DoubleSubscriber forward, strafe, rotation, heartbeat;
    private final BooleanSubscriber stopSub;
    private double lastHb = 0.0, lastHbTime = 0.0;

    public NetworkTablesReceiver() {
        NetworkTable t = NetworkTableInstance.getDefault().getTable("SummonDesk");
        forward  = t.getDoubleTopic("Movement/Forward").subscribe(0.0);
        strafe   = t.getDoubleTopic("Movement/Strafe").subscribe(0.0);
        rotation = t.getDoubleTopic("Movement/Rotation").subscribe(0.0);
        heartbeat= t.getDoubleTopic("Pi/Heartbeat").subscribe(0.0);
        stopSub  = t.getBooleanTopic("Safety/Stop").subscribe(true);
    }

    public void update() {
        double hb = heartbeat.get();
        if (hb != lastHb) { lastHb = hb; lastHbTime = Timer.getFPGATimestamp(); }
    }

    public boolean isStale() {
        return (Timer.getFPGATimestamp() - lastHbTime) > Constants.NT_STALE_TIMEOUT_SEC;
    }
    public boolean isStop() { return stopSub.get(); }
    public double getForward()  { return forward.get(); }
    public double getStrafe()   { return strafe.get(); }
    public double getRotation() { return rotation.get(); }
}
