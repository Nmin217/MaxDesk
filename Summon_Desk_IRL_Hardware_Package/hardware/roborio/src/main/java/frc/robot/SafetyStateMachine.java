package frc.robot;

public class SafetyStateMachine {
    private boolean shouldStop = true;

    public void update(NetworkTablesReceiver nt) {
        shouldStop = nt.isStop() || nt.isStale();
    }
    public boolean shouldStop() { return shouldStop; }
}
