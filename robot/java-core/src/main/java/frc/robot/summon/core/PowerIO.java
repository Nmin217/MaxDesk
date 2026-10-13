package frc.robot.summon.core;

/**
 * Charging-station and battery state. On the real robot the UPS numbers come from Network UPS Tools on the
 * coprocessor (published over NetworkTables); the 12 V battery comes from the PDH; the cord state should come from
 * THREE independent sources (mains-presence sensor on the frame inlet, the UPS "on line" flag, and the breakaway
 * connector's auxiliary contact). Any one of them saying "plugged in" must stop the robot from driving.
 */
public interface PowerIO {
    boolean mainsPresentAtInlet();      // dedicated AC-presence sensor
    boolean upsOnLine();                // UPS status "OL" (wall power present)
    boolean connectorMated();           // breakaway connector auxiliary contact
    double robotBatteryPercent();       // 12 V SLA, estimated from voltage under load
    double upsPercent();                // from the UPS over USB

    /** Interlock: refuse to drive if ANY source says the cord may still be connected. */
    default boolean cordPluggedIn() { return mainsPresentAtInlet() || upsOnLine() || connectorMated(); }
}
