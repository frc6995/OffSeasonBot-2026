package frc.robot.subsystems.power;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Logs the robot-wide half of the power budget: battery voltage, brownout state, roboRIO input
 * current, and CAN bus utilization on both CANivores.
 *
 * <p>This exists to feed {@code tools/power_analysis}, which reads a match's .wpilog and produces
 * a per-subsystem energy/percentile breakdown plus a per-brownout-event attribution. Per-subsystem
 * draw comes from the {@code Supply Current Total} getters in each subsystem, and the robot total
 * is their sum. What this class adds is the voltage those currents are measured against, and the
 * brownout flag.
 *
 * <p>There is deliberately no power distribution board here. The PDH is not on the CAN bus, and a
 * {@code PowerDistribution} with nothing answering at its ID does not fail cleanly - it constructs
 * fine and then reports a CAN error on every read, every loop. Everything below comes from the
 * roboRIO itself or the CANivores, so none of it depends on the PDH being wired. The cost is that
 * loads with no motor controller behind them (roboRIO, radio, Limelights) are not in the total.
 *
 * <p>Values are sampled once in {@link #periodic()} and cached, matching the IO-layer pattern used
 * elsewhere in this project. Epilogue calls the getters below after the command scheduler has run,
 * so they read this loop's values.
 *
 * <p>Bus utilization is logged because the per-motor supply current signals are a large share of
 * CAN traffic (see {@link frc.robot.util.CtreUtil#kCurrentSignalFrequencyHz}) - without it there
 * would be no way to tell whether that traffic was affordable.
 */
public class PowerMonitor extends SubsystemBase {

    private double batteryVoltage;
    private boolean brownedOut;
    private double brownoutVoltage;
    private double rioInputCurrentAmps;

    private double lowerBusUtilization;
    private double upperBusUtilization;

    @Override
    public void periodic() {
        batteryVoltage = RobotController.getBatteryVoltage();
        brownedOut = RobotController.isBrownedOut();
        brownoutVoltage = RobotController.getBrownoutVoltage();
        rioInputCurrentAmps = RobotController.getInputCurrent();

        lowerBusUtilization = busUtilization(Constants.CANBuses.LowerBus);
        upperBusUtilization = busUtilization(Constants.CANBuses.UpperBus);
    }

    private static double busUtilization(CANBus bus) {
        try {
            return bus.getStatus().BusUtilization;
        } catch (RuntimeException e) {
            return 0.0;
        }
    }

    /**
     * Battery voltage as the roboRIO's power input sees it. This is what the brownout detector
     * compares against.
     */
    @Logged(name = "Battery Voltage", importance = Importance.CRITICAL)
    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    /** True while the roboRIO is actively browning out (6V rail disabled, outputs cut). */
    @Logged(name = "Browned Out", importance = Importance.CRITICAL)
    public boolean isBrownedOut() {
        return brownedOut;
    }

    /**
     * The threshold {@link #isBrownedOut()} trips at. Logged so the analyzer need not assume it.
     * CRITICAL because it costs one log entry per match (the lazy backend only writes changes).
     */
    @Logged(name = "Brownout Voltage", importance = Importance.CRITICAL)
    public double getBrownoutVoltage() {
        return brownoutVoltage;
    }

    /** The roboRIO's own draw. Not part of any subsystem's supply current. */
    @Logged(name = "RIO Input Current", importance = Importance.DEBUG)
    public double getRioInputCurrentAmps() {
        return rioInputCurrentAmps;
    }

    /** Fraction of the swerve CANivore's bandwidth in use, 0-1. */
    @Logged(name = "CAN/LowerBus Utilization", importance = Importance.DEBUG)
    public double getLowerBusUtilization() {
        return lowerBusUtilization;
    }

    /** Fraction of the superstructure CANivore's bandwidth in use, 0-1. */
    @Logged(name = "CAN/UpperBus Utilization", importance = Importance.DEBUG)
    public double getUpperBusUtilization() {
        return upperBusUtilization;
    }
}
