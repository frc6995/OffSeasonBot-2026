package frc.robot.subsystems.power;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Logs everything needed to reconstruct the robot's power budget offline: battery voltage,
 * brownout state, PDP per-channel and total current, and CAN bus utilization on both CANivores.
 *
 * <p>This exists to feed {@code tools/power_analysis}, which reads a match's .wpilog and produces
 * a per-subsystem energy/percentile breakdown plus a per-brownout-event attribution. Per-subsystem
 * draw comes from the named motor signals in each subsystem's IO layer; what this class adds is
 * the <i>ground truth</i> those are checked against - the PDP sees every load on the robot,
 * including the ones with no motor controller behind them (roboRIO, radio, Limelights). A large
 * gap between the sum of the named subsystem currents and {@link #getPdpTotalCurrentAmps()} means
 * something is drawing power that nothing in code is accounting for.
 *
 * <p>Values are sampled once in {@link #periodic()} and cached, matching the IO-layer pattern used
 * elsewhere in this project. Epilogue calls the getters below after the command scheduler has run,
 * so they read this loop's values; making them field reads rather than JNI calls also means the
 * per-channel array is fetched once per loop no matter how many getters touch it.
 *
 * <p>Bus utilization is logged because this class ships alongside a large increase in CAN traffic
 * (see {@link frc.robot.util.CtreUtil#kCurrentSignalFrequencyHz}) - without it there would be no
 * way to tell whether that increase was affordable.
 */
public class PowerMonitor extends SubsystemBase {

    /** CAN ID of the CTRE PDP. 0 is the CTRE default and is not configurable on the PDP itself. */
    public static final int kPdpCanId = 0;

    /** Channel count of a CTRE PDP. A REV PDH would be 24. */
    public static final int kChannelCount = 16;

    // PDP channel -> load mapping is a wiring fact, not a code fact, so it is not encoded here.
    // The per-subsystem breakdown in tools/power_analysis comes from the named motor signals in
    // each subsystem's IO layer and does not need this map. It is only needed to interpret
    // getPdpChannelCurrentsAmps() directly - record it here as a comment once someone reads it
    // off the robot.

    /**
     * Null if the PDP could not be constructed. Every getter below tolerates that: losing power
     * logging is not a reason to take the robot down, and simulation has no PDP at all.
     */
    private final PowerDistribution m_pdp;

    // Cached once per loop by periodic(). Never null, never resized - Epilogue is configured with
    // ErrorHandler.crashOnError() in simulation, so a null array here would crash sim outright.
    private double[] pdpChannelCurrentsAmps = new double[kChannelCount];
    private double pdpVoltage;
    private double pdpTotalCurrentAmps;
    private double pdpTotalPowerWatts;
    private double pdpTotalEnergyJoules;
    private double pdpTemperatureCelsius;

    private double batteryVoltage;
    private boolean brownedOut;
    private double brownoutVoltage;
    private double rioInputCurrentAmps;

    private double lowerBusUtilization;
    private double upperBusUtilization;

    public PowerMonitor() {
        m_pdp = createPdp();
    }

    private static PowerDistribution createPdp() {
        try {
            return new PowerDistribution(kPdpCanId, ModuleType.kCTRE);
        } catch (RuntimeException e) {
            DriverStation.reportWarning(
                    "PowerMonitor: could not open the CTRE PDP at CAN ID " + kPdpCanId
                            + ", power logging will be incomplete: " + e,
                    e.getStackTrace());
            return null;
        }
    }

    @Override
    public void periodic() {
        batteryVoltage = RobotController.getBatteryVoltage();
        brownedOut = RobotController.isBrownedOut();
        brownoutVoltage = RobotController.getBrownoutVoltage();
        rioInputCurrentAmps = RobotController.getInputCurrent();

        lowerBusUtilization = busUtilization(Constants.CANBuses.LowerBus);
        upperBusUtilization = busUtilization(Constants.CANBuses.UpperBus);

        if (m_pdp == null) {
            return;
        }

        // One JNI call for every channel rather than kChannelCount separate getCurrent(i) calls.
        double[] currents = m_pdp.getAllCurrents();
        if (currents != null && currents.length == pdpChannelCurrentsAmps.length) {
            // Copy rather than reassign: the logged array is handed straight to Epilogue, and
            // keeping one instance avoids allocating a fresh array every loop.
            System.arraycopy(currents, 0, pdpChannelCurrentsAmps, 0, currents.length);
        } else if (currents != null) {
            // A PDH (24 channels) or a future module type would land here. Take what we can.
            pdpChannelCurrentsAmps = currents;
        }

        pdpVoltage = m_pdp.getVoltage();
        pdpTotalCurrentAmps = m_pdp.getTotalCurrent();
        pdpTotalPowerWatts = m_pdp.getTotalPower();
        pdpTotalEnergyJoules = m_pdp.getTotalEnergy();
        pdpTemperatureCelsius = m_pdp.getTemperature();
    }

    private static double busUtilization(CANBus bus) {
        try {
            return bus.getStatus().BusUtilization;
        } catch (RuntimeException e) {
            return 0.0;
        }
    }

    /**
     * Battery voltage as the roboRIO's power distribution sees it. This, not the PDP's own voltage
     * reading, is what the brownout detector compares against.
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

    /** The threshold {@link #isBrownedOut()} trips at. Logged so the analyzer need not assume it. */
    @Logged(name = "Brownout Voltage", importance = Importance.DEBUG)
    public double getBrownoutVoltage() {
        return brownoutVoltage;
    }

    @Logged(name = "RIO Input Current", importance = Importance.DEBUG)
    public double getRioInputCurrentAmps() {
        return rioInputCurrentAmps;
    }

    /** True if a PDP was found on the bus. If false, every PDP getter below reads zero. */
    @Logged(name = "PDP/Connected", importance = Importance.CRITICAL)
    public boolean isPdpConnected() {
        return m_pdp != null;
    }

    /**
     * Per-channel current, indexed by PDP channel. Logged as a single array topic rather than
     * {@value #kChannelCount} scalar topics.
     */
    @Logged(name = "PDP/Channel Currents", importance = Importance.CRITICAL)
    public double[] getPdpChannelCurrentsAmps() {
        return pdpChannelCurrentsAmps;
    }

    /** Total current through the PDP: the ground truth every per-subsystem sum is checked against. */
    @Logged(name = "PDP/Total Current", importance = Importance.CRITICAL)
    public double getPdpTotalCurrentAmps() {
        return pdpTotalCurrentAmps;
    }

    @Logged(name = "PDP/Voltage", importance = Importance.DEBUG)
    public double getPdpVoltage() {
        return pdpVoltage;
    }

    @Logged(name = "PDP/Total Power", importance = Importance.DEBUG)
    public double getPdpTotalPowerWatts() {
        return pdpTotalPowerWatts;
    }

    @Logged(name = "PDP/Total Energy", importance = Importance.DEBUG)
    public double getPdpTotalEnergyJoules() {
        return pdpTotalEnergyJoules;
    }

    @Logged(name = "PDP/Temperature", importance = Importance.DEBUG)
    public double getPdpTemperatureCelsius() {
        return pdpTemperatureCelsius;
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
