package frc.robot.util.logging;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.CtreUtil;

/**
 * Every Talon FX's supply current and supply voltage, logged once per loop for the power dashboard
 * in {@code tools/power_analysis}.
 *
 * <p>IO layers hand over their motors with {@link #register} from their constructors, and
 * {@link RobotContainerLogger} calls {@link #log} on each Epilogue update. Nothing else reads
 * these signals.
 *
 * <p>Entries are named {@code TalonFX-<CAN ID>/SupplyCurrent} and {@code .../SupplyVoltage}, which
 * is what the dashboard looks for.
 */
public final class PowerLogging {
    private PowerLogging() {}

    /** One CAN bus's signals. refreshAll rejects signals from more than one bus. */
    private static final class Bus {
        final List<BaseStatusSignal> signals = new ArrayList<>();
        final List<String> names = new ArrayList<>();
    }

    private static final Map<String, Bus> s_buses = new LinkedHashMap<>();
    private static final List<Bus> s_busList = new ArrayList<>();

    /**
     * Publishes each motor's supply current and supply voltage at
     * {@link CtreUtil#kCurrentSignalFrequencyHz} and adds them to the power log. Call once from an
     * IO layer's constructor with every motor it owns, before any {@code optimizeBusUtilization}
     * call - without an explicit rate the optimizer would drop both signals to 4 Hz.
     */
    public static void register(TalonFX... motors) {
        BaseStatusSignal[] all = new BaseStatusSignal[motors.length * 2];
        for (int i = 0; i < motors.length; i++) {
            TalonFX motor = motors[i];
            BaseStatusSignal current = motor.getSupplyCurrent(false);
            BaseStatusSignal voltage = motor.getSupplyVoltage(false);
            all[2 * i] = current;
            all[2 * i + 1] = voltage;

            Bus bus = s_buses.computeIfAbsent(motor.getNetwork().getName(), name -> {
                Bus created = new Bus();
                s_busList.add(created);
                return created;
            });
            String prefix = "TalonFX-" + motor.getDeviceID() + "/";
            bus.signals.add(current);
            bus.names.add(prefix + "SupplyCurrent");
            bus.signals.add(voltage);
            bus.names.add(prefix + "SupplyVoltage");
        }
        CtreUtil.reportIfNotOk(
                "set power signal update frequency",
                BaseStatusSignal.setUpdateFrequencyForAll(CtreUtil.kCurrentSignalFrequencyHz, all));
    }

    /**
     * Writes the latest value of every registered signal. refreshAll only copies what Phoenix has
     * already received, so this adds no CAN traffic. A signal slower than the loop repeats its
     * last value, which the lazy backend drops.
     */
    public static void log(EpilogueBackend backend) {
        for (int b = 0; b < s_busList.size(); b++) {
            Bus bus = s_busList.get(b);
            BaseStatusSignal.refreshAll(bus.signals);
            for (int i = 0; i < bus.signals.size(); i++) {
                backend.log(bus.names.get(i), bus.signals.get(i).getValueAsDouble());
            }
        }
        backend.log("BrownedOut", RobotController.isBrownedOut());
    }
}
