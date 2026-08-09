package frc.robot.util;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Wraps the {@code TorqueCurrent} peak-current config for one or more TalonFX
 * motors so it can be adjusted at runtime (e.g. by a subsystem's own state
 * machine, or by a cross-subsystem coordinator like
 * {@link CurrentLimitCoordinator}) without spamming a CAN config frame every
 * periodic() tick.
 *
 * <p>Only meaningful for motors running a TorqueCurrentFOC-family control
 * request (e.g. {@code VelocityTorqueCurrentFOC}, {@code
 * MotionMagicTorqueCurrentFOC}) - the peak values configured here cap how
 * much torque current that closed loop is allowed to command.
 *
 * <p>Pass every motor that should share the same cap (e.g. a lead motor and
 * any followers that also run their own local TorqueCurrent config) to the
 * constructor; {@link #setPeakTorqueCurrentAmps(double)} applies to all of
 * them together and is only actually sent over CAN when the requested value
 * changes.
 */
public class TorqueCurrentLimiter {
    private final TalonFX[] motors;
    private final TorqueCurrentConfigs config = new TorqueCurrentConfigs();
    private double appliedPeakAmps = Double.NaN;

    public TorqueCurrentLimiter(TalonFX... motors) {
        this.motors = motors;
    }

    /**
     * Caps closed-loop torque current output to +/-peakAmps. No-ops (and sends no
     * CAN frame) if this is the same limit already applied.
     */
    public void setPeakTorqueCurrentAmps(double peakAmps) {
        if (Math.abs(peakAmps - appliedPeakAmps) < 1e-3) {
            return;
        }
        appliedPeakAmps = peakAmps;
        // PeakReverseTorqueCurrent must stay negative because CTRE clamps it to 0 if it's
        // positive, which kills all reverse-direction torque.
        config.withPeakForwardTorqueCurrent(peakAmps).withPeakReverseTorqueCurrent(-peakAmps);
        for (TalonFX motor : motors) {
            CtreUtil.reportIfNotOk(
                    "set peak torque current on device " + motor.getDeviceID(),
                    motor.getConfigurator().apply(config));
        }
    }
}
