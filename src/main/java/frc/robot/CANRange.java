package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

@Logged
public class CANRange {

    public class CANRangeConstants {
        public static final int kCAN_ID = 35;
        public static final double kProximityThreshold = Units.inchesToMeters(3.0);
    }

    // Null when the sensor isn't on the robot (see Constants.Hardware.kCANRangeInstalled).
    // Constructing a CANrange that isn't on the bus would make every isCloseToWall() call
    // refresh a signal that can never arrive, and Phoenix reports an error for each one.
    private final CANrange m_frontCANrange =
            Constants.Hardware.kCANRangeInstalled
                    ? new CANrange(CANRangeConstants.kCAN_ID, Constants.CANBuses.UpperBus)
                    : null;

    CANrangeConfiguration m_frontCANrangeConfigurator = new CANrangeConfiguration();

    // In simulation the physical sensor is never actually "detected," so it defaults to false here
    // and is instead driven by a driver-station button (see setSimProximitySupplier) so BLine
    // auto transitions can be exercised in sim. Ignored entirely on real hardware.
    private BooleanSupplier m_simProximitySupplier = () -> false;

    public CANRange() {
        m_frontCANrangeConfigurator.ProximityParams.ProximityThreshold = CANRangeConstants.kProximityThreshold;
    }

    public void setSimProximitySupplier(BooleanSupplier simProximitySupplier) {
        m_simProximitySupplier = simProximitySupplier;
    }

    public Boolean isCloseToWall() {
        if (RobotBase.isSimulation()) {
            return m_simProximitySupplier.getAsBoolean();
        }
        if (m_frontCANrange == null) {
            // Sensor not installed: report "never detected" so BLine Depot auto's wall-proximity
            // transition simply never fires, rather than reading a device that isn't there.
            return false;
        }
        return m_frontCANrange.getIsDetected().getValue();
    }
}
