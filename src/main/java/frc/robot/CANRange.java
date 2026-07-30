package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;

@Logged
public class CANRange {

    public class CANRangeConstants {
        public static final int kCAN_ID = 35;
        public static final double kProximityThreshold = Units.inchesToMeters(3.0);
    }

    CANrange m_frontCANrange = new CANrange(CANRangeConstants.kCAN_ID, Constants.CANBuses.UpperBus);

    CANrangeConfiguration m_frontCANrangeConfigurator = new CANrangeConfiguration();

    public CANRange() {
        m_frontCANrangeConfigurator.ProximityParams.ProximityThreshold = CANRangeConstants.kProximityThreshold;
    }

    public Boolean isCloseToWall() {
        return m_frontCANrange.getIsDetected().getValue();
    }
}
