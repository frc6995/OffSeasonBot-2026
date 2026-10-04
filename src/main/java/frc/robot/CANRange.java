package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.CtreUtil;

@Logged
public class CANRange {

    public class CANRangeConstants {
        public static final int kCAN_ID = 35;
        public static final double kProximityThreshold = 0.1;
    }

    CANrange m_frontCANrange = new CANrange(CANRangeConstants.kCAN_ID, Constants.CANBuses.LowerBus);

    CANrangeConfiguration m_frontCANrangeConfigurator = new CANrangeConfiguration();

    private BooleanSupplier m_simProximitySupplier = () -> false;

    public CANRange() {
        m_frontCANrangeConfigurator.ProximityParams.ProximityThreshold = CANRangeConstants.kProximityThreshold;

        CtreUtil.reportIfNotOk("Config front CANrange",
                m_frontCANrange.getConfigurator().apply(m_frontCANrangeConfigurator));
    }

    public void setSimProximitySupplier(BooleanSupplier simProximitySupplier) {
        m_simProximitySupplier = simProximitySupplier;
    }

    public void periodic() {
        // System.out.println(m_frontCANrange.getIsDetected());
    }

    public Boolean isCloseToWall() {
        if (RobotBase.isSimulation()) {
            return m_simProximitySupplier.getAsBoolean();
        }
       return m_frontCANrange.getIsDetected().getValue();
    }
}
