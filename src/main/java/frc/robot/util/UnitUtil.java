package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;

public class UnitUtil {
    public static Angle CW_180 = Degrees.of(-180);
    public static Angle CCW_180 = Degrees.of(180);

    public static <U extends Unit, M extends Measure<U>> M max(M a, M b) {
        return a.gte(b) ? a : b;
    }
    public static <U extends Unit, M extends Measure<U>> M min(M a, M b) {
        return a.lte(b) ? a : b;
    }
    public static <U extends Unit, M extends Measure<U>> M clamp(M value, M low, M high) {
        return max(low, min(value, high));
    }
    public static BooleanSupplier isWithinTolerance(Supplier<Angle> valueSupplier, Supplier<Angle> targetSupplier, Supplier<Angle> toleranceSupplier) {
        return ()->valueSupplier.get().baseUnitMagnitude() > (targetSupplier.get().baseUnitMagnitude() - (toleranceSupplier.get().baseUnitMagnitude())) &&
            ((valueSupplier.get().baseUnitMagnitude() < (targetSupplier.get().baseUnitMagnitude()) + (toleranceSupplier.get().baseUnitMagnitude())));
       
    }
}
