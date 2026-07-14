package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class TriggerUtil {
    /**
     * Creates a {@link java.util.function.BooleanSupplier BooleanSupplier} to check if the robot is within a radius of a certain pose.
     * 
     * @param targetLocation A supplier for the target location as a field-relative Translation2d
     * @param robotPose A supplier for the pose of the robot
     * @param tolerance The radius for the robot to be within 
     * @return A BooleanSupplier to check if the condition is met
     */
    public static BooleanSupplier isWithinRadius(Supplier<Translation2d> targetLocation, Supplier<Pose2d> robotPose, Supplier<Distance> tolerance) {
        return () -> targetLocation.get().getDistance(robotPose.get().getTranslation()) < tolerance.get().in(Meters);
    }

    /**
     * Creates a {@link java.util.function.BooleanSupplier BooleanSupplier} to check if the robot pose is within a certain zone on the field.
     * 
     * @param lowerLeftCorner A supplier for the lower left corner of the zone as a field-relative Translation2d
     * @param upperRightCornerA supplier for the upper right corner of the zone as a field-relative Translation2d
     * @param robotPose A supplier for the pose of the robot
     * @return A BooleanSupplier to check if the condition is met
     */
    public static BooleanSupplier isWithinZone(Supplier<Translation2d> lowerLeftCorner, Supplier<Translation2d> upperRightCorner, Supplier<Pose2d> robotPose) {
        return () -> lowerLeftCorner.get().getX() < robotPose.get().getX() && lowerLeftCorner.get().getY() < robotPose.get().getY() &&
            upperRightCorner.get().getX() > robotPose.get().getX() && upperRightCorner.get().getY() > robotPose.get().getY();
    }
    
    /**
     * Creates a {@link java.util.function.BooleanSupplier BooleanSupplier} to check if a given value is within a certain range
     * 
     * @param valueSupplier The Supplier which provides a double value to check 
     * @param lowerSupplier The Supplier which provides a double value as a lower limit 
     * @param upperSupplier The Supplier which provides a double value as an upper limit
     * @return A BooleanSupplier to check if the condition is met
     */
    public static BooleanSupplier isValueWithinRange(Supplier<Double> valueSupplier, Supplier<Double> lowerSupplier, Supplier<Double> upperSupplier) {
        return () -> lowerSupplier.get() < valueSupplier.get() && valueSupplier.get() < upperSupplier.get();
    }

    public static BooleanSupplier isWithinTolerance(Supplier<Double> valueSupplier, Supplier<Double> targetSupplier, Supplier<Double> toleranceSupplier) {
        return () -> Math.abs(valueSupplier.get() - targetSupplier.get()) < toleranceSupplier.get();
    }

    /**
     * Negates the given BooleanSupplier
     * 
     * @param booleanSupplier The given BooleanSupplier to negate
     * @return A BooleanSupplier to check if the condition is met
     */
    public static BooleanSupplier negate(BooleanSupplier booleanSupplier) {
        return () -> !booleanSupplier.getAsBoolean();
    }

    /**
     * Creates a BooleanSupplier which applies an AND condition to two BooleanSuppliers 
     * 
     * @param supplierOne The first BooleanSupplier
     * @param supplierTwo The second BooleanSupplier
     * @return A BooleanSupplier to check if the condition is met
     */
    public static BooleanSupplier and(BooleanSupplier supplierOne, BooleanSupplier supplierTwo) {
        return () -> supplierOne.getAsBoolean() && supplierTwo.getAsBoolean();
    }

    /**
     * Creates a BooleanSupplier which applies an OR condition to two BooleanSuppliers 
     * 
     * @param supplierOne The first BooleanSupplier
     * @param supplierTwo The second BooleanSupplier
     * @return A BooleanSupplier to check if the condition is met
     */
    public static BooleanSupplier or(BooleanSupplier supplierOne, BooleanSupplier supplierTwo) {
        return () -> supplierOne.getAsBoolean() || supplierTwo.getAsBoolean();
    }

    public static BooleanSupplier debounce(BooleanSupplier supp, double time) {
        Debouncer debouncer = new Debouncer(time);
        return () -> debouncer.calculate(supp.getAsBoolean());
    }
}
