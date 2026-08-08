package frc.robot.subsystems.hood;


public interface HoodIO {

    public default void setAngle(double angle) {};
    public default void resetEncoder() {};
    public default void updateInputs(HoodIOInputs inputs) {};
    public default void disable() {};
    
    public static class HoodIOInputs {
        public double angle;

        public double appliedVolts;
        public double statorCurrent;
        public double supplyCurrent;
        public boolean hoodMotorConnected;
    }
}
