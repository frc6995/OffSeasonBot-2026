package frc.robot.subsystems.hood;

public interface HoodIO {

    public void setAngle(double angle);
    public void resetEncoder();
    public void updateInputs(HoodIOInputs inputs);
    public void disable();
    
    public static class HoodIOInputs {
        public double angle;

        public double appliedVolts;
        public double statorCurrent;
        public double supplyCurrent;

    }
}
