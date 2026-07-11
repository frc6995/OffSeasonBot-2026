package frc.robot.subsystems.hood;

public interface HoodIO {

    public void setAngle(double angle);
    public void resetEncoder();
    public void updateInputs(HoodIOInputs inputs);
    
    public static class HoodIOInputs {
        public double angle;

    }
}
