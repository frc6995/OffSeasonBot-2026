package frc.robot.subsystems.turret;

public interface TurretIO {

    public class TurretIOInputs {
        public double angle;
        public double appliedVolts;
        public double statorCurrent;
        public double supplyCurrent;
    }

    public void setAngle(double angle);
    public void resetEncoder();
    public void updateInputs(TurretIOInputs inputs);
    public void stop();

}
