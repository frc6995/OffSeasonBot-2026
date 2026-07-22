package frc.robot.subsystems.Flywheel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.util.CtreUtil;
public class Flywheel extends SubsystemBase{
  public static class FlywheelConstants {
    // PID Constants
    public static final double kP = 0.70;
    // Feedforward Constants
    public static final double kS = 0.25;
    public static final double kV = 0.18;
    // CAN IDs
    public static final int kLeadMotorCANID = 40;
    public static final int kFollowMotor1CANID = 41;
    public static final int kFollowMotor2CANID = 42;
    public static final int kFollowMotor3CANID = 43;
    // Motor Config Constants
    public static final boolean kInvertLeadMotor = true;
    public static final double kSupplyCurrentLimit = 40;
    public static final double kStatorCurrentLimit = 80;
    public static final double kNewMaxVoltage = 10;
    public static final double kNewMinVoltage = 0;
    public static final double kReduction = 1;
    public static final double kToleranceRPM = 100;
    public static final double FlywheelMOI = 0.000292639653; //meters^2 kg
    // Sim Constants
    // public static final double kDiameter = 2;
    // public static final double kMass = 4.15;

  }


  // public RealFlywheel() {
  //   new FlywheelIO() {
  //   };
  // }

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  private final FlywheelIO io;
  private final FlywheelIO.FlywheelInputs inputs = new FlywheelIO.FlywheelInputs();

  public enum State {
   
    DISABLED,
    
    ACTIVE
  }

private State flywheelState = State.DISABLED;

public void setState(State state) {
  flywheelState = state;

  }

  public State getShootState() {
    return flywheelState;
  }

  public void stop() {
    flywheelState = State.DISABLED;
  
  }

  public double getVelocityRPM() {
    return inputs.velocityRPM;
  }

  public double getAppliedVolts() {
    return inputs.appliedVolts;
  }

   public boolean areMotorsConnected() {
    return inputs.leadMotorConnected
        && inputs.followerMotor1Connected 
        && inputs.followerMotor1Connected 
        && inputs.followerMotor1Connected;
  }

@Override
public void periodic() {
   
    io.updateInputs(inputs);
    io.setVelocityRPM(ResolveTargetRPM(flywheelState));


}
//shoot is NOT 10000 rpm
private static double ResolveTargetRPM(State state) {
    return switch (state) {
      case DISABLED -> 0.0;
      case ACTIVE -> 10000;
    };
  }
}