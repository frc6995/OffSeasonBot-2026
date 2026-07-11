package frc.robot.subsystems.hood;

import com.ctre.phoenix6.hardware.TalonFX;

public class HoodIOTalonFX implements HoodIO{   
    //need to specify upper or lower CAN bus
    protected final TalonFX hood_motor = new TalonFX(Hood.HoodConstants.kCANID); 

    public HoodIOTalonFX() {

    }

    public void configMotor() {
        
    }

    @Override
    public void resetEncoder() {
        
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        
    }

    @Override
    public void setAngle(double angle) {
        
    }
    
}
