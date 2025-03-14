package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    final String armCANbus = "arm";
    TalonFXS talon1 = new TalonFXS(0, armCANbus);

    //private boolean isRunning = false;

    public ArmSubsystem() {
        
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Slot0.kP = 1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 10;
        config.Slot0.kV = 2;
        talon1.getConfigurator().apply(config);
    }

    public void move1(double position){
        talon1.setPosition(position);

    }
        

   
   

  
 
}
    