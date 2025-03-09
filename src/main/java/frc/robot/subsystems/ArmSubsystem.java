package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFXS armMotor;
    private final PositionDutyCycle positionControl = new PositionDutyCycle(0);
    //private boolean isRunning = false;

    public ArmSubsystem(int motorID) {
        armMotor = new TalonFXS(motorID, "Arm");
    }

    @Override
    public void periodic() {
        armMotor.set(0.5);
    }
 
}
    