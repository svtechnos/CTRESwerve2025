package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFXS armMotor;
    private final PositionDutyCycle positionControl = new PositionDutyCycle(0);

    public ArmSubsystem(int motorID) {
        armMotor = new TalonFXS(motorID);
       
    }

    public void moveArm() {
        armMotor.set(5);
    }

    public void stopArm() {
        armMotor.set(0);
    }
}
    