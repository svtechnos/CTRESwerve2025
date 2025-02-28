package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armMotor;
    private final PositionDutyCycle positionControl = new PositionDutyCycle(0);

    public ArmSubsystem(int motorID) {
        armMotor = new TalonFX(motorID);
        armMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
    }

    public void moveArm(double position) {
        armMotor.setControl(positionControl.withPosition(position));
    }

    public void stopArm() {
        armMotor.set(0);
    }
}
