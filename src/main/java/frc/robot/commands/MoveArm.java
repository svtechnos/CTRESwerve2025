package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class MoveArm extends Command{
    private final ArmSubsystem armSubsystem;
    private final double targetPosition;
    private static final double tolerance = 10.0;
    
    public MoveArm(ArmSubsystem armSubsystem, double targetPosition){
        this.armSubsystem = armSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(armSubsystem);
        
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {                                 
        
    }

    @Override
    public boolean isFinished() {
        return false;
        //return Math.abs(armSubsystem.getPosition() - targetPosition) < tolerance;
    }

    // public void end(boolean interrupted){
    //     if (interrupted){
    //         armSubsystem.stopArm();
    //     }
    // }


    
}
