package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ZeroMechsCommand extends Command{
    
    private IntakeSubsystem intake;
    private ClimbSubsystem climb;

    public ZeroMechsCommand(IntakeSubsystem intake/*, ClimbSubsystem climb*/){
        this.intake = intake;
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        intake.reBoot();
        // climb.reBoot();
        SmartDashboard.putBoolean("MechsZeroed", true);
    }

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("MechsZeroed", false);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}
