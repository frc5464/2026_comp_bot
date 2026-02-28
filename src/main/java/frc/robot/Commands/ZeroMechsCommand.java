package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ZeroMechsCommand extends Command{
    
    private IntakeSubsystem intake;
    private ShooterSubsystem shoot;
    private ClimbSubsystem climb;
    public double resetType;

    public ZeroMechsCommand(IntakeSubsystem intake, ShooterSubsystem shoot, double resetType){
        this.intake = intake;
        this.shoot = shoot;
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        if(resetType == 0){
            intake.reBoot();
        } else if(resetType == 1){
            shoot.reBoot();
        } /*else if(resetType == 2){
            climb.reBoot();
        } */
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
