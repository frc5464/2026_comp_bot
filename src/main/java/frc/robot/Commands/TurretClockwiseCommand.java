package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

// JAKEREVIEW: What is this for? When will the drivers need to use this?

public class TurretClockwiseCommand extends Command{
    
    private TurretSubsystem turret;

    private boolean normalRot;

    public TurretClockwiseCommand(TurretSubsystem turret, boolean normalRot){
        this.turret = turret;
    }

    @Override
    public void execute(){
        if(normalRot == true){
        turret.clockwise();
        } else{
            turret.counterclockwise();
        }
    }

    @Override
    public void end(boolean interrupted){
        turret.stop();
    }
}