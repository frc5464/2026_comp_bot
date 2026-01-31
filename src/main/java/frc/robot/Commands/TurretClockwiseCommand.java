package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretClockwiseCommand extends Command{
    
    private TurretSubsystem turret;

    public TurretClockwiseCommand(TurretSubsystem turret){
        this.turret = turret;
    }

    @Override
    public void execute(){
        turret.clockwise();
    }

    @Override
    public void end(boolean interrupted){
        turret.stop();
    }
}
