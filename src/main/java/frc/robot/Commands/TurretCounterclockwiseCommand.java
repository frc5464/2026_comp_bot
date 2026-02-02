package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCounterclockwiseCommand extends Command{
    private TurretSubsystem turret;

    public TurretCounterclockwiseCommand(TurretSubsystem turret){
        this.turret = turret;
    }

    @Override
    public void execute(){
        turret.counterclockwise();
    }

    @Override
    public void end(boolean interrupted){
        turret.stop();
    }
}
