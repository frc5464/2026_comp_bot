package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

public class AutoTurretAngleCommand extends Command {
    
    private CommandSwerveDrivetrain drivetrain;
    private TurretSubsystem turret;

    public AutoTurretAngleCommand(CommandSwerveDrivetrain drivetrain, TurretSubsystem turret){
        this.drivetrain = drivetrain;
        this.turret = turret;
        addRequirements(turret);
    }

        @Override
        public void execute(){
            turret.autoAim(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY());

        }
}
