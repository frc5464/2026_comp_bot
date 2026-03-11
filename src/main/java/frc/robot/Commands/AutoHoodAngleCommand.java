package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.Vision;

public class AutoHoodAngleCommand extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shoot;
    

    public AutoHoodAngleCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter){
        this.drivetrain = drivetrain;
        this.shoot = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shoot.changeAngle(shoot.xrobot, shoot.yrobot);
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
