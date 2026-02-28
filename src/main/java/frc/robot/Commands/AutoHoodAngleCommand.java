package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.Vision;

public class AutoHoodAngleCommand extends Command{
    // private final CommandSwerveDrivetrain drivetrain;
    // private final ShooterSubsystem m_ShooterSubsystem;
    

    public AutoHoodAngleCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter){
        // this.drivetrain = drivetrain;
        // this.m_ShooterSubsystem = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // m_ShooterSubsystem.changeAngle(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY());
        /* Use the X value in pose estimation to raise the angle the smaller X equals. */
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
