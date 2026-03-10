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
        // if(shoot.encoderPos > shoot.hoodlimitup && shoot.encoderPos < shoot.hoodlimitdown){
        shoot.changeAngle(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY());
        // } else if(shoot.encoderPos < shoot.hoodlimitup){
            // shoot.targetPosition = shoot.hoodlimitup;
        // } else{
            // shoot.targetPosition = shoot.hoodlimitdown;
        // }
        
        
        /* Use the X value in pose estimation to raise the angle the smaller X equals. */
    }

    @Override
    public void end(boolean interrupted){
        if(shoot.targetpos < -4 || shoot.targetpos > -0.4){
            
        }

        //the calculated targetpos is outside of safe range, write limit
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
