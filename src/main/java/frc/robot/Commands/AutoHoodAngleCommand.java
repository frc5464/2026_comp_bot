package frc.robot.Commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.Vision;

public class AutoHoodAngleCommand extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem m_ShooterSubsystem;
    

    public AutoHoodAngleCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter){
        this.drivetrain = drivetrain;
        this.m_ShooterSubsystem = shooter;
        addRequirements(shooter);
    }

//     }

//     @Override
//     public void execute() {
//         m_ShooterSubsystem.changeAngle(desiredposition);
//         /*need to calculate output using estimated camera z-axis and shoothinge motor encoder
//          * 
//          * if z_value is further away, then the motor rotates as needed
//         */
//     }

//     @Override
//     public void end(boolean interrupted){

//     }

//     @Override
//     public boolean isFinished(){
//         return false;
//     }
// }
