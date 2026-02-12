package frc.robot.Commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.Vision;

// public class AutoAngleCommand extends Command{
//     private final Vision m_vision;
//     private final ShooterSubsystem m_ShooterSubsystem;
//     private double desiredposition;
//     public AutoAngleCommand(Vision camera, ShooterSubsystem shooter){
//         this.m_vision = camera;
//         this.m_ShooterSubsystem = shooter;
//     }

//     @Override
//     public void initialize() {

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
