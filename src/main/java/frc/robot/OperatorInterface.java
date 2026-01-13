package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class OperatorInterface {
    public CommandJoystick joytick = new CommandJoystick(0);

    public static void create(SubsystemManager subsystemManager){
        final CommandSwerveDrivetrain drive = subsystemManager.CommandSwerveDrivetrain;
        
     
}
