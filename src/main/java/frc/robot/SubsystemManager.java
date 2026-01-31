package frc.robot;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision;
public class SubsystemManager {
    private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(null, null);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final Vision visionSubsystem = new Vision();



    public CommandSwerveDrivetrain getCommandSwerveDrivetrain(){
        return drivetrain;
    }

    public IntakeSubsystem getIntakeSubsystem(){
        return intakeSubsystem;
    }

    public Vision getVision(){
        return visionSubsystem;
    }


}
