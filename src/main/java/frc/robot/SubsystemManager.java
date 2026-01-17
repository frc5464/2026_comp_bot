package frc.robot;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

public class SubsystemManager {
    private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(null, null);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();




    public CommandSwerveDrivetrain getCommandSwerveDrivetrain(){
        return drivetrain;
    }

    public IntakeSubsystem getIntakeSubsystem(){
        return intakeSubsystem;
    }


}
