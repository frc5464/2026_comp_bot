package frc.robot;

import frc.robot.subsystems.ExampleSubsystem;

public class SubsystemManager {
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

    public ExampleSubsystem getExampleSubsystem() {
        return exampleSubsystem;
    }
}
