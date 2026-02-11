// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.ClimbUpCommand;
import frc.robot.Commands.DummyCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.IntakeToPositionCommand;
import frc.robot.Commands.ShootCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private IntakeCommand intakeCommand;
    private ShootCommand shootCommand;
    private DummyCommand dummyCommand;
    private ClimbUpCommand climbCommand;
    private IntakeToPositionCommand downers;
    private IntakeToPositionCommand uppies;
    // private IntakeSubsystem intake;

    private final RobotContainer m_robotContainer/* = new RobotContainer()*/;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        
        // ShooterSubsystem shoot = new ShooterSubsystem();
        // ClimbSubsystem climb = new ClimbSubsystem();
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData("Auto Mode", m_robotContainer.autoChooser);
        
        
        // intakeCommand = new IntakeCommand(m_robotContainer.intake);
        // // shootCommand = new ShootCommand(shoot);
        // dummyCommand = new DummyCommand();
        // // climbCommand = new ClimbUpCommand(climb, true);
        // downers = new IntakeToPositionCommand(m_robotContainer.intake, 1);
        // uppies = new IntakeToPositionCommand(m_robotContainer.intake, 0);

        // NamedCommands.registerCommand("Intake", intakeCommand);
        // NamedCommands.registerCommand("Shoot", shootCommand);
        // NamedCommands.registerCommand("Dummy", dummyCommand);
        // NamedCommands.registerCommand("Climb", climbCommand);
        // NamedCommands.registerCommand("IntakeDown", downers);
        // NamedCommands.registerCommand("IntakeUp", uppies);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        m_robotContainer.intake.periodic();
        m_robotContainer.shoot.periodic();
        // SmartDashboard.putBoolean("brakeMode", Universals.brakeMode);
        // SmartDashboard.putBoolean("zeroGyro", Universals.zeroGyro);
        
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    public void uselessFunctionMauahahaha(){
        // This can be removed.
    }
}
