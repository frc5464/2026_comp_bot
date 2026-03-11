// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer/* = new RobotContainer()*/;
    // private Vision vision;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {

        m_robotContainer = new RobotContainer();
        SmartDashboard.putData("Auto Mode", m_robotContainer.autoChooser);
        // SmartDashboard.putData("apis", vision.cameras);
        
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        SmartDashboard.putBoolean("Manual Mode", Universals.manualMode);
        m_robotContainer.periodic();
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
    public void simulationPeriodic() {
        // Update drivetrain simulation
        // drivetrain.simulationPeriodic();
        // Update camera simulation
        m_robotContainer.vision.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);

        var debugField = m_robotContainer.vision.getSimDebugField();
        debugField.getObject("EstimatedRobot").setPose(m_robotContainer.drivetrain.getState().Pose);
        // debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.getModulePoses());

        // // Calculate battery voltage sag due to current draw
        // var batteryVoltage =
        //         BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.getCurrentDraw());

        // // Using max(0.1, voltage) here isn't a *physically correct* solution,
        // // but it avoids problems with battery voltage measuring 0.
        // RoboRioSim.setVInVoltage(Math.max(0.1, batteryVoltage));
    }

    public void uselessFunctionMauahahaha(){
        // This can be removed.
    }
}
