// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.AutoHoodAngleCommand;
import frc.robot.Commands.AutoTurretAngleCommand;
import frc.robot.Commands.DummyCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.IntakeToPositionCommand;
import frc.robot.Commands.ManualIntakeToPositionCommand;
import frc.robot.Commands.ReverseShooterCommand;
import frc.robot.Commands.ShootCommand;
// import frc.robot.Commands.ClimbToPositionCommand;
import frc.robot.Commands.SlowDriveModeCommand;
import frc.robot.Commands.TurretClockwiseCommand;
import frc.robot.Commands.TurretCounterclockwiseCommand;
import frc.robot.Commands.ZeroGyroCommand;
import frc.robot.Commands.ZeroMechsCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

    private double MaxSpeed = 0.25 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric slowdrive = new SwerveRequest.FieldCentric()
    //         .withDeadband(SlowSpeed * 0.1).withRotationalDeadband(SlowSpeed * 0.2)
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController zackController = new CommandXboxController(1);
    private final CommandJoystick testController = new CommandJoystick(2);
    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    /* Path Follow */
    public final SendableChooser<Command> autoChooser;

    public IntakeSubsystem intake = new IntakeSubsystem();
    public ShooterSubsystem shoot = new ShooterSubsystem();
    public ClimbSubsystem climb = new ClimbSubsystem();
    public TurretSubsystem turret = new TurretSubsystem();
    public BeltSubsystem belt = new BeltSubsystem();

    public RobotContainer() {
        NamedCommands.registerCommand("IntakeDown", new IntakeToPositionCommand(intake, 1));
        NamedCommands.registerCommand("IntakeUp", new IntakeToPositionCommand(intake, 0));
        NamedCommands.registerCommand("Intake", new IntakeCommand(intake));
        NamedCommands.registerCommand("Shoot", new ShootCommand(shoot, belt));
        // NamedCommands.registerCommand("ClimbUp", new ClimbToPositionCommand(climb, 0));
        // NamedCommands.registerCommand("ClimbDown", new ClimbToPositionCommand(climb, 1));
        

        autoChooser = AutoBuilder.buildAutoChooser();
        // autoChooser.setDefaultOption("Pos3_20pc", getAutonomousCommand());
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        // boolean isCompetition = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        // autoChooser = AutoBuilder.buildAutoChooser();
        // (stream) -> isCompetition
            // ? stream.filter(auto -> auto.getName().startsWith("comp")) : stream); 

        configureBindings();

        //controller deadband for drive controller
        double driveX = driveController.getRawAxis(1);
        double driveY = driveController.getRawAxis(0);
        double driveRot = -driveController.getRawAxis(4);
        if(Math.abs(driveX) < 0.1){ driveX = 0;}
        if(Math.abs(driveY) < 0.1){ driveY = 0;}
        if(Math.abs(driveRot) < 0.1){ driveRot = 0;}

        //controller deadband for zack's controller
        double zackDriveX = zackController.getRawAxis(1);
        double zackDriveY = zackController.getRawAxis(0);
        double zackDriveRot = -zackController.getRawAxis(4);
        if(Math.abs(zackDriveX) < 0.1){ driveX = 0;}
        if(Math.abs(zackDriveY) < 0.1){ driveY = 0;}
        if(Math.abs(zackDriveRot) < 0.1){ driveRot = 0;}

        //controller deadband for test controller
        double tDriveX = testController.getRawAxis(0);
        double tDriveY = testController.getRawAxis(1);
        double tDriveRot = -testController.getRawAxis(4);
        if(Math.abs(tDriveX) < 0.1){ driveX = 0;}
        if(Math.abs(tDriveY) < 0.1){ driveY = 0;}
        if(Math.abs(tDriveRot) < 0.1){ driveRot = 0;}
    }

    public void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driveController.getLeftY() * MaxSpeed * Universals.driveSpeedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(driveController.getLeftX() * MaxSpeed * Universals.driveSpeedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );    
        turret.setDefaultCommand(new AutoTurretAngleCommand(drivetrain, turret));
        shoot.setDefaultCommand(new AutoHoodAngleCommand(drivetrain, shoot));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //Brake Mode
        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        //point Mode
        driveController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        ));

        driveController.x().whileTrue(new SlowDriveModeCommand());

        driveController.leftTrigger().whileTrue(new IntakeCommand(intake));
 
        //rev up feeder motor up to speed, then shoots when up to speed
        driveController.rightTrigger().whileTrue(new ShootCommand(shoot, belt));

        zackController.a().onTrue(new IntakeToPositionCommand(intake, 0));
        zackController.b().onTrue(new IntakeToPositionCommand(intake, 1));
        
        driveController.povUp().whileTrue(new ManualIntakeToPositionCommand(intake, 0));
        driveController.povDown().whileTrue(new ManualIntakeToPositionCommand(intake, 1));


        // testController.povUp().whileTrue(new ClimbUpCommand(climb, true));
        // testController.povDown().whileTrue(new ClimbUpCommand(climb, false));

        // testController.axisGreaterThan(3, 0.5).whileTrue(new TurretClockwiseCommand(turret));
        // testController.axisLessThan(3, -0.5).whileTrue(new TurretCounterclockwiseCommand(turret));

        zackController.leftBumper().whileTrue(new ReverseShooterCommand(shoot));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //joystick.leftTrigger(0.1).whileTrue(new IntakeCommand(intake, true));
        
        // Zero Gyro              //Reset the field-centric heading on left bumper press.
        driveController.start().onTrue(new ZeroGyroCommand(drivetrain));

        driveController.back().whileTrue(new ZeroMechsCommand(intake));

        // joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        
    }


        


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

        // // // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );

    }
}
