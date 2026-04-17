// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import perfectrobotcode.src;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.AutoHoodAngleCommand;
import frc.robot.Commands.AutoShootCommand;
import frc.robot.Commands.AutoTurretAngleCommand;
// import frc.robot.Commands.BeltCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.IntakeToPositionCommand;
import frc.robot.Commands.ManualIntakeToPositionCommand;
import frc.robot.Commands.ManualModeCommand;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.ShooterHoodCommand;
import frc.robot.Commands.ShuttleCommand;
// import frc.robot.Commands.ClimbToPositionCommand;
import frc.robot.Commands.SlowDriveModeCommand;
import frc.robot.Commands.ManualTurretCommand;
import frc.robot.Commands.ZeroGyroCommand;
import frc.robot.Commands.ZeroMechsCommand;
import frc.robot.Commands.Scrapped.FeedCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Vision;

// import frc.robot.subsystems.VisionRetry;
// import frc.robot.subsystems.Vision;


public class RobotContainer {

    private double MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric slowdrive = new SwerveRequest.FieldCentric()
    //         .withDeadband(SlowSpeed * 0.1).withRotationalDeadband(SlowSpeed * 0.2)
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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

    public CandleSubsystem candle = new CandleSubsystem();
    // public Vision vision = new Vision();

    // public OldVision vision = new OldVision();
    // public VisionRetry vision;

    //Create wait chooser
    private final SendableChooser<Double> m_waitChooser = new SendableChooser<>();

    public RobotContainer() {
        shoot.audio.AllowMusicDurDisable = true;
        // shoot.play();
        //Add options for wait times
        m_waitChooser.setDefaultOption("O seconds (No Delay)", 0.0);
        m_waitChooser.addOption("1 second", 1.0);
        m_waitChooser.addOption("3 second", 3.0);
        m_waitChooser.addOption("5 second", 5.0);
        SmartDashboard.putData("Auto Wait Time", m_waitChooser);

        NamedCommands.registerCommand("IntakeDown", new IntakeToPositionCommand(intake, 1));
        NamedCommands.registerCommand("IntakeUp", new IntakeToPositionCommand(intake, 0));
        NamedCommands.registerCommand("Intake", new IntakeCommand(intake, 3.25, true));
        NamedCommands.registerCommand("Shoot", new ShootCommand(shoot, belt, intake, false, 3));
        NamedCommands.registerCommand("LongShoot", new ShootCommand(shoot, belt, intake, false, 8));
        NamedCommands.registerCommand("LongIntake", new IntakeCommand(intake, 18, true));

        autoChooser = AutoBuilder.buildAutoChooser();
        // autoChooser.setDefaultOption("Pos3_20pc", getAutonomousCommand());
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand();

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
        
        // vision = new VisionRetry(drivetrain::addVisionMeasurement);
    }

    public void periodic(){
        shoot.periodic();
        intake.periodic();
        // vision.periodic();
    }

    public void configureBindings() {
        // silence the unplugged controller message if we are simulating!
        if(Robot.isSimulation()){
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveController.getLeftY() * MaxSpeed * Universals.driveSpeedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed * Universals.driveSpeedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );    
        // shoot.setDefaultCommand(new FeedCommand(shoot));
        // belt.setDefaultCommand(new BeltCommand(belt));
        turret.setDefaultCommand(new AutoTurretAngleCommand(drivetrain, turret));
        // shoot.setDefaultCommand(new AutoHoodAngleCommand(drivetrain, shoot));
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

        driveController.leftTrigger().whileTrue(new IntakeCommand(intake, 67, true));
        zackController.x().whileTrue(new IntakeCommand(intake, 67, false));
        // zackController.x().whileTrue(new ReverseIntakeCommand(intake));

        zackController.povUp().onTrue(new ShooterHoodCommand(shoot, true));
        zackController.povDown().onTrue(new ShooterHoodCommand(shoot, false));
        //rev up feeder motor up to speed, then shoots when up to speed
        driveController.rightTrigger().whileTrue(new ShootCommand(shoot, belt, intake, false, 67));
        driveController.rightBumper().whileTrue(new AutoShootCommand(shoot, belt, drivetrain, false, 67));
        driveController.leftBumper().whileTrue(new ShuttleCommand(shoot, belt, false, 67));

        zackController.back().whileTrue(new ManualModeCommand());
        zackController.y().whileTrue(new FeedCommand(shoot, belt, true, 67));
        // zackController.a().toggleOnTrue(new BeltCommand(belt));
        // zackController.rightBumper().whileTrue(new BeltCommand(belt));

        zackController.a().onTrue(new IntakeToPositionCommand(intake, 0));
        zackController.b().onTrue(new IntakeToPositionCommand(intake, 1));
        
        zackController.a().whileTrue(new ManualIntakeToPositionCommand(intake, 1));
        zackController.b().whileTrue(new ManualIntakeToPositionCommand(intake, -1));

        // testController.povUp().whileTrue(new ClimbUpCommand(climb, true));
        // testController.povDown().whileTrue(new ClimbUpCommand(climb, false));

        zackController.axisGreaterThan(4, 0.5).whileTrue(new ManualTurretCommand(turret, true));
        zackController.axisLessThan(4, -0.5).whileTrue(new ManualTurretCommand(turret, false));

        zackController.leftBumper().whileTrue(new ShootCommand(shoot, belt, intake, true, 67));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //joystick.leftTrigger(0.1).whileTrue(new IntakeCommand(intake, true));
        
        // Zero Gyro              //Reset the field-centric heading on left bumper press.
        driveController.start().onTrue(new ZeroGyroCommand(drivetrain));

        zackController.start().whileTrue(new ZeroMechsCommand(intake, shoot, turret, 0));
        // zackController.back().whileTrue(new ZeroMechsCommand(intake, shoot, 1));

        // joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        
    }


        


    public Command getAutonomousCommand() {
        //Get selected delay from SmartDashboard
        double waitTime = m_waitChooser.getSelected();
        // return m_waitChooser.getSelected();
        new WaitCommand(waitTime);

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
