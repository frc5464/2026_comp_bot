// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.RaiseIntakeCommand;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.SlowDriveModeCommand;
// import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ZeroGyroCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    // Universals universals = new Universals();
    private double MaxSpeed = 0.25 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    
    private double SlowSpeed = 0.1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);// private SubsystemManager subsystemManager;
    private double SlowAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond);
    

    // /**
    //  * Connects this to joysticks
    //  * f
    //  * 
    //  * @param subsystemManager
    //  */
    // public static void create(SubsystemManager subsystemManager){
       // final CommandSwerveDrivetrain drivesubsystem = subsystemManager.getCommandSwerveDrivetrain();
        public IntakeSubsystem intake = new IntakeSubsystem();
        public ShooterSubsystem shoot = new ShooterSubsystem();
    
    // }
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric slowdrive = new SwerveRequest.FieldCentric()
            .withDeadband(SlowSpeed * 0.1).withRotationalDeadband(SlowSpeed * 0.2)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);
    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path Follow */
    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        // boolean isCompetition = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        // autoChooser = AutoBuilder.buildAutoChooser();
        // (stream) -> isCompetition
            // ? stream.filter(auto -> auto.getName().startsWith("comp")) : stream); 
        


        //controller deadband
        double driveX = joystick.getRawAxis(1);
        double driveY = joystick.getRawAxis(0);
        double driveRot = -joystick.getRawAxis(4);
        if(Math.abs(driveX) < 0.1){ driveX = 0;}
        if(Math.abs(driveY) < 0.1){ driveY = 0;}
        if(Math.abs(driveRot) < 0.1){ driveRot = 0;}
    }

    // public void driveNormal(){
    //     // Note that X is defined as forward according to WPILib convention,
    //     // and Y is defined as to the left according to WPILib convention.
    //     drivetrain.setDefaultCommand(
    //         // Drivetrain will execute this command periodically
    //         drivetrain.applyRequest(() ->
    //             drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
    //                 .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //                 .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //         )
    //     );
    // }

    // public void slowMode(){
    //     drivetrain.setDefaultCommand(
    //         drivetrain.applyRequest(() -> 
    //             slowdrive.withVelocityX(joystick.getLeftY() * SlowSpeed)
    //                     .withVelocityY(joystick.getLeftX() * SlowSpeed)
    //                     .withRotationalRate(-joystick.getRightX() * SlowAngularRate)
    //         )
    //     );
    // }
    public void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed * Universals.driveSpeedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed* Universals.driveSpeedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );    

        // drivetrain.applyRequest(() -> 
        //         slowdrive.withVelocityX(joystick.getLeftY() * SlowSpeed)
        //                 .withVelocityY(joystick.getLeftX() * SlowSpeed)
        //                 .withRotationalRate(-joystick.getRightX() * SlowAngularRate)
        // )
        //     );


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //Brake Mode
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        //point Mode
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.x().whileTrue(new SlowDriveModeCommand());

        joystick.leftTrigger().whileTrue(new IntakeCommand(intake));
        //reverse intake
        joystick.povUp().whileTrue(new RaiseIntakeCommand(intake));

        //rev up feeder motor up to speed, then shoots when up to speed
        joystick.rightTrigger().whileTrue(new ShootCommand(shoot));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        

        

        //joystick.leftTrigger(0.1).whileTrue(new IntakeCommand(intake, true));
        
        // Zero Gyro              //Reset the field-centric heading on left bumper press.
        joystick.start().onTrue(new ZeroGyroCommand(drivetrain));
        // joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //joystick2.leftBumper().whileTrue(new IntakeCommand(intake, false));



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
