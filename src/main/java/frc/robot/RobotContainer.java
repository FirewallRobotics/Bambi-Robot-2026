// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignWithClimberCommand;
import frc.robot.commands.AngleAndRunIntakeCommand;
import frc.robot.commands.AngleArmCommand;
import frc.robot.commands.HonkCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualKicker;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveAssistanceSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  /** The maximum lateral (X, Y) speed of the robot */
  private double MaxSpeed = Units.feetToMeters(12); // kSpeedAt12Volts desired top speed

  /** The maximum rotational speed of the robot */
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /** The main drive request that moves the robot in field orientation */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /** The command for braking the drivetrain */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /** The command for facing towards the HUB */
  private final SwerveRequest.FieldCentricFacingAngle face =
      new SwerveRequest.FieldCentricFacingAngle();

  /** Creates the telemetry subsystem for the drivetrain (Auto created by CTRE) */
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /** getter for the drivetrain's telemetry subsystem */
  public Telemetry getLogger() {
    return logger;
  }

  /** Object that allows for binding to the primary drivers xbox controller */
  public final CommandXboxController joystick = new CommandXboxController(0);

  /** Object that allows for binding to the secondary drivers xbox controller */
  public final CommandXboxController secondDriver = new CommandXboxController(1);

  /** The only instance of the subsystem that controls the drivetrain */
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /** The only instance of the subsystem that controls the sucking part of the intake */
  private final IntakeSubsystem intakeSubsystem;

  /** Subsystem for assisting the drivers */
  private final DriveAssistanceSubsystem driveAssistanceSubsystem;

  /** The only instance of the Subsystem that talks to the Limelights/vision system */
  @SuppressWarnings("unused")
  private final VisionSubsystem visionSubsystem;

  /** The only instance of the subsystem that controls the shooting mechanism */
  private final ShooterSubsystem shooterSubsystem;

  /** The only instance of the subsystem that controls the intake arm */
  private final IntakeArmSubsystem armSubsystem;

  /** The only instance of the subsystem that controls the storage agitator belts */
  private final AgitatorSubsystem agitatorSubsystem;

  /** The only instance of the subsystem that controls the lifting mechanism on the shooter */
  private final KickerSubsystem kickerSubsystem;

  public RobotContainer() {

    // setup the HUB facing command
    // This was initally in driver assistance subsystem but for whatever reason it only seemed to
    // work here ¯\_(ツ)_/¯
    // Setup the PID for rotating
    face.HeadingController = new PhoenixPIDController(6, 0, 0);
    // The angle given by vision subsystem points positive X
    face.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
    // This allows for the robot to rotate around smoothly
    // without this line the robot will do a full turn to follow a point passing over 360 degrees
    face.HeadingController.enableContinuousInput(-1, 1);

    // create the subsystems
    intakeSubsystem = new IntakeSubsystem();
    driveAssistanceSubsystem = new DriveAssistanceSubsystem(drivetrain, this);
    visionSubsystem = new VisionSubsystem(this);
    shooterSubsystem = new ShooterSubsystem(drivetrain);
    armSubsystem = new IntakeArmSubsystem();
    agitatorSubsystem = new AgitatorSubsystem();
    kickerSubsystem = new KickerSubsystem();

    // create the commands for use in pathplanner
    NamedCommands.registerCommand("Honk", new HonkCommand("la-cucaracha.chrp"));
    NamedCommands.registerCommand(
        "Shoot", new ShootCommand(shooterSubsystem, kickerSubsystem, agitatorSubsystem, false));
    NamedCommands.registerCommand(
        "Intake", new AngleAndRunIntakeCommand(armSubsystem, intakeSubsystem, agitatorSubsystem));
    NamedCommands.registerCommand("Climb", new AlignWithClimberCommand(drivetrain));

    // configure the controller bindings
    configureBindings();

    // debug for the intake arm
    // SmartDashboard.putNumber("Hold Voltage", 0);

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  private void configureBindings() {
    // setup the default command for driving field oriented
    // acts as a quick off switch for the drivetrain
    DriveFieldOriented();

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // set the bindings for the primary driver
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .rightTrigger()
        .whileTrue(driveAssistanceSubsystem.vibrateIfFaceingHUBDiscriptive(joystick));
    joystick
        .rightTrigger()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    face.withTargetDirection(
                        new Rotation2d(VisionSubsystem.getAngleToHUB(drivetrain)))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // register the telemetry (Auto generated)
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick
        .rightBumper()
        .whileTrue(new ShootCommand(shooterSubsystem, kickerSubsystem, agitatorSubsystem, true));
    joystick.leftBumper().whileTrue(new AngleArmCommand(armSubsystem, true));
    // joystick.povLeft().whileTrue(new AngleArmCommand(armSubsystem, false));

    // configure the second drivers controller
    secondDriver
        .povLeft()
        .whileTrue(
            new SequentialCommandGroup(
                new AngleArmCommand(armSubsystem, false),
                new IntakeCommand(intakeSubsystem, agitatorSubsystem, armSubsystem)));
    secondDriver.povLeft().whileFalse(new AngleArmCommand(armSubsystem, true));

    secondDriver
        .b()
        .whileTrue(new ShootCommand(shooterSubsystem, kickerSubsystem, agitatorSubsystem, false));

    secondDriver.y().whileTrue(new ManualKicker(kickerSubsystem));
  }

  public void periodic() {
    // periodically update the position of the joysticks so that they control the facing
    face.VelocityX = joystick.getLeftY() * MaxSpeed;
    face.VelocityY = joystick.getLeftX() * MaxSpeed;
  }

  /**
   * sets the default command for the drivetrain to drive in field orientation. This allows the
   * robot to move
   */
  public void DriveFieldOriented() {

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));
  }

  /**
   * Constructs a new PathPlannerAuto command.
   *
   * @param autoName the name of the autonomous routine to load and run
   * @throws AutoBuilderException if AutoBuilder is not configured before attempting to load the
   *     autonomous routine (which is the job of CommandSwerveDrivetrain)
   */
  public Command getAutonomousCommand(String name) {
    return new PathPlannerAuto(name);
  }
}
