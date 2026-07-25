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
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AngleAndRunIntakeCommand;
import frc.robot.commands.AngleArmCommand;
import frc.robot.commands.HonkCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualKicker;
import frc.robot.commands.PanicKicker;
import frc.robot.commands.PanicShooter;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SwitchConstantOff;
import frc.robot.commands.SwitchWiggleCommand;
import frc.robot.generated.TunerConstants;
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

  private final SwerveRequest.RobotCentricFacingAngle adjust =
      new SwerveRequest.RobotCentricFacingAngle();

  private final SwerveRequest.ApplyFieldSpeeds wiggle = new SwerveRequest.ApplyFieldSpeeds();

  boolean wiggleDirection = false;
  double wiggleCounter = 0;

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
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

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

  /** The only instance of the subsystem that controls the lifting mechanism on the shooter */
  private final KickerSubsystem kickerSubsystem;

  /** Auto chooser. Sent to driver dashboard so they can pick an auto to run */
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {

    SmartDashboard.putBoolean("Wiggle", false);

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

    adjust.HeadingController = new PhoenixPIDController(6, 0, 0);
    wiggle.ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

    // create the subsystems
    intakeSubsystem = new IntakeSubsystem();
    visionSubsystem = new VisionSubsystem();
    shooterSubsystem = new ShooterSubsystem(drivetrain);
    armSubsystem = new IntakeArmSubsystem();
    kickerSubsystem = new KickerSubsystem();
    driveAssistanceSubsystem = new DriveAssistanceSubsystem(drivetrain, this, shooterSubsystem);

    // create the commands for use in pathplanner
    NamedCommands.registerCommand("Honk", new HonkCommand("la-cucaracha.chrp"));
    NamedCommands.registerCommand("ManualKicker", new ManualKicker(kickerSubsystem));
    NamedCommands.registerCommand(
        "Intake", new AngleAndRunIntakeCommand(armSubsystem, intakeSubsystem));

    // configure the controller bindings
    configureBindings();

    // debug for the intake arm
    // SmartDashboard.putNumber("Hold Voltage", 0);

    // Add the autos we have and send it to smartdashboard
    m_chooser.addOption("Lynk Rush (Y O I N K)", new PathPlannerAuto("New Auto"));
    SmartDashboard.putData("Auto Chooser", m_chooser);

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(new PathPlannerAuto("Warm Up"));
  }

  private void configureBindings() {
    // setup the default command for driving field oriented
    // acts as a quick off switch for the drivetrain
    DriveFieldOriented();

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
    joystick
        .b()
        .whileTrue(
            drivetrain.driveToPose(
                new Pose2d(3.55, 1.15, new Rotation2d(Math.toRadians(135))),
                new Translation2d(0, 0.5)));
    joystick.leftTrigger().whileTrue(new ManualKicker(kickerSubsystem));
    joystick
        .y()
        .whileTrue(
            drivetrain.driveToPose(
                new Pose2d(2, 3.75, new Rotation2d(Math.toRadians(-90))),
                new Translation2d(0, -0.5)));
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    joystick.leftTrigger().whileTrue(new ManualKicker(kickerSubsystem));
    joystick
        .leftTrigger()
        .and(() -> SmartDashboard.getBoolean("Wiggle", true))
        .whileTrue(drivetrain.applyRequest(() -> wiggle));

    // Reset the field-centric heading on left bumper press.
    joystick.leftBumper().whileFalse(new AngleArmCommand(armSubsystem, true));
    joystick
        .leftBumper()
        .whileTrue(
            new SequentialCommandGroup(
                new AngleArmCommand(armSubsystem, false),
                new IntakeCommand(intakeSubsystem, armSubsystem)));
    joystick.povRight().onTrue(new SwitchConstantOff());
    // register the telemetry (Auto generated)
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.rightBumper().whileTrue(new ShootCommand(shooterSubsystem, kickerSubsystem, true));
    joystick
        .rightBumper()
        .and(() -> SmartDashboard.getBoolean("Wiggle", true))
        .whileTrue(drivetrain.applyRequest(() -> wiggle));
    // joystick.povLeft().whileTrue(new AngleArmCommand(armSubsystem, false));

    // configure the second drivers controller
    secondDriver.y().whileTrue(new PanicKicker(kickerSubsystem, true));
    secondDriver.a().whileTrue(new PanicKicker(kickerSubsystem, false));
    secondDriver.povDown().whileTrue(new PanicShooter(shooterSubsystem));
    secondDriver.leftBumper().whileTrue(new SwitchConstantOff());
    secondDriver.rightBumper().whileTrue(new SwitchWiggleCommand());
  }

  public void periodic() {
    // periodically update the position of the joysticks so that they control the facing
    face.VelocityX = -joystick.getLeftY() * MaxSpeed;
    face.VelocityY = -joystick.getLeftX() * MaxSpeed;

    if (wiggleDirection) {
      wiggle.Speeds =
          new ChassisSpeeds(joystick.getLeftY() * MaxSpeed, joystick.getLeftX() * MaxSpeed, 1);
      wiggleCounter += 1;
    } else {
      wiggle.Speeds =
          new ChassisSpeeds(joystick.getLeftY() * MaxSpeed, joystick.getLeftX() * MaxSpeed, -1);
      wiggleCounter += 1;
    }

    if (wiggleCounter > 15) {
      wiggleCounter = 0;
      if (wiggleDirection) {
        wiggleDirection = false;
      } else {
        wiggleDirection = true;
      }
    }
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
   * @throws AutoBuilderException if AutoBuilder is not configured before attempting to load the
   *     autonomous routine (which is the job of CommandSwerveDrivetrain)
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
