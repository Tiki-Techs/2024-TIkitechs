// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.HoodPositioner;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));
  public static final Shooter s_Shooter = new Shooter();
  private final Intake s_Intake = new Intake();
  private final Climb s_Climb = new Climb();
  public static final Hood s_Hood = new Hood();
  public final Vision s_Vision = new Vision();
  public final Lights s_Lights = new Lights();
  public static XboxController driverXbox = new XboxController(0);
  public static XboxController mechXbox = new XboxController(1);

  private static AprilTagFieldLayout m_fieldLayout;
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    NamedCommands.registerCommand("Start Intake", s_Intake.StartIntake());
    NamedCommands.registerCommand("Stop Intake", s_Intake.StopIntake());
    NamedCommands.registerCommand("Aim And Shoot", s_Shooter.AutoShoot());
    NamedCommands.registerCommand("Fire", s_Shooter.Fire());
    NamedCommands.registerCommand("Stop Shooter", s_Shooter.StopShooter());
    AutoBuilder autochooser = new AutoBuilder();
    m_fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue
    m_fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
     autoChooser = AutoBuilder.buildAutoChooser("Default Auto");

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX());

    s_Shooter.setDefaultCommand(new RunCommand(() -> s_Shooter.RunShooter(mechXbox.getRightTriggerAxis(), false), s_Shooter));
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    s_Climb.setDefaultCommand(new RunCommand(() -> s_Climb.move(mechXbox.getLeftTriggerAxis()*0.2), s_Climb));
    s_Hood.setDefaultCommand(new HoodPositioner(s_Hood));
    s_Intake.setDefaultCommand(new RunCommand(() -> s_Intake.run(), s_Intake));
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Y Button
    new JoystickButton(driverXbox, 4).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(mechXbox, 1).onFalse(new InstantCommand(() -> s_Shooter.quickReverse()));
  }

  private static AprilTag speakerSupplier() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
        ?
        // blue speaker
        new AprilTag(7, m_fieldLayout.getTagPose(7).get())
        : new AprilTag(4, m_fieldLayout.getTagPose(4).get());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    // // // // // // // drivebase.setMotorBrake(brake);
  }
}
