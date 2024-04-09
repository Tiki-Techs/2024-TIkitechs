
package frc.robot.commands.swervedrive.auto;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class SimpleAuto extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean overrideJS;

  private SwerveSubsystem swerve;
  private Shooter shooter;
  private Hood hood;
  private Intake intake;
  private SlewRateLimiter yLim = new SlewRateLimiter(1);
  private SlewRateLimiter xLim = new SlewRateLimiter(1);
  private SlewRateLimiter rotLim = new SlewRateLimiter(3);
  private final Timer m_timer = new Timer();
  private boolean done = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve        The subsystem used by this command.
   * @param fieldRelative Field Relative Boolean
   */

  public SimpleAuto(SwerveSubsystem swerve, Shooter shooter, Intake intake, Hood hood, boolean fieldRelative) {
    this.shooter = shooter;
    this.intake = intake;
    this.swerve = swerve;
    this.hood = hood;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    addRequirements(shooter);
    addRequirements(intake);
    // this.driver = driver;
    this.fieldRelative = fieldRelative;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("timer", m_timer.get());
    double autoyAxis = 0.7;
    double autoxAxis = 0.0;
    double rotAxis = 0.0;

    Translation2d translation = new Translation2d(
        autoyAxis, autoxAxis);
    Translation2d stop = new Translation2d(0.0, 0.0);
    while (m_timer.get() < 3) {
      RobotContainer.s_Shooter.m_pidController.setReference(4500, ControlType.kVelocity);
      RobotContainer.positioner.m_Leader
          .set(RobotContainer.positioner.hoodPID.calculate(RobotContainer.s_Hood.getRotation(), 57));
      // dont do anything
    }

    while (m_timer.get() < (3.5)) {
      shooter.index.set(-1);
    }

    shooter.index.set(0);

    while (m_timer.get() < 10) {
      RobotContainer.s_Shooter.m_pidController.setReference(0, ControlType.kVelocity);
      var setpoint = Math.max(HoodConstants.lowerLimit, Math.min(65, HoodConstants.upperLimit));
      RobotContainer.positioner.m_Leader
          .set(RobotContainer.positioner.hoodPID.calculate(RobotContainer.s_Hood.getRotation(), setpoint));
    }
    while (m_timer.get() < 15) {
      swerve.drive(translation, rotAxis, fieldRelative);
    }
    swerve.drive(stop, rotAxis, fieldRelative);
    m_timer.stop();
    done = true;

    // if(xAxis == 0 && yAxis == 0 && rotAxis == 0){
    // swerve.zeroModules();
    // swerve.stopModules();
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
