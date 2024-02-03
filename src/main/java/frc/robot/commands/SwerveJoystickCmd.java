package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class SwerveJoystickCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean overrideJS;
  private double yAxis;
  private double xAxis;
  private SwerveSubsystem swerve;
  private CommandXboxController driver;
  private SlewRateLimiter yLim = new SlewRateLimiter(1);
  private SlewRateLimiter xLim = new SlewRateLimiter(1);
  private SlewRateLimiter rotLim = new SlewRateLimiter(3);



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveJoystickCmd(SwerveSubsystem swerve, CommandXboxController driver, boolean fieldRelative) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

    this.driver = driver;
    this.fieldRelative = fieldRelative;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(driver.getLeftY()) < Constants.chassisConstants.deadBand) {
      yAxis = 0;
    }
    else {
      yAxis = 0.4*driver.getLeftY() + 0.6* Math.pow(driver.getLeftY(), 3);
    }

    if (Math.abs(driver.getLeftX()) < Constants.chassisConstants.deadBand) {
      xAxis = 0;
    }
    else {
      xAxis = 0.4*driver.getLeftX() + 0.6* Math.pow(driver.getLeftX(), 3);
    }
  
  //  double yAxis = driver.getLeftY();
    //double xAxis = driver.getLeftX();

    
    translation = new Translation2d(yAxis, xAxis).times(10);
    
    if (Math.abs(driver.getRightX()) < 0.15) {
      rotation = 0;
    }
    else {
      rotation =  driver.getRightX()* 0.33;
    }
    swerve.drive(translation, rotation, fieldRelative);

    // if(xAxis == 0 && yAxis == 0 && rotAxis == 0){
    //   swerve.zeroModules();
    //   swerve.stopModules();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
