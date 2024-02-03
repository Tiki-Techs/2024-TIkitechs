
package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.ControlModeValue;
import com.playingwithfusion.CANVenom.ControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.chassisConstants;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private TalonFXConfiguration turnConfig;
    private final TalonFX turnMotor;
  
    // private final TalonFXSensorCollection driveMotorEnc;
    // private final TalonFXSensorCollection turnMotorEnc
  
    //private DigitalInput absoluteEnc;
    public DutyCycleEncoder absoluteEncoder;
    public double absoluteEncoderOffset;
  
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.chassisConstants.kS, Constants.chassisConstants.kV, Constants.chassisConstants.kA); //Revise and 
    /** Creates a new ExampleSubsystem. */
    public SwerveModule(int driveMotorID, boolean driveMotorReversed, int turnMotorID, boolean turnMotorReversed, int absoluteEncoderID, double offset, double kP, double kI, double kD) {
       this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
       this.turnMotor = new TalonFX(turnMotorID);
       
     // driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);      
  
    //   // driveMotorEnc = new TalonFXSensorCollection(driveMotor);
    //   // turnMotorEnc = new TalonFXSensorCollection(turnMotor);
  
    //   //turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
  
    //   //driveMotor.setNeutralMode(NeutralMode.Coast);
    //   turnMotor.setNeutralMode(NeutralMode.Coast);
  
    //   // turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  
    //   driveMotor.setInverted(driveMotorReversed);
    //   turnMotor.setInverted(turnMotorReversed);
  
    //   //absoluteEnc = new DigitalInput(absoluteEncoderID);
  
      absoluteEncoder = new DutyCycleEncoder(absoluteEncoderID);
    //   absoluteEncoder.setConnectedFrequencyThreshold(1);
       this.absoluteEncoderOffset = offset;
       turnMotor.setPosition(0);
    //   absoluteEncoder.setPositionOffset(-0.5);
    //   absoluteEncoder.setDutyCycleRange(0, 1);
  
    //   turnMotor.config_kP(0, kP);
    //   turnMotor.config_kI(0, kI);
    //   turnMotor.config_kD(0, kD);
  
    //   zeroModules();
    }
  
    public void resetEncoder(){
       driveMotor.getEncoder().setPosition(0);
       turnMotor.setPosition(0);
     }
    public Rotation2d getTurnAngle(){
      return new Rotation2d((2 * Math.PI/(2048 * chassisConstants.turnMotorGearRat)) * (turnMotor.getRotorPosition().getValue() % (2048 * chassisConstants.turnMotorGearRat)));
    }
    public double falconToDegrees(double counts, double gearRatio){
      return counts * (360.0 / (gearRatio * 2048.0));
    }
  
    public double degreesToFalcons(double degrees, double gearRatio){
      double ticks = degrees / (360.0 / (gearRatio * 2048.0));
      return ticks;
    }
  
    public double falconToRPM(double velocityCounts, double gearRatio){
      double motorRPM = velocityCounts * (600.0 / 2048.0);
      double mechRPM = motorRPM / gearRatio;
      return mechRPM;
    }
  
    public double RPMToFalcon(double RPM, double gearRatio){
      double motorRPM = RPM * gearRatio;
      double sensorCounts = motorRPM * (2048.0 / 600.0);
      return sensorCounts;
    }
  
    public double falconToMPS(double velocitycounts, double circumference, double gearRatio){
      double wheelRPM = falconToRPM(velocitycounts, gearRatio);
      double wheelMPS = (wheelRPM * circumference) / 60;
      return wheelMPS;
    }
  
    public double MPSToFalcons(double velocity, double circumference, double gearRatio){
      double wheelRPM = ((velocity * 60) / circumference);
      double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
      return wheelVelocity;
    }
    public double falconToMeters(double count, double wheelCircumference, double gearRat){
      return count * (wheelCircumference / (gearRat * 2048));
    }
  
    public SwerveModuleState getState(){
      double velocity = falconToMPS(driveMotor.getEncoder().getVelocity(), Constants.chassisConstants.circumference, Constants.chassisConstants.turnMotorGearRat);
      Rotation2d angle = getTurnAngle();
      return new SwerveModuleState(velocity, angle);
    }
    public SwerveModulePosition getPosition(){
      return new SwerveModulePosition(falconToMeters(driveMotor.getEncoder().getPosition(), chassisConstants.circumference, chassisConstants.driveMotorGearRat), getTurnAngle());
    }
  
    public void setDesiredState(SwerveModuleState desiredState){
      desiredState = SwerveModuleState.optimize(desiredState, getTurnAngle());
  
     // double velocity = MPSToFalcons(desiredState.speedMetersPerSecond, Constants.chassisConstants.circumference, Constants.chassisConstants.driveMotorGearRat);
      //double turnOutput = turnPIDController.calculate(turnAngleRadians(), desiredState.angle.getRadians());
      driveMotor.set(desiredState.speedMetersPerSecond/5);
      long nearestDegree = Math.round(desiredState.angle.getDegrees());

      double setTurnValue = (2048 / 360) * nearestDegree;
      double inputAngle = nearestDegree;
      double setPoint = setTurnValue * chassisConstants.turnMotorGearRat;

      final PositionVoltage m_request = new PositionVoltage(setPoint);

// set position to 10 rotations
      turnMotor.setControl(m_request);
    }
    public void zeroModules(){
      //turnMotor.set(ControlMode.Position, 0);
      // while(Math.abs(absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffset) > 0.01){
      //   turnMotor.set(ControlMode.PercentOutput, 0.1);
      // }
      // turnMotor.set(ControlMode.PercentOutput, 0);

      resetEncoder();
    }
  
    public void stop(){
      driveMotor.set(0);
      final PositionVoltage m_request = new PositionVoltage(getState().angle.getDegrees() * chassisConstants.turnMotorGearRat);
      turnMotor.setControl(m_request);
    }
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
