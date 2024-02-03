package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Conversions;

public class SwerveModule {
        public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private CANSparkMax mDriveMotor;
    private DutyCycleEncoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.chassisConstants.kS, Constants.chassisConstants.kV, Constants.chassisConstants.kA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();

    public SwerveModule(int moduleNumber, Rotation2d angleOffset, int AbsEncoderID, int angleMotorID, int driveMotorID){
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new DutyCycleEncoder(AbsEncoderID);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(angleMotorID);
        
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    }

    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState){
        mDriveMotor.set(desiredState.speedMetersPerSecond/5);
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getEncoder().getVelocity(), Constants.chassisConstants.circumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getEncoder().getVelocity(), Constants.chassisConstants.circumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

}
