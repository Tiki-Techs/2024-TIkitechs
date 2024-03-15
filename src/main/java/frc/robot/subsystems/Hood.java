package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.commands.swervedrive.HoodPositioner;

public class Hood extends SubsystemBase {

    public DutyCycleEncoder rotEncoder = new DutyCycleEncoder(HoodConstants.EncoderID);
    public PIDController hoodPID = new PIDController(0.002, 0, 0);

    public double kP = 0.01;

    public Hood() {
        SmartDashboard.putNumber("Hood Setpoint", 65);
    }

    public double getRotation() {
        return Rotation2d.fromRotations(rotEncoder.getAbsolutePosition()).getDegrees() - HoodConstants.encoderOffset;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood PID", hoodPID.calculate(getRotation(), HoodPositioner.setpoint));
        SmartDashboard.putNumber("Hood Setpoint", HoodPositioner.setpoint);
        SmartDashboard.putNumber("Hood Angle", getRotation());
        
    }

}
