package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

    DutyCycleEncoder rotEncoder = new DutyCycleEncoder(HoodConstants.EncoderID);
    public PIDController hoodPID = new PIDController(0.002, 0, 0);

    public double kP = 0.01;

    public Hood() {
    }

    public double getRotation() {
        return Rotation2d.fromRotations(rotEncoder.getAbsolutePosition()).getDegrees() - HoodConstants.encoderOffset;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood PID", hoodPID.calculate(getRotation(), 35));
        SmartDashboard.putNumber("Hood Angle", getRotation());
    }

}
