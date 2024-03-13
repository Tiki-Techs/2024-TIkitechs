package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry dis = table.getEntry("targetpose_cameraspace");
    public static double distance;
    public static double angle;

    @Override
    public void periodic() {
        distance = dis.getDoubleArray(new double[5])[2];
        
        SmartDashboard.putNumber("Distance", distance);
            angle = table.getEntry("tx").getDouble(0);
        
        SmartDashboard.putNumber("tx", angle);
        // if(getTag(0)){

        // }
    }

}
