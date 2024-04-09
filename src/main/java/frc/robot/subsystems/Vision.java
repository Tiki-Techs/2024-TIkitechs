package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry dis = table.getEntry("targetpose_cameraspace");
    public static double distance;
    public static double angle;

    public Vision() {
    }

    @Override
    public void periodic() {
        distance = dis.getDoubleArray(new double[5])[2];

        SmartDashboard.putNumber("Distance", distance);
        angle = table.getEntry("tx").getDouble(0);

        SmartDashboard.putNumber("tx", angle);
        // CameraServer.startAutomaticCapture();

    }

    // if(getTag(0)){

    // }
}
