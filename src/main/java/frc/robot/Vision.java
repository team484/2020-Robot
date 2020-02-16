/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSub;

/**
 * Add your docs here.
 */
public class Vision {
    Vision instance = null;
    private static NetworkTableInstance inst;
    private static NetworkTable table;
    protected static double oneFrameOldAngle = 0;
    protected static double zeroFrameOldAngle = 0;
    protected static double[] targetTVec = {0,0,0};
    public Vision() {
        if (instance != null) return;
        Vision.inst = NetworkTableInstance.getDefault();
        Vision.inst.setUpdateRate(0.001);
        Vision.table = Vision.inst.getTable("vision");
        NetworkTableEntry entry = Vision.table.getEntry("tvec");

        entry.addListener(event -> {
            double[] eventVal = event.value.getDoubleArray();
            if (eventVal.length < 3) return;
            Vision.targetTVec = eventVal;
            Vision.targetTVec[1] = -Vision.targetTVec[1];
            Vision.oneFrameOldAngle = Vision.zeroFrameOldAngle;
            Vision.zeroFrameOldAngle = DriveSub.getGyroAngle();
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kNew | EntryListenerFlags.kImmediate);
    }

    public static double getDistanceToTarget() {
        double sqVal = 0;
        sqVal += Math.pow(Vision.targetTVec[0], 2);
        sqVal += Math.pow(Vision.targetTVec[1], 2);
        sqVal += Math.pow(Vision.targetTVec[2], 2);
        return Math.sqrt(sqVal);
    }

    public static double[] getRotatedTVec(double angle) {
        double cosA = Math.cos(-angle);
        double sinA = Math.sin(-angle);
        double newY = Vision.targetTVec[1] * cosA - Vision.targetTVec[2]*sinA;
        double newZ = Vision.targetTVec[2] * cosA + Vision.targetTVec[1]*sinA;
        double[] result = {Vision.targetTVec[0], newY, newZ};
        return result;
    }

    public static double getAngle() {
        double[] tVec = getRotatedTVec(Math.toRadians(17));
        double frameAngle = 90+Math.toDegrees(Math.atan(-tVec[2]/tVec[0]));
        if (frameAngle > 90) {
            frameAngle -= 180.0;
        }
        double gyroAngleChange = DriveSub.getGyroAngle() - Vision.oneFrameOldAngle;
        return frameAngle+gyroAngleChange;
    }

    public static boolean isTarget() {
        return Vision.table.getEntry("isTarget").getBoolean(false);
    }

    public static double[] getAngleDistance() {
        double[] tVec = getRotatedTVec(Math.toRadians(17));
        double frameAngle = 90+Math.toDegrees(Math.atan(-tVec[2]/tVec[0]));
        if (frameAngle > 90) {
            frameAngle -= 180.0;
        }
        double gyroAngleChange = DriveSub.getGyroAngle() - Vision.oneFrameOldAngle;
        double[] result = {frameAngle+gyroAngleChange, tVec[2]};
        return result;
    }

}
