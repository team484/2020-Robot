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
import frc.robot.subsystems.DriveSub;

/**
 * Add your docs here.
 */
public class Vision {
    Vision instance = null;
    private static NetworkTableInstance inst;
    private static NetworkTable table;
    protected static double zeroFrameOldAngle = 0;
    protected static double oneFrameOldAngle = 0;
    protected static double[] targetTVec = {0,0,0};
    private static boolean isTarget = false;
    public Vision() {
        if (instance != null) return;
        Vision.inst = NetworkTableInstance.getDefault();
        Vision.inst.setUpdateRate(0.001);
        Vision.table = Vision.inst.getTable("vision");
        NetworkTableEntry entry = Vision.table.getEntry("tvec");

        entry.addListener(event -> {
            if (!Vision.isTarget()) {
                Vision.isTarget = false;
                return;
            }
            double[] eventVal = event.value.getDoubleArray();
            if (eventVal.length < 3) {
                Vision.isTarget = false;
                return;
            }
            double[] rotatedVec = eventVal;
            rotatedVec[1] = -rotatedVec[1];
            rotatedVec = Vision.getRotatedTVec(Math.toRadians(17), rotatedVec);
            Vision.targetTVec = rotatedVec;
            Vision.oneFrameOldAngle = Vision.zeroFrameOldAngle;
            Vision.zeroFrameOldAngle = DriveSub.getGyroAngle();
            Vision.isTarget = true;
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kNew | EntryListenerFlags.kImmediate);
    }

    public static double getAngluarDistanceToTarget() {
        double sqVal = 0;
        sqVal += Math.pow(Vision.targetTVec[0], 2);
        sqVal += Math.pow(Vision.targetTVec[1], 2);
        sqVal += Math.pow(Vision.targetTVec[2], 2);
        return Math.sqrt(sqVal);
    }

    public static double[] getRotatedTVec(double angle, double[] tvec) {
        double cosA = Math.cos(-angle);
        double sinA = Math.sin(-angle);
        double newY = tvec[1] * cosA - tvec[2]*sinA;
        double newZ = tvec[2] * cosA + tvec[1]*sinA;
        double[] result = {tvec[0], newY, newZ};
        return result;
    }

    public static double getAngle() {
        double frameAngle = 90+Math.toDegrees(Math.atan(-Vision.targetTVec[2]/Vision.targetTVec[0]));
        if (frameAngle > 90) {
            frameAngle -= 180.0;
        }
        double gyroAngleChange = DriveSub.getGyroAngle() - Vision.zeroFrameOldAngle;
        return frameAngle+gyroAngleChange;
    }

    public static boolean isTarget() {
        return Vision.table.getEntry("isTarget").getBoolean(false);
    }

    public static double[] getAngleDistance() {
        double frameAngle = 90+Math.toDegrees(Math.atan(-Vision.targetTVec[2]/Vision.targetTVec[0]));
        if (frameAngle > 90) {
            frameAngle -= 180.0;
        }
        double gyroAngleChange = DriveSub.getGyroAngle() - Vision.oneFrameOldAngle;
        double[] result = {frameAngle+gyroAngleChange, Vision.targetTVec[2]};
        return result;
    }


    public static boolean isTargetTracked() {
        return Vision.isTarget() && Vision.isTarget;
    }

}
