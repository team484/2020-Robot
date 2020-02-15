/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.commands.controlpanel.ControlPanelSpinDoNothing;

/**
 * Add your docs here.
 */
public class ControlPanelSpinnerSub extends SubsystemBase {

    private static final ColorMatch colorMatcher = new ColorMatch();
    private static final Color blueTri = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private static final Color greenTri = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private static final Color redTri = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private static final Color yellowTri = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public ControlPanelSpinnerSub() {
        setDefaultCommand(new ControlPanelSpinDoNothing(this));
        colorMatcher.addColorMatch(redTri);
        colorMatcher.addColorMatch(greenTri);
        colorMatcher.addColorMatch(blueTri);
        colorMatcher.addColorMatch(yellowTri);
    }

    public static void set(double speed){
        RobotIO.intakeAndcontrolDoubleSpeedController.set2(speed);
    }
    public enum WheelColor {
        RED, GREEN, BLUE, YELLOW, UNKNOWN;
    }
    public static WheelColor getColor() {
        ColorMatchResult match = colorMatcher.matchClosestColor(RobotIO.colorSensor.getColor());
        if (match.confidence < 0.5) {
            return WheelColor.UNKNOWN;
        }
       
        if (match.color == blueTri) {
            return WheelColor.RED;
        }
        if (match.color == greenTri) {
            return WheelColor.YELLOW;
        }
        if (match.color == redTri) {
            return WheelColor.BLUE;
        }
        if (match.color == yellowTri) {
            return WheelColor.GREEN;
        }
        return WheelColor.UNKNOWN; }
    }
