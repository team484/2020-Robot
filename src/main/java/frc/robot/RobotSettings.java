/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotSettings {
    // Motor IDs
    public static final int LEFT_MOTOR_1_ID = 1;
    public static final int LEFT_MOTOR_2_ID = 2;
    public static final int LEFT_MOTOR_3_ID = 3;
    
    public static final int RIGHT_MOTOR_1_ID = 4;
    public static final int RIGHT_MOTOR_2_ID = 5;
    public static final int RIGHT_MOTOR_3_ID = 6;
    
    public static final int SHOOTER_MOTOR_1_ID = 7;
    public static final int SHOOTER_MOTOR_2_ID = 8;
    
    public static final int HORIZONTAL_BALL_CONVEYOR_MOTOR_ID = 9;
    public static final int VERTICAL_BALL_CONVEYOR_MOTOR_ID = 10;
    
    public static final int CLIMBER_WHEELS_MOTOR_ID = 11;
    
    public static final int INTAKE_AND_CONTROL_PANEL_MOTOR_ID = 12;
    
    public static final int INTAKE_ARM_MOTOR_ID = 13;
    
    public static final int LEFT_ELEVATOR_MOTOR_ID = 14;
    public static final int RIGHT_ELEVATOR_MOTOR_ID = 15;

    public static final int CLUTCH_SERVO_PORT = 0;

    // Angles for engaging and disengaging climber clutch. Make final when assigning a value.
    public static int CLUTCH_DISENGAGE_ANGLE = 90;
    public static int CLUTCH_ENGAGE_ANGLE = 0;
    public static boolean CLUTCH_EGAGED = false;
    
    public static final double VERTICAL_CONVEYOR_SPEED = 1.0;

    public static final int DRIVE_STICK_PORT = 1;
    public static final int OP_STICK_PORT = 2;

    public static final int RIGHT_ENCODER_A_PORT = 0;
    public static final int RIGHT_ENCODER_B_PORT = 1;
    public static final int LEFT_ENCODER_A_PORT = 2;
    public static final int LEFT_ENCODER_B_PORT = 3;
    

    public static final int INTAKE_UP_SETPOINT = -90;
    public static final int INTAKE_MID_SETPOINT = 45;
    public static final int INTAKE_DOWN_SETPOINT = 0;
    public static final double INTAKE_KP = 1.0;
    public static final double INTAKE_KI = 1.0;
    public static final double INTAKE_KD = 1.0;

    //Driver Stick Buttons
    public static final int ELEVATOR_CONTROLS_BUTTON_ID = 1;

    //Operator Stick Buttons
    public static final int BALL_SHOOTER_BUTTON_ID = 1;
    public static final int MANUAL_INTAKE_LOWER_BUTTON_ID = 3;
    public static final int MANUAL_INTAKE_RAISE_BUTTON_ID = 5;
    public static final int SHOOTER_AIM_DOWN_BUTTON_ID = 4;
    public static final int SHOOTER_AIM_UP_BUTTON_ID = 6;
    public static final int INTAKE_UPPER_LEVEL_BUTTON_ID = 7;
    public static final int INTAKE_MIDDLE_LEVEL_BUTTON_ID = 9;
    public static final int INTAKE_LOWER_LEVEL_BUTTON_ID = 11;
    public static final int INTAKE_WHEELS_AND_CONTROL_SPIN_BUTTON_ID = 8;
    public static final int POSITION_CONTROL_BUTTON_ID = 10;
    public static final int ROTATION_CONTROL_BUTTON_ID = 12;

    //Motor Values
    public static final double INTAKE_WHEELS_MOTOR_SPEED = 1.0;
    public static final double CONTROL_PANEL_MOTOR_SPEED = 1.0;
    public static int ELEVATOR_MAX_HEIGHT;
}
