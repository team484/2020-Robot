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
    // CAN IDs
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
    public static final int INTAKE_AND_CONTROL_PANEL_MOTOR_ID = 12;
    public static final int INTAKE_ARM_MOTOR_ID = 13;
    public static final int LEFT_ELEVATOR_MOTOR_ID = 14;
    public static final int RIGHT_ELEVATOR_MOTOR_ID = 15;
    public static final int CAN_TIMEOUT_INTERVAL = 50;


    // Digital IO
    public static final int HORIZONTAL_BALL_SENSOR_PORT = 0;
    public static final int VERTICAL_BALL_SENSOR_PORT = 1;
    public static final int LEFT_ENCODER_A_PORT = 3;
    public static final int LEFT_ENCODER_B_PORT = 4;
    public static final int RIGHT_ENCODER_A_PORT = 7;
    public static final int RIGHT_ENCODER_B_PORT = 8;


    // PWM IO
    public static final int CLUTCH_SERVO_PORT = 0;
    public static final int LED_PORT = 2;

    // Relay IO
    public static final int VISION_LED_PORT = 0;

    // Control Panel Spinner Subsystem
    public static final double CONTROL_PANEL_MOTOR_SPEED = 0.5;


    //Drivetrain Subsystem
    public static final double LEFT_ENCODER_DPP = -0.000243519452; //meters per pulse
    public static final double RIGHT_ENCODER_DPP = -LEFT_ENCODER_DPP;
    public static final double JOYSTICK_ROTATION_MULTIPLIER = 0.7; //Set to <1 to make robot turn slower
    public static final double DRIVEBASE_WIDTH = 0.60325; //meters
    public static final double DRIVE_VOLTAGE_COMPENSATION_TARGET = 11.0;
        //Drivetrain characterization
    public static final double DRIVE_KS = 0.326; //volts
    public static final double DRIVE_KV = 1.52; //volt seconds per meter
    public static final double DRIVE_KA = 0.0; //volt seconds^2 per meter

    public static final double DRIVE_KP = 0.0;

    public static final double DRIVE_MAX_SPEED = 1.5; // meters per second
    public static final double DRIVE_MAX_ACCEL = 1.5; // meters/second^2

    public static final double DRIVE_RAMSETE_B = 3;
    public static final double DRIVE_RAMSETE_Z = 0.7;

    public static final double DRIVE_ROTATE_KP = 0.1;
    public static final double DRIVE_ROTATE_KI = 0.5;
    public static final double DRIVE_ROTATE_KD = 0.01;


    //Elevator Subsystem
    public static double CLUTCH_CLOSE_ANGLE = 0;
    public static double CLUTCH_OPEN_ANGLE = 0.5;
    public static double ELEVATOR_MAX_HEIGHT = 66.0;
    public static double ELEVATOR_DPP = 1; //Encoder DPP
    public static int ELEVATOR_MAX_CURRENT = 50; //Amps
    

    // Horizontal Conveyor Subsystem
    public static boolean HORIZONTAL_SENSOR_NORMAL_STATE = false; //the state of the sensor when no ball is present

    // Intake Arm Subsystem
    public static final double INTAKE_UP_SETPOINT = 0;
    public static final double INTAKE_MID_SETPOINT = -20;
    public static final double INTAKE_DOWN_SETPOINT = -46;
    public static final double INTAKE_KP = 0.05;
    public static final double INTAKE_KI = 0.0;
    public static final double INTAKE_KD = 0.04;
    public static final double INTAKE_ARM_ANGLE_ERROR = 1;
    public static final double INTAKE_ARM_VELOCITY_ERROR = 1;
    public static final int INTAKE_ARM_MAX_CURRENT = 7; //amps
    public static final double INTAKE_ARM_VERT_HOLD_POWER = 0.03;
    public static final double INTAKE_ARM_HORIZ_HOLD_POWER = -0.03;

    // Intake Wheel Subsystem
    public static final double INTAKE_WHEELS_MOTOR_SPEED = 1.0;

    // Shooter subsystem
    public static final double SHOOTER_TARGET_RPM = 13300; //13300 for far 11500 for close 8000 for lob
    public static final double SHOOTER_KP = 10.0; //100
    public static final double SHOOTER_KI = 0.0; //0.06
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KF = 29;//20
    public static final int SHOOTER_IZ = 4;
    public static final int SHOOTER_SLOT = 0;
    public static final double ALLOWABLE_ERROR = 250; //250 for far 700 for close
    public static final int SHOOTER_MAX_PEAK_CURRENT = 70;
    public static final int SHOOTER_MAX_CONTINUOUS_CURRENT = 50;
    
    // Vertical Conveyor Subsystem
    public static final double VERTICAL_CONVEYOR_SPEED = 0.5;
    public static boolean VERTICAL_SENSOR_NORMAL_STATE = false; //the state of the sensor when no ball is present


    // Joysticks
    public static final int DRIVE_STICK_PORT = 0;
    public static final int OP_STICK_PORT = 1;
        //Driver Stick Buttons
        public static final int ELEVATOR_CONTROLS_BUTTON = 2;
        public static final int OPEN_CLUTCH_BUTTON = 6;
        public static final int DRIVER_AIM_BUTTON = 1;

        //Operator Stick Buttons
        public static final int BALL_SHOOTER_BUTTON = 1;
        public static final int PICKUP_BALL_BUTTON = 2;
        public static final int INTAKE_UPPER_LEVEL_BUTTON = 11;
        public static final int INTAKE_MIDDLE_LEVEL_BUTTON = 9;
        public static final int INTAKE_LOWER_LEVEL_BUTTON = 7;
        public static final int INTAKE_WHEELS_AND_CONTROL_SPIN_BUTTON = 8;
        public static final int POSITION_CONTROL_BUTTON = 10;
        public static final int ROTATION_CONTROL_BUTTON = 12;  
        public static final int EJECT_BUTTON = 3;
        public static final int SPIT_BUTTON = 5;
}
