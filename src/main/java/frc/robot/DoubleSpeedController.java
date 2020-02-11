/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedController;

import java.lang.Math;

/**
 * Add your docs here.
 */
public class DoubleSpeedController {

    //two variables, one for each set method
    public static SpeedController speedController;
    public static double speedIntake = 0;
    public static double speedControlPanel = 0;
    // constructor that takes in the speed controller
    public DoubleSpeedController(SpeedController controller){
        speedController = controller;
    }

    //two set methods:

    //set 1 : takes in a speed, saves it to the variable, then compares abs(var1, var2) , sets the speed controller output to the largest one
    public static void set1(double speed){
        speedIntake = speed;
        if (Math.abs(speedIntake) > Math.abs(speedControlPanel)){
            speedController.set(speedIntake);
        }
        else{
            speedController.set(speedControlPanel);
        }
    }

    public void set2(double speed){
        speedControlPanel = speed;
        if (Math.abs(speedControlPanel) > Math.abs(speedIntake)){
            speedController.set(speedControlPanel);
        }
        else{
            speedController.set(speedIntake);
        }
    }

    //set 1: (double speed)
    //var1 = speed
    // if abs(var1) > abs(var2): speedController.set(var1)
    //else: speedc.set(var2)
}
