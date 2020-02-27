/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.Vision;

public class LEDSub extends SubsystemBase {
  int FRONT_LED_LENGTH = 236;

  private static final int SEGMENT_1_END = 59;
  private static final int SECGMENT_2_END = 118;
  // Wasn't being used, but didn't want to delete
  //private static final int SECGMENT_3_END = 177;
  /**
   * Creates a new LEDSub.
   */
  public LEDSub() {
    RobotIO.leds.setLength(FRONT_LED_LENGTH);
    RobotIO.leds.start();
  }

  int i = 0;
  int hue = 0;
  @Override
  public void periodic() {
    hue++;
    if (hue >= 180) {
      hue = 0;
    }
    i++;
    if (i >= SEGMENT_1_END) {
      i = 0;
    }
    int insideHue = hue;
    int outsideHue = hue;
    double insideHeight = Double.valueOf(i)/Double.valueOf(SEGMENT_1_END);
    double outsideHeight = insideHeight;


    // CELEBRATE
    if (RobotIO.driveStick.getRawButton(8)) {
      setLEDs(1, (insideHue * 15) % 180);
      return;
    }


    if (DriverStation.getInstance().isDisabled()) { //Disabled
      setLEDs(outsideHeight, insideHeight, outsideHue, insideHue);
      return;
    }

    LedStruct insideStatus = getInsideStatus(i);
    LedStruct outsideStatus = getOutsideStatus(i);

    setLEDs(outsideStatus.height, insideStatus.height, outsideStatus.hue, insideStatus.hue);

    
  }

  public void setLEDs(double height, int hue) {
    setLEDs(height, height, hue, hue);
  }
  public void setLEDs(double heightOutside, double heightInside, int hueOutside, int hueInside) {
    int brightness = 255;
    AddressableLEDBuffer frontBuffer = new AddressableLEDBuffer(FRONT_LED_LENGTH);
    int i_inside = (int) (heightInside * Double.valueOf(SEGMENT_1_END));
    int i_outside = (int) (heightOutside * Double.valueOf(SEGMENT_1_END));
    for (int j = 0; j < i_inside; j++) {
      frontBuffer.setHSV(SECGMENT_2_END-j-5, hueInside, 255, brightness);
      frontBuffer.setHSV(SECGMENT_2_END+j+1, hueInside, 255, brightness);
    }
    for (int j = 0; j < i_outside; j++) {
      frontBuffer.setHSV(j, hueOutside, 255, brightness);
      frontBuffer.setHSV(FRONT_LED_LENGTH-j-2, hueOutside, 255, brightness);
    }
    frontBuffer.setHSV(SECGMENT_2_END, hueInside, 255, brightness);
    frontBuffer.setHSV(SECGMENT_2_END-1, hueInside, 255, brightness);
    frontBuffer.setHSV(SECGMENT_2_END-2, hueInside, 255, brightness);
    frontBuffer.setHSV(SECGMENT_2_END-3, hueInside, 255, brightness);
    frontBuffer.setHSV(SECGMENT_2_END-4, hueInside, 255, brightness);
    frontBuffer.setHSV(FRONT_LED_LENGTH-1, hueOutside, 255, brightness);
    frontBuffer.setHSV(SECGMENT_2_END, hueInside, 255, brightness);


    RobotIO.leds.setData(frontBuffer);
  }



  private LedStruct getInsideStatus(int i) {
    //-----Flash for 30 seconds left-----//
    double timeLeft = DriverStation.getInstance().getMatchTime();
    if (timeLeft > 28 && timeLeft < 30) { //Flash yellow when there is 30 seconds left
      if ((int) (timeLeft*3) % 2 == 0) {
        return new LedStruct((135.0-timeLeft)/135.0,30);
      } else {
        return new LedStruct(0,30);
      }
    }



    //-----Give Shooter RPM-----//
    if (ShooterSub.isActive()) {
      if (!Vision.isTargetTracked()) { //Flash LEDs if cannot find vision target
        if ((int) (timeLeft*3) % 2 == 0 || !HorizontalConveyorSub.isBallPresent()) {
          return new LedStruct(1.0,0);
        } else {
          return new LedStruct(0,0);
        }
      }
      double percentRPM = ShooterSub.getAveragedSpeed()/ShooterSub.getDesiredRPM();
      if (percentRPM < 0.9) {
        return new LedStruct(percentRPM, 0); //red when <90% of desired RPM
      }
      return new LedStruct(percentRPM, 60); //green when >90% of desired RPM
    }



    //-----Bringing balls up elevator-----//
    if (RobotIO.ballConveyerVertical.get() > 0) {
      double height = Double.valueOf(i)/Double.valueOf(SEGMENT_1_END);
      return new LedStruct(height, 160);
    } else if (RobotIO.ballConveyerVertical.get() < 0) {
      double height = 1.0-Double.valueOf(i)/Double.valueOf(SEGMENT_1_END);
      return new LedStruct(height, 160);
    }



    //-----Can't bring ball up elevator, it's full-----//
    if (VerticalConveyer.isBallPresent() && 
    (RobotIO.operatorStick.getRawButton(RobotSettings.PICKUP_BALL_BUTTON)
    || RobotIO.operatorStick.getRawButton(RobotSettings.INTAKE_WHEELS_AND_CONTROL_SPIN_BUTTON))) {
      if ((int) (timeLeft*4) % 2 == 0 || !HorizontalConveyorSub.isBallPresent()) {
        return new LedStruct(1.0,140);
      } else {
        return new LedStruct(0,140);
      }
    }



    // If there's nothing more important, show match time
    if (timeLeft < 10 && timeLeft > 0) { // Flash when there is <10 seconds left
      if ((int) (timeLeft*2) % 2 == 0) {
        return new LedStruct((135.0-timeLeft)/135.0,120);
      } else {
        return new LedStruct(0,120);
      }
    }
    return new LedStruct((135.0-timeLeft)/135.0,120);
  }





  private LedStruct getOutsideStatus(int i) {
    //-----Flash for 30 seconds left-----//
    double timeLeft = DriverStation.getInstance().getMatchTime();
    if (timeLeft > 28 && timeLeft < 30) { //Flash yellow when there is 30 seconds left
      if ((int) (timeLeft*3) % 2 == 0) {
        return new LedStruct((135.0-timeLeft)/135.0,30);
      } else {
        return new LedStruct(0,30);
      }
    }



    //-----Give Drivetrain aligned-----//
    if (Robot.driveSubVision) {
      if (!Vision.isTargetTracked()) { //Flash LEDs if cannot find vision target
        if ((int) (timeLeft*3) % 2 == 0) {
          return new LedStruct(1.0,0);
        } else {
          return new LedStruct(0,0);
        }
      }
      double height = Math.max(1- Math.abs(Vision.getAngle())/5.0, 0);
      
      if (height < 0.9) {
        return new LedStruct(height, 0); //red when <90% of desired RPM
      }
      return new LedStruct(height, 60); //green when >90% of desired RPM
    }

    if (ElevatorSub.elevatorError) {
      if ((i/5) % 2 == 0) {
        return new LedStruct(1.0,0);
      } else {
        return new LedStruct(0,0);
      }
    }

    // If there's nothing more important, show match time
    if (timeLeft < 10 && timeLeft > 0) { // Flash when there is <10 seconds left
      if ((int) (timeLeft*2) % 2 == 0) {
        return new LedStruct((135.0-timeLeft)/135.0,120);
      } else {
        return new LedStruct(0,120);
      }
    }
    return new LedStruct((135.0-timeLeft)/135.0,120);
  }

  class LedStruct {
    public double height;
    public int hue;
    public LedStruct(double height, int hue) {
      this.height=height;
      this.hue=hue;
    }
  }
}
