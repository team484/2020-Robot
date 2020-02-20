/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSub extends SubsystemBase {
  int FRONT_LED_LENGTH = 236;
  AddressableLED frontLEDs = new AddressableLED(2);

  private static final int SEGMENT_1_END = 59;
  private static final int SECGMENT_2_END = 118;
  private static final int SECGMENT_3_END = 177;
  /**
   * Creates a new LEDSub.
   */
  public LEDSub() {
    frontLEDs.setLength(FRONT_LED_LENGTH);
  }

  int i = 0;
  int hue = 0;
  @Override
  public void periodic() {
    if (DriverStation.getInstance().isDisabled()) { //Disabled
      setLEDs(Double.valueOf(i)/Double.valueOf(SEGMENT_1_END), hue, 255);

    } else if (ShooterSub.isActive()) { // Shooting
      double percentRPM = ShooterSub.getAveragedSpeed()/ShooterSub.getDesiredRPM();
      if (percentRPM < 0.9) {
        setLEDs(percentRPM, 0, 255); //red when <90% of desired RPM
      } else {
        setLEDs(percentRPM, 60, 255); //green when >90% of desired RPM
      }
    } else { // Match time remaining
      double timeLeft = DriverStation.getInstance().getMatchTime();
      if (timeLeft > 28 && timeLeft < 30) { //Flash yellow when there is 30 seconds left
        if ((int) (timeLeft*3) % 2 == 0) {
          setLEDs((135.0-timeLeft)/135.0,30,255);
        } else {
          setLEDs((135.0-timeLeft)/135.0,30,0);
        }
      } else if (timeLeft < 10 && timeLeft > 0) { // Flash when there is <10 seconds left
        setLEDs((135.0-timeLeft)/135.0,120,(int) (timeLeft*2) % 2 == 0 ? 255 : 0);
      } else { // Solid color normally
        setLEDs((135.0-timeLeft)/135.0,120,255);
      }
    }
    hue++;
    if (hue >= 180) {
      hue = 0;
    }
    i++;
    if (i >= SEGMENT_1_END) {
      i = 0;
    }
    
  }

  public void setLEDs(double height, int hue, int brightness) {
    AddressableLEDBuffer frontBuffer = new AddressableLEDBuffer(FRONT_LED_LENGTH);
    int i = (int) (height * Double.valueOf(SEGMENT_1_END));
    for (int j = 0; j < i; j++) {
      frontBuffer.setHSV(j, hue, 255, brightness);
      frontBuffer.setHSV(SECGMENT_2_END-j-5, hue, 255, brightness);
      frontBuffer.setHSV(SECGMENT_2_END+j+1, hue, 255, brightness);
      frontBuffer.setHSV(FRONT_LED_LENGTH-j-2, hue, 255, brightness);
    }
    frontBuffer.setHSV(SECGMENT_2_END, hue, 255, brightness);
    frontBuffer.setHSV(SECGMENT_2_END-1, hue, 255, brightness);
    frontBuffer.setHSV(SECGMENT_2_END-2, hue, 255, brightness);
    frontBuffer.setHSV(SECGMENT_2_END-3, hue, 255, brightness);
    frontBuffer.setHSV(SECGMENT_2_END-4, hue, 255, brightness);
    frontBuffer.setHSV(FRONT_LED_LENGTH-1, hue, 255, brightness);
    frontBuffer.setHSV(SECGMENT_2_END, hue, 255, brightness);


    frontLEDs.setData(frontBuffer);
    frontLEDs.start();
  }
}
