/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class RobotMap 
{
  /*****
   * Drivetrain instance variables
   *****/

  //Motor IDs
  private final int TALON_ONE_ID = 12;
  private final int TALON_TWO_ID = 14;
  private final int TALON_THREE_ID = 11; 
  private final int TALON_FOUR_ID = 8;

  private final int FALCON_ONE_ID = 1; //21
  private final int FALCON_TWO_ID = 2; //12
  private final int FALCON_THREE_ID = 11;

  //Motor Speed
  private final double motorSpeed = 0.3;
  
  //Joystick Port
  private final int JOYSTICK_PORT_ONE = 0; 
  

  /*****
   * Constructor methods
   *****/
  public RobotMap() {}

  /*****
   * Getter methods
   *****/
  public int getJoystickPortOne() {
    return this.JOYSTICK_PORT_ONE;
  }

  public int getTalonOneId() {
    return this.TALON_ONE_ID;
  }

  public int getTalonTwoId() {
    return this.TALON_TWO_ID;
  }

  public int getTalonThreeId() {
    return this.TALON_THREE_ID;
  }

  public int getTalonFourId() {
    return this.TALON_FOUR_ID;
  }

  public double getMotorSpeed() {
    return this.motorSpeed;
  }

  public int getFalconOneId() {
    return FALCON_ONE_ID;
  }

  public int getFalconTwoId() {
    return FALCON_TWO_ID;
  }

  public int getFalconThreeId() {
    return FALCON_THREE_ID;
  }

}
