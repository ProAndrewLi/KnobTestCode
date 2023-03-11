package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SolenoidTesting extends SubsystemBase {

  //PneumaticsModuleType.CTREPCM is for CTRE Pneumatics Control Module
  //PneumaticsModuleType.REVPH for REV Pneumatics Hub
  private final PneumaticsModuleType type = PneumaticsModuleType.REVPH;

  public DoubleSolenoid solenoidOne;
  public DoubleSolenoid solenoidTwo;
  public DoubleSolenoid solenoidThree;
  public DoubleSolenoid solenoidFour;

  public SolenoidTesting(){
      this.solenoidOne = new DoubleSolenoid(type, 0, 1);
      this.solenoidTwo = new DoubleSolenoid(type, 2, 3);
      this.solenoidThree = new DoubleSolenoid(type, 4, 5);
      this.solenoidFour = new DoubleSolenoid(type, 6, 7);
  }

  public void toggleSolenoid() {
      if (this.solenoidOne.get().equals(DoubleSolenoid.Value.kForward)) {
          retractSolenoid();
      } else {
          extendSolenoid();
      }
    }
  
    public void extendSolenoid() {
      this.solenoidOne.set(DoubleSolenoid.Value.kForward);
      this.solenoidTwo.set(DoubleSolenoid.Value.kForward);
      this.solenoidThree.set(DoubleSolenoid.Value.kForward);
      this.solenoidFour.set(DoubleSolenoid.Value.kForward);
    }
  
    public void retractSolenoid() {
      this.solenoidOne.set(DoubleSolenoid.Value.kReverse);
      this.solenoidTwo.set(DoubleSolenoid.Value.kReverse);
      this.solenoidThree.set(DoubleSolenoid.Value.kReverse);
      this.solenoidFour.set(DoubleSolenoid.Value.kReverse);
    }

}
