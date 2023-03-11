package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class MotorTesting extends SubsystemBase {

    WPI_TalonSRX TALON_ONE;
    WPI_TalonSRX TALON_TWO;
    WPI_TalonSRX TALON_THREE;
    WPI_TalonSRX TALON_FOUR;

    WPI_TalonFX FALCON_ONE;
    WPI_TalonFX FALCON_TWO;
    WPI_TalonFX FALCON_THREE;

    public MotorTesting() {
        TALON_ONE = new WPI_TalonSRX(Robot.ROBOTMAP.getTalonOneId());
        TALON_TWO = new WPI_TalonSRX(Robot.ROBOTMAP.getTalonTwoId());
        TALON_THREE = new WPI_TalonSRX(Robot.ROBOTMAP.getTalonThreeId());
        TALON_FOUR = new WPI_TalonSRX(Robot.ROBOTMAP.getTalonFourId());

        FALCON_ONE = new WPI_TalonFX(Robot.ROBOTMAP.getFalconOneId());
        FALCON_TWO = new WPI_TalonFX(Robot.ROBOTMAP.getFalconTwoId());
        FALCON_THREE = new WPI_TalonFX(Robot.ROBOTMAP.getFalconThreeId());

        configureMotors();
    }

    public void first(boolean isForward) {
        if(isForward) {
            TALON_ONE.set(Robot.ROBOTMAP.getMotorSpeed());
        } else {
            TALON_ONE.set(-Robot.ROBOTMAP.getMotorSpeed());
        }
    }

    public void second(boolean isForward) {
        if(isForward) {
            TALON_TWO.set(Robot.ROBOTMAP.getMotorSpeed());
        } else {
            TALON_TWO.set(-Robot.ROBOTMAP.getMotorSpeed());
        }
    }

    public void third(boolean isForward) {
        if(isForward) {
            TALON_THREE.set(Robot.ROBOTMAP.getMotorSpeed());
        } else {
            TALON_THREE.set(-Robot.ROBOTMAP.getMotorSpeed());
        }
    } 

    public void fourth(boolean isForward) {
        if(isForward) {
            TALON_FOUR.set(Robot.ROBOTMAP.getMotorSpeed());
        } else {
            TALON_FOUR.set(-Robot.ROBOTMAP.getMotorSpeed());
        }
    }

    public void fifth(boolean isForward) {
        if(isForward) {
            FALCON_ONE.set(Robot.ROBOTMAP.getMotorSpeed());
        } else {
            FALCON_ONE.set(-Robot.ROBOTMAP.getMotorSpeed());
        }
    }
    
    public void sixth(boolean isForward) {
        if(isForward) {
            FALCON_TWO.set(Robot.ROBOTMAP.getMotorSpeed());
        } else {
            FALCON_TWO.set(-Robot.ROBOTMAP.getMotorSpeed());
        }
    }

    public void seventh(boolean isForward) {
        if(isForward) {
            FALCON_THREE.set(Robot.ROBOTMAP.getMotorSpeed());
        } else {
            FALCON_THREE.set(-Robot.ROBOTMAP.getMotorSpeed());
        }
    }

    public void configureMotors() {
        TALON_ONE.configFactoryDefault();
        TALON_TWO.configFactoryDefault();
        TALON_THREE.configFactoryDefault();
        TALON_FOUR.configFactoryDefault();
        FALCON_ONE.configFactoryDefault();
        FALCON_TWO.configFactoryDefault();
        FALCON_THREE.configFactoryDefault();

        TALON_ONE.setNeutralMode(NeutralMode.Coast);
        TALON_TWO.setNeutralMode(NeutralMode.Coast);
        TALON_THREE.setNeutralMode(NeutralMode.Coast);
        TALON_FOUR.setNeutralMode(NeutralMode.Coast);
        FALCON_ONE.setNeutralMode(NeutralMode.Brake);
        FALCON_TWO.setNeutralMode(NeutralMode.Brake);
        FALCON_THREE.setNeutralMode(NeutralMode.Coast);
    }

    public void stop() {
        TALON_ONE.stopMotor();
        TALON_TWO.stopMotor();
        TALON_THREE.stopMotor();
        TALON_FOUR.stopMotor();
        FALCON_ONE.stopMotor();
        FALCON_TWO.stopMotor();
        FALCON_THREE.stopMotor();
    }
}
