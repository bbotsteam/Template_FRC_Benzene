package team4384.robot;

import com.ctre.phoenix.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Joystick {
    //use gyroscope

    private final edu.wpi.first.wpilibj.Joystick Xbox = new edu.wpi.first.wpilibj.Joystick(0);
    private final TalonFX motor = new TalonFX(0);
    //I want to square the joy stick imputs so first i have to get the imputs
    // by  private int CAN_ID_1 = 0;getRightTriggerAxis() And then i have to
    //square the values so so the motor has a more gradual speed change. It gets faster
}

public void main(double speed){
    motor.setInverted(false);
    motor.set(TalonFXControlMode.PercentOutput, speed);

}

public double getLeftStickAxis(){
    getLeftStickAxis = speed;
    speed = new Math.pow(speed, speed);
    System.out.println("speed:" + speed);
    return;
}