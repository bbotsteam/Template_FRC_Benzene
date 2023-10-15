// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    @Override
    public void robotInit() {
        boolean getXButtonPressed;
        private final
        private final edu.wpi.first.wpilibj.Joystick Xbox = new edu.wpi.first.wpilibj.Joystick(0); // 0 is the USB Port to be used as indicated on the Driver Station
       
        // private final WPI_TalonSRX motor = new WPI_TalonSRX(CAN_ID_1);
        private final TalonFX motor = new TalonFX(0);
    }
    public void main(){
        if (getXButtonPressed = true){
             // should i use getAButton to check the last time the A button was pressed
    
            /*motor.set(TalonSRXControlMode.Current, 10.0);
            motor.set(TalonSRXControlMode.PercentOutput, 75.0);
            motor.set(TalonSRXControlMode.Velocity, 75.0);
            motor.set(TalonSRXControlMode.Position, 75.0); */
            motor.set(TalonFXControlMode.PercentOutput, 100.0);
            System.out.println("Motor is spinning");
    
        }
                 else(getXButtonPressed = false){
                motor.stopMotor();
                System.out.println("Motor is not spinning");
             }
            
            if (getYButtonPressed = true){
                // could I just set the motor as motor.setInverted(true);
               /* motor.setInverted(TalonSRXControlMode.Current, 10.0);
                motor.setInverted(TalonSRXControlMode.PercentOutput, 75.0);
                motor.setInverted(TalonSRXControlMode.Velocity, 75.0);
                motor.setInverted(TalonSRXControlMode.Position, 75.0);*/
                motor.setInverted(TalonFXControlMode.PercentOutput, 100.0);
                System.out.println("Motor is spinning");
                
    
                else(getYButtonPressed = false)
                    motor.stopMotor();
                    System.out.println("Motor is not spinning");
            }
    
            
            if (getYButtonPressed && getXButtonPressed){
                motor.stopMotor();
            }
    }
}
