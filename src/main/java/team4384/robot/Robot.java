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

        Joystick Xbox = new Joystick(0); // 0 is the USB Port to be used as indicated on the Driver Station
        JoystickButton X = new JoystickButton(Xbox, 1);
        JoystickButton Y = new JoystickButton(Xbox, 2);

        TalonFX motor = new TalonFX(0);

        if (X.getAsBoolean() == true){
            motor.setInverted(false);
            motor.set(TalonFXControlMode.PercentOutput, 10.0);
            System.out.println("Motor is spinning");
    
        } else {
            motor.set(TalonFXControlMode.PercentOutput, 0);
            System.out.println("Motor is not spinning");
        }

        if (Y.getAsBoolean() == true) {
            motor.setInverted(true);
            motor.set(TalonFXControlMode.PercentOutput, 10.0);
            System.out.println("Motor is spinning");
        } else {
            motor.set(TalonFXControlMode.PercentOutput, 0);
            System.out.println("Motor is not spinning");
        }
    }
}
