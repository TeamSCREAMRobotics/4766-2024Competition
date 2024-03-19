   // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  //declares motors
  TalonFX driveIntakeMotor = new TalonFX(IntakeConstants.intakeMasterID);
  TalonFX followerIntakeMotor = new TalonFX(IntakeConstants.intakeFollowerID);
  final VoltageOut m_request = new VoltageOut(0);
  final DutyCycleOut m_cycle = new DutyCycleOut(0);

  
  /** Creates a new Intake. */
  public Intake() {
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveIntakeMotor.getConfigurator().apply(intakeConfig);
    followerIntakeMotor.getConfigurator().apply(intakeConfig);
    followerIntakeMotor.setControl(new Follower(driveIntakeMotor.getDeviceID(), false));
  }

  //runs intake forward with a set voltage
  public void runIntake(double output){
    followerIntakeMotor.setControl(new Follower(driveIntakeMotor.getDeviceID(), false));
    driveIntakeMotor.setControl(m_request.withOutput(output));
  }

  //runs intake backwards in case pieces get stuck
  public void Outtake(){
    driveIntakeMotor.setControl(m_request.withOutput(IntakeConstants.outtakeOutput));
  }

  public void randOut(Double percentOut){
    driveIntakeMotor.setControl(m_cycle.withOutput(percentOut));
  }

  //stops intake
  public void resetIntake(){
    driveIntakeMotor.setControl(m_request.withOutput(0));
  }

  

  
}
