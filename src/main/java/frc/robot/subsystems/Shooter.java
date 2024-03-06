// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  //declaring devices and making sure intake loaded is true (one loaded prior to starting match)
  TalonFX shooterConveyor = new TalonFX(ShooterConstants.conveyorID);
  public TalonFX shooterMaster = new TalonFX(ShooterConstants.shooterMasterID);
  TalonFX shooterFollower = new TalonFX(ShooterConstants.shooterFollowerID);
  DigitalInput beamBreak = new DigitalInput(ShooterConstants.beamBreakChannelID);
  boolean intakeLoaded = true;
  double shooterLowerVelocity = ShooterConstants.shooterLowerVelocity;
  final VoltageOut m_request = new VoltageOut(0);
  final DutyCycleOut m_cycle = new DutyCycleOut(0);
  
  public double shooterMaxVelocity = ShooterConstants.shooterMaxVelocity;
public Object shooterLower;
  
  /** Creates a new Shooter. */
  public Shooter() {
    Slot0Configs shooterConfigs = new Slot0Configs();
    shooterConfigs.kP = 60;
    shooterMaster.getConfigurator().apply(shooterConfigs);
    shooterFollower.getConfigurator().apply(shooterConfigs);
    TalonFXConfiguration conveyorConfig = new TalonFXConfiguration();
    conveyorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterFollower.setControl(new Follower(shooterMaster.getDeviceID(), false));
  }

  //runs the conveyor forward using a set voltage
  public void runConveyor(double output){
    shooterConveyor.setControl(m_request.withOutput(output));
  }

  //runs the conveyor backwards in case pieces get stuck or need to come out backwards
  public void Outtake(){
    
    shooterConveyor.setControl(m_cycle.withOutput(-0.5));
  }

  public void randOut(double percentOut){
    shooterConveyor.setControl(m_cycle.withOutput(percentOut));
  }

  //gets current velocity of shooter to check if up to speed
  public double getShooterVelocity(){
   return shooterMaster.getVelocity().getValueAsDouble();
  }

  //runs shooter
  public void shoot(double voltage){
    shooterMaster.setControl(m_request.withOutput(voltage));
    }

  public void shootSlow(){
    shooterMaster.setControl(m_request.withOutput(shooterLowerVelocity));
  }

  //checks to see if beam break is triggered
  public boolean beamBreakTriggered(){
    return !beamBreak.get();
  }
  

  //stops conveyor
  public void resetConveyor(){
    shooterConveyor.setControl(m_request.withOutput(0));
  }

  //stops shooter
  public void resetShooter(){
    shooterMaster.setControl(m_request.withOutput(0));
  }

  //tells robot the intake has a note in it
  public void setIntakeLoaded(boolean isNote){
    intakeLoaded = isNote;
  }

  //checks to see if intake is loaded
  public boolean getIntakeLoaded(){
    return intakeLoaded;
  }

  public boolean shooterAtVelocity(){
    return getShooterVelocity() == shooterMaxVelocity;
  }
}
