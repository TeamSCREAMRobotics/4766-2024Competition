// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  //initializes the motor for the climber

  public TalonFX climberMotor = new TalonFX(ClimberConstants.climberID);
   final VoltageOut m_request = new VoltageOut(0);
   public double climbPos;
   
  /** Creates a new Climber. */
  public Climber() {
 
    TalonFXConfiguration climbConfig = new TalonFXConfiguration();
    climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climberMotor.getConfigurator().apply(climbConfig);

    
  }

  //stops the climber when called
  public void stopClimber(){
    climberMotor.setControl(m_request.withOutput(0));
  }

  public void updateClimberPos(){
    climbPos = climberMotor.getPosition().getValueAsDouble();
  }

  public double returnClimberPos(){
    updateClimberPos();
    return climbPos;
  }

  //runs the climber to the setPoint (up and down)
  public void runClimber(double setPoint){
    //sets PID values (Phoenix 6 sucks)
    var slot0Configs = new Slot0Configs();
      slot0Configs.kP = 15; 
      slot0Configs.kI = 0; 
      slot0Configs.kD = 0.1;
      
    //applies PID values
    climberMotor.getConfigurator().apply(slot0Configs);
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    climberMotor.setControl(m_request.withPosition(setPoint));
    
  }

  public boolean atSetpoint(double setPoint){
    if(climberMotor.getPosition().getValueAsDouble() == setPoint)return true;
    return false;
  }

  public void manualClimb(double rightY){
    final VoltageOut m_request = new VoltageOut(0);
    climberMotor.setControl(m_request.withOutput(rightY*6));
  }

  public void setZero(){
    climberMotor.setPosition(0);
  }

  
}
