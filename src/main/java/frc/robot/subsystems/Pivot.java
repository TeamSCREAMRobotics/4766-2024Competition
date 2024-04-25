// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  public TalonFX pivotMaster = new TalonFX(PivotConstants.pivotMasterID);
  public TalonFX pivotFollower = new TalonFX(PivotConstants.pivotFollowerID);
  double pivotPercent;
  double pivotPos;
 public boolean endSetPoint = true;
  
  /** Creates a new Pivot. */
  public Pivot() {
    pivotMaster.getConfigurator().apply(new TalonFXConfiguration());
    pivotFollower.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration pivot_config = new TalonFXConfiguration();
    pivot_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivot_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivot_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 8.2;
    pivot_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivot_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.1;
    pivot_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    

    pivotMaster.getConfigurator().apply(pivot_config);
    pivotFollower.getConfigurator().apply(pivot_config);
    pivotFollower.setControl(new Follower(
      pivotMaster.getDeviceID(), 
      false));
  }

  //updates the current position of the pivot (wasn't updating when it wasn't a separate class for some reason)
  public void updatePivotPos(){
    pivotPos = pivotMaster.getPosition().getValueAsDouble();
  }

  //returns the current positon of the pivot
  public double returnPivotPos(){
    updatePivotPos();
    return pivotPos;
  }

  //main class for moving the pivot to a set position using the PID parameters in constants. 
  public void goToSetPoint(double setPoint){
    var slot0Configs = new Slot0Configs();
      slot0Configs.kG = PivotConstants.pivotKG;
      slot0Configs.kP = PivotConstants.pivotKP;
      slot0Configs.kI = 0;
      slot0Configs.kD = PivotConstants.pivotKD;
      slot0Configs.kV = PivotConstants.pivotKV;
      slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    pivotMaster.getConfigurator().apply(slot0Configs);
    MotionMagicConfigs pivot_magic = new MotionMagicConfigs();
    pivot_magic.MotionMagicCruiseVelocity = 90;
    pivot_magic.MotionMagicAcceleration = 35;
    pivotMaster.getConfigurator().apply(pivot_magic);
    final MotionMagicVoltage magic_request = new MotionMagicVoltage(0).withSlot(0);
    
    pivotMaster.setControl(magic_request.withPosition(setPoint));
  }

  //Compares current location of pivot with desired location and returns true if they are equal
  public boolean isAtSetPoint(double setPoint){
    return pivotMaster.getPosition().getValue()==setPoint;
  }

  //zeroes the pivot (had a few womp womps)
  public void setZero(){
    pivotMaster.setPosition(0);
  }

  public void manualPivot(double percent){
    final VoltageOut m_request = new VoltageOut(0);
    pivotMaster.setControl(m_request.withOutput((percent * 4)/*+PivotConstants.pivotKG*/)); 
    //This is being printed so I could see where motor positions were for setpoints 
    //System.out.println(pivotMaster.getPosition());
  }
  //resets the john
  public void resetPivot(){
    final VoltageOut m_request = new VoltageOut(0);
    pivotMaster.setControl(m_request.withOutput(0));
  }

  public void endSetPointCommand(boolean set){
    endSetPoint = set;
  }

  @Override
  public void periodic(){
    
  }
  
}
