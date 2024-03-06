// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class armSetPoint extends Command {

  Shooter s_Shooter;
  Pivot s_Pivot = new Pivot();
  double shooterPhase;
  double setPoint;
  int timer;
  /** Creates a new armSetPoint. */
  public armSetPoint(Pivot pivot, Shooter shooter, double setpoint) {
    s_Pivot = pivot;
    s_Shooter = shooter;
    setPoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Pivot);
  }

  @Override
  public void initialize(){
    shooterPhase = 0;
    s_Pivot.endSetPointCommand(false);
    timer = 0;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //pivots arm to setPoint
    s_Pivot.goToSetPoint(setPoint);
    
    if (s_Pivot.pivotMaster.getPosition().getValueAsDouble() <setPoint-0.19){
      return;
    }
    //spins up shooter
    s_Shooter.shoot(ShooterConstants.shooterLowerVelocity);
    System.out.println(s_Shooter.shooterMaster.getVelocity());
    //checks to see if shooter is at velocity before running conveyor
    //won't stop if note is being shot (friction will lower velocity, will adjust code when prototype is built)
    if(s_Shooter.getShooterVelocity()>1.8&&shooterPhase != 3){
      //runs conveyor
      s_Shooter.runConveyor(ShooterConstants.conveyorIntakeOutput);
      //moves to phase one if on phase 0
      if(shooterPhase==0) shooterPhase = 1;
      //switches to phase 2 after note passes beambreak
      
      //switches to phase 3 if on phase 2 and the beambreak isn't triggered (indicates note being shot)
      if(!s_Shooter.beamBreakTriggered()){
        shooterPhase = 2;
        //shuts off conveyor
        s_Shooter.resetConveyor();
       
        
          s_Pivot.endSetPointCommand(true);

        shooterPhase = 3;
          
        
        
      }
    }
  }

  @Override
  public void end(boolean interrupted){
    s_Shooter.resetShooter();
    
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //ends command when the arm is at the current setpoint.
    return shooterPhase == 3;
    
  }
}
