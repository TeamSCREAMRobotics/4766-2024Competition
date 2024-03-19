// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Pivot.armSetPoint;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  Shooter s_Shooter;
  Pivot s_Pivot;
  double shooterPhase;
  double shooterVoltage;
  int timer;
  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Pivot pivot, double input) {
    s_Shooter = shooter;
    s_Pivot = pivot;
    shooterVoltage = input;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Shooter);
    addRequirements(s_Pivot);
  }

  @Override
  public void initialize(){
    //checks for note before running, if no note command ends instantly
    shooterPhase = 0;
    //else shooterPhase = 4;
    //resets timer
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //spins up shooter
    s_Shooter.shoot(shooterVoltage);
    System.out.println(s_Shooter.shooterMaster.getVelocity());
    //checks to see if shooter is at velocity before running conveyor
    //won't stop if note is being shot (friction will lower velocity, will adjust code when prototype is built)
    if(s_Shooter.getShooterVelocity()>60&&shooterPhase != 3){
      //runs conveyor
      s_Shooter.runConveyor(ShooterConstants.conveyorIntakeOutput);
      //moves to phase one if on phase 0
      if(shooterPhase==0) shooterPhase = 1;
      //switches to phase 2 after note passes beambreak
      
      //switches to phase 3 if on phase 2 and the beambreak isn't triggered (indicates note being shot)
      if(shooterPhase >= 1 && !s_Shooter.beamBreakTriggered()){
        shooterPhase = 2;
        //shuts off conveyor
        s_Shooter.resetConveyor();
        shooterPhase = 3;
        
        
      }
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops shooter
    s_Shooter.resetShooter();
    //tells robot there isn't a note in the shooterbox
    s_Shooter.setIntakeLoaded(false);
    s_Pivot.endSetPointCommand(true);
    System.out.println("finished shoot");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //ends if in phase 4
    return shooterPhase == 3;
  }
}
