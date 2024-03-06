// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Outtake extends Command {
  private Intake s_Intake;
  private Shooter s_Shooter;
  int timer;
  /** Creates a new Outtake. */
  public Outtake(Intake intake, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Shooter = shooter;
    addRequirements(s_Intake);
    addRequirements(s_Shooter);
  }

  @Override
  public void initialize(){
    timer = 0;
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.Outtake();
    s_Shooter.Outtake();
    if(timer <10){
      timer++;
      return;
    }
    
  }

  @Override
  public void end(boolean interrupted){
    s_Intake.resetIntake();
    s_Shooter.resetConveyor();
  }

  @Override
  public boolean isFinished(){
    return timer == 200;
  }

 
}
