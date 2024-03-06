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
  private double Output;
  int timer;
  /** Creates a new Outtake. */
  public Outtake(Intake intake, Shooter shooter, double IntakeOutput) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Intake = intake;
    s_Shooter = shooter;
    this.Output = IntakeOutput;
    addRequirements(s_Intake);
    addRequirements(s_Shooter);
  }

  @Override
  public void initialize(){
  // System.out.println("Outtake is initialized");
   timer = 0;
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer > 30){
    s_Intake.randOut(Output);
    s_Shooter.randOut(-0.9);
    }
    if(timer < 75){
  //    System.out.println("Outtake is running" + timer);
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
    return timer == 75;
  }

 
}
