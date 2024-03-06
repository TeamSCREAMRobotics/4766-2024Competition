// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Pivot.armSetPoint;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class runIntake extends Command {
  private Shooter s_Shooter;
  private Intake s_Intake;

  double output;
  double intakePhase;
  int timer;
  
  /** Creates a new Intake. */
  public runIntake(Intake intake, Shooter shooter, double output) {
    s_Shooter = shooter;
    s_Intake = intake;
    this.output = output;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
    addRequirements(s_Shooter);
    
  }

  @Override
  public void initialize(){
    s_Shooter.setIntakeLoaded(false);
    intakePhase = 0;
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //checks for note before running
    //runs intake
    s_Intake.runIntake(output);

    if(output == -5){
      s_Intake.runIntake(output);
      s_Shooter.runConveyor(output);
      timer ++;
      if(s_Shooter.beamBreakTriggered() == false && timer == 15){
        intakePhase = 3;
      }
    }
    else{
    
    //runs conveyor until beambreak is triggered
    if(intakePhase == 0){
      s_Shooter.runConveyor(output);
      if(s_Shooter.beamBreakTriggered())intakePhase = 1;
    }

    //runs conveyor back for one cycle to ensure the note isn't touching the flywheel
    else if(intakePhase == 1){
      
      intakePhase = 2;
    }
    else if(intakePhase == 2){
      s_Shooter.Outtake();
      intakePhase = 3;
    }
    //tells the robot the intake is loaded
    else{
      s_Shooter.setIntakeLoaded(true);
    }
    
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.resetIntake();
    s_Shooter.resetConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Shooter.getIntakeLoaded();
  }
}
