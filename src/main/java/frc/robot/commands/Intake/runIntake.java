// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
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
    
    
  }

  @Override
  public void initialize(){
    intakePhase = 0;
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //checks for note before running
    //runs intake
    s_Intake.runIntake(output);

    //runs conveyor until beambreak is triggered
    s_Shooter.runConveyor(output);
      
    
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
    return s_Shooter.beamBreakTriggered();
  }
}
