// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class resetIntake extends Command {
  private Shooter s_Shooter;
  private Intake s_Intake;
  private boolean isDone;
  /** Creates a new resetIntake. */
  public resetIntake(Shooter shooter, Intake intake) {
    s_Intake = intake;
    s_Shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Shooter);
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.resetIntake();
    s_Shooter.resetConveyor();
    isDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
