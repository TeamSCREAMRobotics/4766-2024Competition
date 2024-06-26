// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  Climber s_Climber = new Climber();
  double setPoint;
  /** Creates a new Climb. */
  public Climb(Climber climber, double setpoint) {
    s_Climber = climber;
    setPoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber);
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Climber.runClimber(setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Climber.atSetpoint(setPoint);
  }
}
