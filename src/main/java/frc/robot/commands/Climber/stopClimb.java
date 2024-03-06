// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class stopClimb extends Command {
  Climber s_Climber = new Climber();
  /** Creates a new stopClimb. */
  public stopClimb(Climber climber) {
    s_Climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Climber.stopClimber();
  }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
