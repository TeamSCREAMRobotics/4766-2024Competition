// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

//rumbles the selected controller for a set period of time.
public class ControllerRumble extends Command {
  double rumbleTime;
  double timer;
  XboxController commandDriver;
  boolean done;
  
  /** Creates a new ControllerRumble. */
  public ControllerRumble(XboxController controller, double RumbleTime) {
    rumbleTime = RumbleTime;
    commandDriver = controller;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   timer = 0;
   done = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(timer < rumbleTime + 1){
      commandDriver.setRumble(RumbleType.kBothRumble, 1);
      timer++;
      // System.out.println(timer);
      return;
    }
  
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    commandDriver.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return timer == rumbleTime;
  }
}
