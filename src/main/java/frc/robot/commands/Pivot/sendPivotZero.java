// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class sendPivotZero extends Command {
  Pivot s_Pivot = new Pivot();
  double setPoint;
  int timer;
  double timerLength;
  /** Creates a new armSetPoint. */
  public sendPivotZero(Pivot pivot, double setpoint, double timeLength) {
    timerLength = timeLength;
    s_Pivot = pivot;
    setPoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Pivot);
  }

  @Override
  public void initialize(){
    timer = 0;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //pivots arm to setPoint
    s_Pivot.goToSetPoint(setPoint);
    timer++;
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //ends command when the arm is at the current setpoint.
    return timer == timerLength;
  }
}
