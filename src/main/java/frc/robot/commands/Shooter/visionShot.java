// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelper;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
//auto aim using the Apriltags. Lines up, raises pivot, and spins shooter, waits for driver to shoot (more accurate in practice)
public class visionShot extends Command {
  Swerve s_Swerve;
  Pivot s_Pivot;
  Shooter s_Shooter;
  Limelight s_Limelight;

  double setpoint;
  double tagDist;
  double shooterPhase;
  int timer;
  double shootVelocity;
  Timer waitTime = new Timer();

  PIDController targetPID = new PIDController(0.04, 0, 0.0);
  
  /** Creates a new visionShot. */
  public visionShot(Swerve swerve, Pivot pivot, Shooter shooter, Limelight limelight) {
    s_Shooter = shooter;
    s_Swerve = swerve;
    s_Pivot = pivot;
    s_Limelight = limelight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
    addRequirements(s_Pivot);
    addRequirements(s_Shooter);
    addRequirements(s_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    waitTime.reset();
    tagDist = LimelightHelper.getTY("limelight");
    shooterPhase = 0;
    shootVelocity = Constants.VELOCITY_STATE_MAP.get(tagDist);
    setpoint = Constants.PIVOT_STATE_MAP.get(tagDist);
    s_Pivot.endSetPointCommand(false);
    timer = 0;
    targetPID.setTolerance(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tagDist = LimelightHelper.getTY("limelight");


// This runs if isAuto is false.
    

 if(LimelightHelper.getTV("limelight")){
      timer = 0;
    }

    if(!LimelightHelper.getTV("limelight")){
      timer ++;
    }



      if(LimelightHelper.getTX("limelight") != 0)
        {s_Swerve.autoDrive(s_Swerve.fieldRelativeSpeeds(new Translation2d(), targetPID.calculate(LimelightHelper.getTX("limelight"))));
        }


       //pivots arm to setPoint
    s_Pivot.goToSetPoint(setpoint);

    double currentAngle = s_Pivot.pivotMaster.getPosition().getValueAsDouble();
     if (currentAngle > setpoint - 0.05 && s_Pivot.pivotMaster.getVelocity().getValueAsDouble() < 0.5){
      return;
    }
    //spins up shooter
    s_Shooter.shoot(shootVelocity);
    // The command 

    }


  
   
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.resetConveyor();
    s_Shooter.resetShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !s_Shooter.beamBreakTriggered() || timer >= 15;
  }
}