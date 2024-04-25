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
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
//visionshot for autos that automatically shoots
public class visionShotAuto extends Command {
  Swerve s_Swerve;
  Pivot s_Pivot;
  Shooter s_Shooter;
  Limelight s_Limelight;

  double setpoint;
  double tagDist;
  double shooterPhase;
  int timer;
  double shootVelocity;
  boolean isAuto;
  boolean isNote;
  Timer waitTime = new Timer();

  PIDController targetPID = new PIDController(0.03, 0, 0.0);
  
  /** Creates a new visionShot. */
  public visionShotAuto(Swerve swerve, Pivot pivot, Shooter shooter, Limelight limelight) {
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
    isNote = false;
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
    System.out.println("Pivot Velocity" + s_Pivot.pivotMaster.getVelocity().getValueAsDouble());
    if(!s_Shooter.beamBreakTriggered()){
      isNote = true;
    }

// If the command is in auto mode it will go through checks to make sure its ready to fire instead of waiting for driver.

      // These if statements are used incase the aprilTag detection has slight loss.
       if(LimelightHelper.getTV("limelight")){
      timer = 0;
    }

    if(!LimelightHelper.getTV("limelight")){
      timer ++;
    }

        // Centers the robot to the aprilTag
         if(LimelightHelper.getTX("limelight") != 0 && s_Shooter.beamBreakTriggered())
        {s_Swerve.autoDrive(s_Swerve.fieldRelativeSpeeds(new Translation2d(), targetPID.calculate(LimelightHelper.getTX("limelight"))));}

        // Sends pivot to setpoint which is the value fetched from the table.
        s_Pivot.goToSetPoint(setpoint);

        // Makes sure that the pivot is not moving before starting the flywheels.
          waitTime.start();
          s_Shooter.shoot(shootVelocity);
        

    
        
        //Checks to make sure that the flywheels are at the proper voltage before moving the note.
        if(s_Shooter.shooterMaster.getVelocity().getValueAsDouble() >= shootVelocity * 7 && s_Pivot.pivotMaster.getVelocity().getValueAsDouble() == 0.0 && s_Pivot.pivotMaster.getPosition().getValueAsDouble() > 1){
          s_Shooter.runConveyor(ShooterConstants.conveyorIntakeOutput);
        }


    }
   
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.resetConveyor();
    s_Shooter.resetShooter();
    s_Pivot.resetPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !s_Shooter.beamBreakTriggered() || timer >= 15 || isNote;
  }
}