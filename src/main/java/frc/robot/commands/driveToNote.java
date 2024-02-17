// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class driveToNote extends Command {

      double limelight_aim_proportional(){
        double kP = .035;
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
        targetingAngularVelocity *= Constants.AutoConstants.K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }

    double limelight_range_proportional(){
        double kP =.1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= Constants.Swerve.MAX_SPEED;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }
  /** Creates a new driveToNote. */
  public driveToNote() {

    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final var rot_limelight = limelight_aim_proportional();
    int rotationAxis = (int) rot_limelight;

    final var forward_limelight = limelight_range_proportional();
    Constants.Swerve.MAX_SPEED = forward_limelight;

    Translation2d tran2d = new Translation2d(0, forward_limelight);
    //while using Limelight, turn off field-relative driving.
    // Swerve.fieldRelative = false;

    RobotContainer.s_Swerve.drive(tran2d, rotationAxis, false, true);
    System.out.println("Swerve Driving *******");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Driving Interrupted(!)");
    Translation2d tran2d = new Translation2d(0, 0);
    RobotContainer.s_Swerve.drive(tran2d, 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
