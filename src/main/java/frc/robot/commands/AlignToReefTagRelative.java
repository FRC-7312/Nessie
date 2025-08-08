package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private String desiredAlignment;
  private Timer dontSeeTagTimer, stopTimer;
  private Swerve swerve;
  private double tagID = -1;
  private String limelightName = "limelight-left";

  public AlignToReefTagRelative(String desiredAlignment, Swerve swerve) {
    xController = new PIDController(Constants.AlignmentConstants.X_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.AlignmentConstants.Y_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.AlignmentConstants.ROT_P, 0, 0);  // Rotation
    this.desiredAlignment = desiredAlignment;
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setTolerance(Constants.AlignmentConstants.ROT_TOLERANCE);
    xController.setTolerance(Constants.AlignmentConstants.X_TOLERANCE);
    yController.setTolerance(Constants.AlignmentConstants.Y_TOLERANCE);

    switch(desiredAlignment) {
      case "left forward":
        xController.setSetpoint(Constants.AlignmentConstants.LEFT_X_FORWARD);
        yController.setSetpoint(Constants.AlignmentConstants.LEFT_Y);
        rotController.setSetpoint(Constants.AlignmentConstants.LEFT_ROT);
        limelightName = "limelight-left";
        break;
      case "right forward":
        xController.setSetpoint(Constants.AlignmentConstants.RIGHT_X_FORWARD);
        yController.setSetpoint(Constants.AlignmentConstants.RIGHT_Y);
        rotController.setSetpoint(Constants.AlignmentConstants.RIGHT_ROT);
        limelightName = "limelight-right";
        break;
      case "left back":
        xController.setSetpoint(Constants.AlignmentConstants.LEFT_X_BACK);
        yController.setSetpoint(Constants.AlignmentConstants.LEFT_Y);
        rotController.setSetpoint(Constants.AlignmentConstants.LEFT_ROT);
        limelightName = "limelight-left";
        break;
      case "right back":
        xController.setSetpoint(Constants.AlignmentConstants.RIGHT_X_BACK);
        yController.setSetpoint(Constants.AlignmentConstants.RIGHT_Y);
        rotController.setSetpoint(Constants.AlignmentConstants.RIGHT_ROT);
        limelightName = "limelight-right";
        break;
      case "center forward":
        xController.setSetpoint(Constants.AlignmentConstants.CENTER_X_FORWARD);
        yController.setSetpoint(Constants.AlignmentConstants.CENTER_Y);
        rotController.setSetpoint(Constants.AlignmentConstants.CENTER_ROT);
        break;
      case "center back":
        xController.setSetpoint(Constants.AlignmentConstants.CENTER_X_BACK);
        yController.setSetpoint(Constants.AlignmentConstants.CENTER_Y);
        rotController.setSetpoint(Constants.AlignmentConstants.CENTER_ROT);
        break;
    }
    SmartDashboard.putNumber("X Setpoint", xController.getSetpoint());
    SmartDashboard.putNumber("Y Setpoint", yController.getSetpoint());
    SmartDashboard.putNumber("Rot Setpoint", rotController.getSetpoint());

    tagID = LimelightHelpers.getFiducialID(limelightName);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV(limelightName) && LimelightHelpers.getFiducialID(limelightName) == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace(limelightName);

      double xSpeed = xController.calculate(postions[2]);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);
      if(limelightName.equals("limelight-left")) {
        rotValue = -rotValue;
      }

      swerve.drive(
        new Translation2d(xSpeed, ySpeed).times(Constants.SwerveConstants.maxSpeed).times(.5), 
        rotValue * Constants.SwerveConstants.maxAngularVelocity, 
        false, 
        true
      );

      if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
        stopTimer.reset();
      }
    }else {
      swerve.drive(new Translation2d(), 0, false, true);
    }

    SmartDashboard.putNumber("Alignment/Pose Valid Timer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, false, true);
  }

  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(Constants.AlignmentConstants.DONT_SEE_TAG_WAIT_TIME) ||
      stopTimer.hasElapsed(Constants.AlignmentConstants.POSE_VALIDATION_TIME);
  }
}