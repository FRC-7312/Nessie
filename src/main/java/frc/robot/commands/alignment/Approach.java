package frc.robot.commands.alignment;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.PositionState;
import frc.robot.subsystems.Swerve;

public class Approach extends Command {

  private PositionState targetPosition;
  private Swerve swerve;

  private PIDController yController;

  private double tagID;
  private Timer dontSeeTagTimer;
  private Timer poseValidationTimer;

  public Approach(PositionState targetstate, Swerve swerve) {
  
    yController = new PIDController(
      Constants.VisionConstants.X_P,
      Constants.VisionConstants.X_I,
      Constants.VisionConstants.X_D
    );
    yController.setTolerance(Constants.VisionConstants.Y_TOLERANCE);


    this.targetPosition = targetstate;
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.poseValidationTimer = new Timer();
    this.poseValidationTimer.start();

    tagID = LimelightHelpers.getFiducialID(targetPosition.getPerferredCameraName());
    System.out.println("Approach initialized with tag ID: " + tagID);
    if(tagID != -1) {
      System.out.println("Approach initialized with tag ID: " + tagID);
    }else {
      System.out.println("Approach failed to initialize: " + tagID);
      end(false);
    }
  }

  @Override
  public void execute() {
    double[] robotInTargetSpace = null;

    if (LimelightHelpers.getTV(Constants.VisionConstants.LEFT_LIMELIGHT_NAME) && 
        LimelightHelpers.getTV(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME) && 
        LimelightHelpers.getFiducialID(Constants.VisionConstants.LEFT_LIMELIGHT_NAME) == tagID && 
        LimelightHelpers.getFiducialID(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME) == tagID) {
      dontSeeTagTimer.reset();
      robotInTargetSpace = averageArrays(
        LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.LEFT_LIMELIGHT_NAME),
        LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME)
      );
      doDrive(robotInTargetSpace);
    }else if (LimelightHelpers.getTV(Constants.VisionConstants.LEFT_LIMELIGHT_NAME) && 
              LimelightHelpers.getFiducialID(Constants.VisionConstants.LEFT_LIMELIGHT_NAME) == tagID) {
      dontSeeTagTimer.reset();
      robotInTargetSpace = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.LEFT_LIMELIGHT_NAME);
      doDrive(robotInTargetSpace);
    }else if (LimelightHelpers.getTV(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME) && 
              LimelightHelpers.getFiducialID(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME) == tagID) {
      dontSeeTagTimer.reset();
      robotInTargetSpace = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME);
      doDrive(robotInTargetSpace);
    }else {
      swerve.drive(new Translation2d(), 0, true, true);
    }
    if(!yController.atSetpoint()) {
      poseValidationTimer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, true, true);
    if (interrupted) {
      System.out.println("Approach command interrupted.");
    } else {
      System.out.println("Approach command completed successfully.");
    }
  }

  @Override
  public boolean isFinished() {
    boolean dontSeeTagTimerElapsed = dontSeeTagTimer.hasElapsed(Constants.VisionConstants.DONT_SEE_TAG_WAIT_TIME);
    boolean poseValidationElapsed = poseValidationTimer.hasElapsed(Constants.VisionConstants.POSE_VALIDATION_TIME);

    if(dontSeeTagTimerElapsed) {
      System.out.println("Dont See Tag Timer Elapsed");
      return true;
    }else if(poseValidationElapsed) {
      System.out.println("Pose Validation Timer Elapsed");
      return true;
    }
    return false;
  }

  public static double[] averageArrays(double[] a, double[] b) {
    double[] result = new double[a.length];
    for (int i = 0; i < a.length; i++) {
      result[i] = (a[i] + b[i]) / 2.0;
    }
    return result;
  }

  private void doDrive(double[] robotInTargetSpace) {
    swerve.drive(
      new Translation2d(yController.calculate(robotInTargetSpace[0], targetPosition.getX()), 0), 
      0, 
      false, 
      true
    );
  }

}
