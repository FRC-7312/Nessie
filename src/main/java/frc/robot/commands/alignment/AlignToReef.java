package frc.robot.commands.alignment;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.PositionState;
import frc.robot.subsystems.Swerve;

public class AlignToReef extends Command {

  private PositionState targetPosition;
  private Swerve swerve;

  private PIDController xController, yawController;

  private double tagID;
  private Timer dontSeeTagTimer;
  private Timer poseValidationTimer;

  public AlignToReef(PositionState targetstate, Swerve swerve) {

    xController = new PIDController(
      Constants.VisionConstants.X_P,
      Constants.VisionConstants.X_I,
      Constants.VisionConstants.X_D
    );
    xController.setTolerance(Constants.VisionConstants.X_TOLERANCE);

    yawController = new PIDController(
      Constants.VisionConstants.YAW_P,
      Constants.VisionConstants.YAW_I,
      Constants.VisionConstants.YAW_D
    );
    yawController.setTolerance(Constants.VisionConstants.YAW_TOLERANCE);

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
    System.out.println("AlignToReef initialized with tag ID: " + tagID);
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
    if(!(xController.atSetpoint() && yawController.atSetpoint())) {
      poseValidationTimer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, true, true);
    if (interrupted) {
      System.out.println("AlignToReef command interrupted.");
    } else {
      System.out.println("AlignToReef command completed successfully.");
    }
  }

  @Override
  public boolean isFinished() {
    return 
      this.dontSeeTagTimer.hasElapsed(Constants.VisionConstants.DONT_SEE_TAG_WAIT_TIME) || 
      this.poseValidationTimer.hasElapsed(Constants.VisionConstants.POSE_VALIDATION_TIME);
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
      new Translation2d(0, -xController.calculate(robotInTargetSpace[0], targetPosition.getX())), 
      -yawController.calculate(robotInTargetSpace[4], targetPosition.getYaw()), 
      false, 
      true
    );
  }

}
