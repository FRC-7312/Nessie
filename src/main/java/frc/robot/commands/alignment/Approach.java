package frc.robot.commands.alignment;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.PositionState;
import frc.robot.subsystems.RevBlinkin;
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
      RevBlinkin.getInstance().setPattern(RevBlinkin.BlinkinPattern.BREATH_GRAY);;
    }else {
      System.out.println("Approach failed to initialize: " + tagID);
      end(true);
    }

    swerve.setX();
    new WaitCommand(.2).schedule();
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

    SmartDashboard.putBoolean("at set", yController.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, true, true);
    if (interrupted) {
      System.out.println("AlignToReef command interrupted.");
      RevBlinkin.getInstance().flashAlternate(
        RevBlinkin.BlinkinPattern.RED,
        RevBlinkin.BlinkinPattern.BLACK,
        0.1,
        0.5,
        RevBlinkin.BlinkinPattern.RED
      );
    } else {
      System.out.println("AlignToReef command completed successfully.");
      RevBlinkin.getInstance().flashAlternate(
        RevBlinkin.BlinkinPattern.GREEN,
        RevBlinkin.BlinkinPattern.BLACK,
        0.1,
        0.5,
        RevBlinkin.BlinkinPattern.RED
      );
    }
  }

  @Override
  public boolean isFinished() {
    boolean dontSeeTagTimerElapsed = dontSeeTagTimer.hasElapsed(Constants.VisionConstants.DONT_SEE_TAG_WAIT_TIME);

    if(dontSeeTagTimerElapsed) {
      System.out.println("Dont See Tag Timer Elapsed");
      return true;
    }
    if(yController.atSetpoint()) {
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
    SmartDashboard.putNumber("robotPose1", robotInTargetSpace[1]);
    SmartDashboard.putNumber("robotPoseM", targetPosition.getY());
    swerve.drive(
      new Translation2d(yController.calculate(robotInTargetSpace[0], targetPosition.getY()), 0), 
      0, 
      false, 
      true
    );
  }

}
