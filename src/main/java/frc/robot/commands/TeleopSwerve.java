package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.PositionState;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;


public class TeleopSwerve extends Command {    
    
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier alignLeftSupplier;
    private BooleanSupplier alignRightSupplier;
    private BooleanSupplier lockSup;

    private PIDController yController = new PIDController(
        Constants.VisionConstants.Y_P,
        Constants.VisionConstants.Y_I,
        Constants.VisionConstants.Y_D
    );

    private PIDController rotationController = new PIDController(
        Constants.VisionConstants.YAW_P,
        Constants.VisionConstants.YAW_I,
        Constants.VisionConstants.YAW_D
    );

    public TeleopSwerve(Swerve swerve, 
                        DoubleSupplier translationSup, 
                        DoubleSupplier strafeSup, 
                        DoubleSupplier rotationSup, 
                        BooleanSupplier alignLeftSupplier,
                        BooleanSupplier alignRightSupplier,
                        BooleanSupplier lockSup) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.alignLeftSupplier = alignLeftSupplier;
        this.alignRightSupplier = alignRightSupplier;
        this.lockSup = lockSup;
    }

    @Override
    public void execute() {
        double[] leftBotPose = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.LEFT_LIMELIGHT_NAME);
        double[] rightBotPose = LimelightHelpers.getBotPose_TargetSpace(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME);

        boolean didSomething = false;

        if (alignLeftSupplier.getAsBoolean() || alignRightSupplier.getAsBoolean()) {

            System.out.println("Align Suppliers Triggered");

            double x = -999, yaw = -999;

            if (LimelightHelpers.getTV(Constants.VisionConstants.LEFT_LIMELIGHT_NAME) && LimelightHelpers.getTV(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME)) {
                x = (leftBotPose[0] + rightBotPose[0]) / 2.0;
                yaw = (leftBotPose[4] + rightBotPose[4]) / 2.0;
            } else if (LimelightHelpers.getTV(Constants.VisionConstants.LEFT_LIMELIGHT_NAME)) {
                x = leftBotPose[0];
                yaw = leftBotPose[4];
            } else if (LimelightHelpers.getTV(Constants.VisionConstants.RIGHT_LIMELIGHT_NAME)) {
                x = rightBotPose[0];
                yaw = rightBotPose[4];
            } else {
                return;
            }

            PositionState targetPosition = alignLeftSupplier.getAsBoolean() 
                ? StateMachine.LEFT_FRONT 
                : StateMachine.RIGHT_FORWARD;

            swerve.drive(
                new Translation2d(MathUtil.applyDeadband(
                    translationSup.getAsDouble(), 
                    Constants.ControlConstants.STICK_DEADBAND) , -yController.calculate(x, targetPosition.getX())), 
                -rotationController.calculate(yaw, targetPosition.getYaw()), 
                false, 
                true
            );
            didSomething = true;

        }

        if (!didSomething && lockSup.getAsBoolean()) {
            swerve.setX();
            didSomething = true;
        }

        if (!didSomething) {
            double translationVal = MathUtil.applyDeadband(
                translationSup.getAsDouble(), 
                Constants.ControlConstants.STICK_DEADBAND
            );
            double strafeVal = MathUtil.applyDeadband(
                strafeSup.getAsDouble(), 
                Constants.ControlConstants.STICK_DEADBAND
            );
            double rotationVal = MathUtil.applyDeadband(
                rotationSup.getAsDouble(), 
                Constants.ControlConstants.STICK_DEADBAND
            );

            swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed), 
                rotationVal * Constants.SwerveConstants.maxAngularVelocity, 
                true, 
                true
            );
        }
    }

}