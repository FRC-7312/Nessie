package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveModules;
    public Pigeon2 gyro;
    public static Field2d odometryField;
    public double speedMultiplier;
    public StructPublisher<Pose3d> posePublisher;
    
    public Swerve() {
        gyro = new Pigeon2(Constants.SwerveConstants.pigeonID, "cani");
        Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
        pigeonConfig.MountPose.MountPoseYaw = 180.0;
        gyro.getConfigurator().apply(pigeonConfig);
        gyro.reset();
        speedMultiplier = 1.0;
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Pose", Pose3d.struct).publish();

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());
        odometryField = new Field2d();


        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {}

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getOdometryPose, // Robot pose supplier
                this::resetOdometryPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );

        System.out.println("Swerve subsystem initialized");
    }

    public Command changeSpeedMultiplierCommand() {
        return runOnce(() -> {
            if(speedMultiplier == 1.0){
                speedMultiplier = 0.25;
            } else {
                speedMultiplier = 1.0;
            }
            System.out.println("Swerve speed multiplier changed to: " + speedMultiplier);
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() * speedMultiplier, 
                                    translation.getY() * speedMultiplier, 
                                    rotation * speedMultiplier, 
                                    getGyroYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX() * speedMultiplier, 
                                    translation.getY() * speedMultiplier, 
                                    rotation * speedMultiplier));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for(SwerveModule mod : swerveModules){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] states = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.AutoConstants.kMaxSpeedMetersPerSecond);
        setModuleStates(states);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.AutoConstants.kMaxSpeedMetersPerSecond);
        
        for(SwerveModule mod : swerveModules){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveModules){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveModules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getOdometryPose() {
        return swerveOdometry.getPoseMeters();
    }
    
    public void resetOdometryPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getOdometryHeading(){
        return getOdometryPose().getRotation();
    }
    
    public void setOdometryHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getOdometryPose().getTranslation(), heading));
    }

    public Command zeroOdometryHeadingCommand(){
        return runOnce(() -> {
            zeroOdometryHeading();;
        });
    }

    public void zeroOdometryHeading() {
        gyro.reset();
        System.out.println("Zeroed Odometry Heading");
    }

    public Rotation2d getGyroYaw() {
        return gyro.getRotation2d();
    }
    

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : swerveModules){
            mod.resetToAbsolute();
        }
    }

    public Command applyTeleopHeadingOffset() {
        return runOnce(() -> {
            gyro.setYaw(getGyroYaw().rotateBy(Rotation2d.k180deg).getDegrees());
        });
    }

    public void setX(){
      swerveModules[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),true);
      swerveModules[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)),true);
      swerveModules[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)),true);
      swerveModules[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),true);
    }

    @Override
    public void periodic(){
        Pose2d current2DPose = getOdometryPose();
        swerveOdometry.update(getGyroYaw(), getModulePositions());  
        posePublisher.set(new Pose3d(current2DPose.getX(), current2DPose.getY(), 0, new Rotation3d(0, 0, gyro.getRotation2d().getRadians())));
        SmartDashboard.putBoolean("Swerve/Fast Mode", speedMultiplier == 1.00);
        SmartDashboard.putNumber("Swerve/Gyro", getGyroYaw().getDegrees() - getGyroYaw().getDegrees() % 1);
    }

}