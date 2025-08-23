package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PositionState {
    private double x;
    private double y;
    private double yaw;

    /**
     * Constructor for PositionState.
     * @param x The x position in meters.
     * @param y The y position in meters.
     * @param yaw The yaw in degrees.
     */
    public PositionState(double x, double y, double yaw) {
        this.x = x;
        this.y = y;
        this.yaw = yaw;
    }

    /**
     * Get the Pose2d representation of this position state.
     * @return A Pose2d object representing the position and orientation.
     */
    public Pose2d getPose() {
        return new Pose2d(x, y, new Rotation2d(Math.toRadians(yaw)));
    }

    /**
     * Get the x position in meters.
     * @return The x position in meters.
     */
    public double getX() {
        return x;
    }

    /**
     * Get the y position in meters.
     * @return The y position in meters.
     */
    public double getY() {
        return y;
    }

    /**
     * Get the yaw in degrees.
     * @return The yaw in degrees.
     */
    public double getYaw() {
        return yaw;
    }
    
}
