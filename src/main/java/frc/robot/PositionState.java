package frc.robot;

public class PositionState {
    private double x;
    private double y;
    private double yaw;

    public PositionState(double x, double y, double yaw) {
        this.x = x;
        this.y = y;
        this.yaw = yaw;
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
