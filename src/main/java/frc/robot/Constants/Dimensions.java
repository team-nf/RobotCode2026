package frc.robot.Constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;

public final class Dimensions {
    // Robot dimensions
    public static final Distance BUMPER_LENGTH = Meters.of(0.837); // 33 inches
    public static final Distance BUMPER_WIDTH = Meters.of(0.853);  // 33.6 inches
    public static final Distance BUMPER_HEIGHT = Meters.of(0.154);   // 36 inches

    public static final Distance BUMPER_COLLISION_DISTANCE = Meters.of(0.870); // 22 inches

    public static final Distance HOPPER_EXTENSION_LENGTH = Meters.of(0.320); // 12 inches

    public static final Distance FUEL_SHOOTER_OFFSET = Meters.of(0.1385); // 4.22 inches

    public static final Distance HUB_HEIGHT = Meters.of(1.8); // 16.4 inches

    public static final Pose3d LEFT_SHOOTER_POSE = new Pose3d(
        Meters.of(-0.107), 
        Meters.of(0.0825), 
        Meters.of(0.416), 
        new Rotation3d()    
    );

    public static final Pose3d RIGHT_SHOOTER_POSE = new Pose3d(
        Meters.of(-0.107), 
        Meters.of(-0.0825), 
        Meters.of(0.416), 
        new Rotation3d()    
    );

    public static boolean[] checkFieldCollision(Pose2d[] robotCorners)
    {
        boolean x_collided = false;
        boolean y_collided = false;

        double fieldXMin = 0.1;
        double fieldXMax = 16.33;
        double fieldYMin = 0.1;
        double fieldYMax = 8.15;

        for (Pose2d corner : robotCorners) {
            double x = corner.getX();
            double y = corner.getY();

            if (x < fieldXMin || x > fieldXMax) {
                x_collided = true;
            }
            if (y < fieldYMin || y > fieldYMax) {
                y_collided = true;
            }
        }

        return new boolean[] {x_collided, y_collided};
    }
}