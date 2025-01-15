package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightReader {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final String[] entry = { "tx", "ty", "ta" };

    public class RetValPretty {
        String tag;
        double val;

        RetValPretty(double val, String tag) {
            this.val = val;
            this.tag = tag;
        }
    }

    public class CameraTargetDistance {
        private double dx, dy, tx, ty, ta, d;
        private final double lengthApriltag = 0.145; // measured in meters

        // intrinsics of camera

        private final double hFOV = 62.235;
        private final double vFOV = 48.697;

        /**
         * We assume target apriltag is orthogonal to the vision field, thus an estimate
         * can be made.
         * 
         * @return target distance, in x and y directional components to crosshair.
         */
        private CameraTargetDistance() {
            tx = table.getEntry("tx").getDouble(0.0);
            ty = table.getEntry("ty").getDouble(0.0);
            ta = table.getEntry("ta").getDouble(0.0);
            d = Math.sqrt(
                    (lengthApriltag * lengthApriltag / (ta / 100)) / (4 * Math.tan(hFOV / 2 * (Math.PI / 180))
                            * Math.tan(vFOV / 2 * (Math.PI / 180))));
            dx = d * Math.tan(tx * (Math.PI / 180));
            dy = d * Math.tan(ty * (Math.PI / 180));
        }

    }

    public void reportDistance() {
        CameraTargetDistance inst = new CameraTargetDistance();
        SmartDashboard.putNumber("distanceX", inst.dx);
        SmartDashboard.putNumber("distanceY", inst.dy);
        SmartDashboard.putNumber("distance", inst.d);
    }

    public RetValPretty[] getPrettyRetVal() {
        RetValPretty[] ret = new RetValPretty[entry.length];
        for (int i = 0; i < entry.length; i++) {
            ret[i] = new RetValPretty(table.getEntry(entry[i]).getDouble(0.0), entry[i]);
        }
        return ret;
    }

    public void postToSmartDashboard() {
        RetValPretty[] poster = this.getPrettyRetVal();
        for (RetValPretty item : poster) {
            SmartDashboard.putNumber(item.tag, item.val);
        }
    }

    // public Translation2d getHeading() {

    // }
}
