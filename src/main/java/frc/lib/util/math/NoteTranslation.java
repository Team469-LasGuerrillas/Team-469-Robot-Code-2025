package frc.lib.util.math;

import java.util.Optional;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class NoteTranslation {
    public static Translation2d getCameraToTargetTranslation(double txnc, double tync, Pose3d cameraPose) {
        double yawRadians = Math.toRadians(txnc);
        double pitchRadians = Math.toRadians(tync);

        Transform3d cameraToTarget = new Transform3d(new Translation3d(), new Rotation3d(0.0, pitchRadians, 0.0));
        cameraToTarget = cameraToTarget
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, yawRadians)));
        Transform3d cameraGroundPlaneToCamera = new Transform3d(new Translation3d(),
                new Rotation3d(0.0, cameraPose.getRotation().getY(), cameraPose.getRotation().getZ()));
        Rotation3d cameraGroundPlaneToTarget = new Pose3d().plus(cameraGroundPlaneToCamera.plus(cameraToTarget))
                .getRotation().unaryMinus();

        // Built a unit vector from adjusted rotation.
        // Raw vector: x = 1, y = tan(yaw), z = tan(pitch)
        // Make it a unit vector by dividing each component by magnitude
        // sqrt(x^2+y^2+z^2).
        double tan_ty = Math.tan(cameraGroundPlaneToTarget.getZ()); // y and z switch intentional
        double tan_tz = -Math.tan(cameraGroundPlaneToTarget.getY()); // y and z switch intentional

        // Find the fixed height difference between the center of the tag and the camera
        // lens
        // double differential_height = tagLocation.getZ()
        //         - (isTurretCamera ? Constants.kCameraHeightOffGroundMeters : Constants.kCameraBHeightOffGroundMeters);

        double differential_height = cameraPose.getZ() - Units.inchesToMeters(2.25);

        // We now obtain 3d distance by dividing differential_height by our normalized z
        // component z / (Math.sqrt(x^2+y^2+z^2))
        double distance = differential_height * Math.sqrt(1.0 + tan_tz * tan_tz + tan_ty * tan_ty) / tan_tz;
        // Build a 3d vector from distance (which we now know) and orientation (which we
        // already computed above).
        Translation3d cameraToTargetTranslation = new Translation3d(distance, cameraGroundPlaneToTarget);

        // Grab the x and y components.
        return new Translation2d(cameraToTargetTranslation.getX(), cameraToTargetTranslation.getY())    ;
    } 
}
