package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Locale;
@Config
public class AprilTag {
    // Create the AprilTag processor.
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static double cameraCorrection = 1.085;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    HardwareMap hwMap;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    public AprilTag() {}
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(706.965424613, 706.965424613, 316.107943962, 246.246519532)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
            int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            FtcDashboard.getInstance().startCameraStream(camera, 0);

        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
    private AprilTagPoseFtc getPose(AprilTagDetection detection){
        Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        // Original source data
        double x = detection.rawPose.x;
        double y = detection.rawPose.y;
        double z = detection.rawPose.z;

        double yaw = rot.firstAngle;
        double roll = rot.secondAngle;
        double pitch = rot.thirdAngle;

        return new AprilTagPoseFtc(x,y,z,yaw,pitch,roll,Math.hypot(x, y),
                AngleUnit.DEGREES.fromUnit(AngleUnit.RADIANS, Math.atan2(-x, y)),
                AngleUnit.DEGREES.fromUnit(AngleUnit.RADIANS, Math.atan2(z, y)));
    }
    private AprilTagPoseFtc getTag(int tag){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == tag) {
                    return getPose(detection);
                }
            }
        }
        return null;
    }
    public double getRange(int tag){
        AprilTagPoseFtc d = getTag(tag);
        if(d!=null){
            return d.range*DistanceUnit.mmPerInch*cameraCorrection;
        }
        return 0;
    }
    public double getYaw(int tag){
        AprilTagPoseFtc d = getTag(tag);
        if(d!=null){
            return d.bearing;
        }
        return 0;
    }
    public boolean isTag(int tag){
        AprilTagPoseFtc d = getTag(tag);
        return d!=null;
    }
    public void telemetryAprilTag(Telemetry telemetry) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.rawPose != null) {

                AprilTagPoseFtc pose = getPose(detection);


                telemetry.addLine(String.format(Locale.getDefault(), "\n==== (ID %d)", detection.id));
                telemetry.addLine(String.format(Locale.getDefault(), "XYZ %6.1f %6.1f %6.1f  (mm)", pose.x*DistanceUnit.mmPerInch*cameraCorrection, pose.y*DistanceUnit.mmPerInch*cameraCorrection, pose.z*DistanceUnit.mmPerInch*cameraCorrection));
                telemetry.addLine(String.format(Locale.getDefault(), "PRY %6.1f %6.1f %6.1f  (deg)", pose.pitch, pose.roll, pose.yaw));
                telemetry.addLine(String.format(Locale.getDefault(), "RBE %6.1f %6.1f %6.1f  (mm, deg, deg)", pose.range*DistanceUnit.mmPerInch*cameraCorrection, pose.bearing, pose.elevation));
            } else {
                telemetry.addLine(String.format(Locale.getDefault(), "\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format(Locale.getDefault(), "Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
    public void close(){
        visionPortal.close();
    }
}   // end method initAprilTag()
