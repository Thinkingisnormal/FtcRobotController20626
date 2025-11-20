package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*;


import com.google.blocks.ftcrobotcontroller.runtime.*;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.*;


import java.util.*;

public class LimeLight {

    Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        LLResult result = null;
        result.getPipelineIndex();

        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        // Sending numbers to Python
        double[] inputs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
        limelight.updatePythonInputs(inputs);

        // Getting numbers from Python
        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            double firstOutput = pythonOutputs[0];
            telemetry.addData("Python output:", firstOutput);
        }

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            }
        }

        // First, tell Limelight which way your robot is facing
        IMU imu = hardwareMap.get(IMU.class, "limelight");
        double robotYaw = imu.getRobotOrientation().firstAngle;
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }

        List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
        LLResultTypes.DetectorResult detection;
        for (LLResultTypes.ColorResult colorTarget : colorTargets) {
            double x = detection.getTargetXDegrees(); // Where it is (left-right)
            double y = detection.getTargetYDegrees(); // Where it is (up-down)
            double area = colorTarget.getTargetArea(); // size (0-100)
            telemetry.addData("Color Target", "takes up " + area + "% of the image");
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double x = detection.getTargetXDegrees(); // Where it is (left-right)
            double y = detection.getTargetYDegrees(); // Where it is (up-down)
            double StrafeDistance_3D;
            StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y;();
            telemetry.addData("Fiducial " + id, "is " + x + " meters away");
        }


        LLResultTypes.FiducialResult fiducial;
        fiducial.getRobotPoseTargetSpace(); // Robot pose relative it the AprilTag Coordinate System (Most Useful)
        fiducial.getCameraPoseTargetSpace(); // Camera pose relative to the AprilTag (useful)
        fiducial.getRobotPoseFieldSpace(); // Robot pose in the field coordinate system based on this tag alone (useful)
        fiducial.getTargetPoseCameraSpace(); // AprilTag pose in the camera's coordinate system (not very useful)
        fiducial.getTargetPoseRobotSpace(); // AprilTag pose in the robot's coordinate system (not very useful)

        List<LLResultTypes.BarcodeResult> barcodes = result.getBarcodeResults();
        for (LLResultTypes.BarcodeResult barcode : barcodes) {
            String data = barcode.getData(); // What the barcode says
            String family = barcode.getFamily(); // What type of barcode it is
            telemetry.addData("Barcode", data + " (" + family + ")");
        }

        List<LLResultTypes.ClassifierResult> classifications = result.getClassifierResults();
        for (LLResultTypes.ClassifierResult classification : classifications) {
            String className = classification.getClassName(); // What Limelight thinks it sees
            double confidence = classification.getConfidence(); // Confidence Score
            telemetry.addData("I see a", className + " (" + confidence + "%)");
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        for (detections) {
            String className = detection.getClassName(); // What was detected
            double x = detection.getTargetXDegrees(); // Where it is (left-right)
            double y = detection.getTargetYDegrees(); // Where it is (up-down)
            telemetry.addData(className, "at (" + x + ", " + y + ") degrees");
        }

        long staleness = result.getStaleness();
        if (staleness < 100) { // Less than 100 milliseconds old
            telemetry.addData("Data", "Good");
        } else {
            telemetry.addData("Data", "Old (" + staleness + " ms)");
        }

        LLFieldMap fieldMap = new LLFieldMap(); // You'll need to fill this with field data
        boolean success = limelight.uploadFieldmap(fieldMap, null); // null means use the default slot
        if (success) {
            telemetry.addData("Field Map", "Uploaded successfully!");
        } else {
            telemetry.addData("Field Map", "Oops, upload failed");
        }

        limelight.captureSnapshot("auto_pov_10s");

        limelight.deleteSnapshots();
        telemetry.addData("Snapshots", "All cleared out!");
    }
}