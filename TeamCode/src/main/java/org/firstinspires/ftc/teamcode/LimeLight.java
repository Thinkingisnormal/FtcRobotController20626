package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@Autonomous
public class LimeLight extends OpMode {
    Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        IMU imu = hardwareMap.get(IMU.class, "imu");
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
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
        assert result != null;
        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            double firstOutput = pythonOutputs[0];
            telemetry.addData("Python output:", firstOutput);
        }

        if (result.isValid()) {
            Pose3D botPose = result.getBotpose();
            if (botPose != null) {
                double x = botPose.getPosition().x;
                double y = botPose.getPosition().y;
                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            }
        }

        // First, tell Limelight which way your robot is facing
        if (result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }
        //Color Results
        List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        for (int i = 0; i < colorTargets.size(); i++) {
            LLResultTypes.ColorResult colorTarget = colorTargets.get(i);
            LLResultTypes.DetectorResult det = detections.get(i);

            double x = det.getTargetXDegrees(); // horizontal angle
            double y = det.getTargetYDegrees(); // vertical angle
            double area = colorTarget.getTargetArea(); // the area of the object

            // identify the colors
            String colorName = det.getClassName(); // color class name

            if (colorName.equalsIgnoreCase("Green"))
            {
                telemetry.addData("Green Target", "at (" + x + ", " + y + ") degrees, area: " + area + "%");
            }
            else if (colorName.equalsIgnoreCase("Purple"))
            {
                telemetry.addData("Purple Target", "at (" + x + ", " + y + ") degrees, area: " + area + "%");
            }
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            Pose3D robotPoseTarget = fiducial.getRobotPoseTargetSpace(); // Robot pose relative it the AprilTag Coordinate System (Most Useful)

            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double distance = robotPoseTarget.getPosition().x; // distance from tag
            double StrafeDistance_3D = robotPoseTarget.getPosition().y;
            telemetry.addData("Fiducial " + id, "is " + distance + " meters away");

        }

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
        detections = result.getDetectorResults();
        for (LLResultTypes.DetectorResult det : detections) {
            String className = det.getClassName(); // What was detected
            double x = det.getTargetXDegrees(); // Where it is (left-right)
            double y = det.getTargetYDegrees(); // Where it is (up-down)
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
        telemetry.update();
    }
}