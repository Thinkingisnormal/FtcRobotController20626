package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.lang.Math;
import java.util.List;
public class AprilTagLimelight  {
    private Limelight3A limelight;
    private IMU imu;

    private Telemetry telemetry;

    private double calcedDistance;

    public void LimeInit(HardwareMap hardwareMap, IMU imu, Telemetry telemetry) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;
        this.imu = imu;
        limelight.pipelineSwitch(0);
        limelight.start(); // swap to start() if you want to conserve battery.
    }

    public void Stop() {
        limelight.stop();
    }

    public void aprilTagLoop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw()); // tells bot what our current yaw/heading is
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tx",llResult.getTx()); // target x
            telemetry.addData("Ty",llResult.getTy()); // target y
            telemetry.addData("Ta",llResult.getTa()); // area of target in pixels
            telemetry.addData("botpose", botPose.toString());
            this.calcedDistance = calcDistance(llResult.getTy());
            telemetry.addData("Distance (in):", this.calcedDistance);

        }
        else {
            telemetry.addLine("Apriltag is not in view.");
        }

//        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
//        for (LLResultTypes.FiducialResult fiducial : fiducials) {
//            int id = fiducial.getFiducialId(); // The ID number of the fiducial
//            double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
//            double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
//            telemetry.addData("botpose",fiducial.getRobotPoseTargetSpace());
//            calcDistance(llResult.getTy());
//        }

    }
   public double calcDistance (double ty) {

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 11.5;

        // distance from the target to the floor
        double goalHeightInches = 28.5;

        double angleToGoalDegrees = limelightMountAngleDegrees + ty;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
       return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }




    //rumbles gamepad if distances is in range of min and max to notify the driver they are in good spot.

    public void rangeRumble (Gamepad g1, Gamepad g2) {

        final double goalHeight = 30 * 0.0254; // inches to meters (PLEASE MEASURE AS THIS IS ESTIMATION)
        final double botHeight  = 12 * 0.0254; // inches to meters (PLEASE MEASURE AS THIS IS ESTIMATION)

        double iVelocity = 6.36; // m/s SHOULDN'T BE HARDCODED
        double botVelocity = 0;  // m/s SHOULDN'T BE HARDCODED; will get velocity from odometry

        final double angle = Math.toRadians(15);

        final double g = 9.81;
        final double deltaY = goalHeight - botHeight;

        double vy = iVelocity * Math.sin(angle);
        double vx = iVelocity * Math.cos(angle) + botVelocity;

        double time = (vy + Math.sqrt(vy*vy + 2*g*deltaY)) / g;
        double range = vx * time;


        double MIN_DISTANCE = range * 0.95;
        double MAX_DISTANCE = range * 1.05;

        if ((calcedDistance > MIN_DISTANCE) && (calcedDistance < MAX_DISTANCE)) {
            g1.rumble(200);
            g2.rumble(200);
        } else {g1.stopRumble();g2.rumble(200);}
    }


}
