package org.firstinspires.ftc.teamcode.Robot;


public class FunnyRobot {
    public static final FunnyRobot inst = new FunnyRobot();

    public enum opmode {
        AUTO, TELEOP

    }
    public enum Alliance {
        RED, BlUE
    }

    public static Alliance alliance = Alliance.BlUE;

    //public static opmode opmode = Mode.AUTO;

    //hardware

    //Telemetry

}
