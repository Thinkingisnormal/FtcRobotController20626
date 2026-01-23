package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Robot.FunnyRobot;

@Configurable
public class Booster extends SubsystemBase {

    public enum BoosterState {
        REST,
        FIRE
    }

    private BoosterState state = BoosterState.REST;

    // Tunable servo positions
    public static double LEFT_REST_POS = 0.0;
    public static double LEFT_FIRE_POS = 0.35;

    public static double RIGHT_REST_POS = 1.0;
    public static double RIGHT_FIRE_POS = 0.65;

    public Booster() {}

    public void rest() {
        state = BoosterState.REST;
    }

    public void fire() {
        state = BoosterState.FIRE;
    }

    @Override
    public void periodic() {
        FunnyRobot robot = FunnyRobot.get();

        switch (state) {
            case REST:
                robot.boosterLeft.set(LEFT_REST_POS);
                robot.boosterRight.set(RIGHT_REST_POS);
                break;

            case FIRE:
                robot.boosterLeft.set(LEFT_FIRE_POS);
                robot.boosterRight.set(RIGHT_FIRE_POS);
                break;
        }
    }
}

