package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Robot.FunnyRobot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class AutoShoot extends CommandBase {
    private final FunnyRobot robot = FunnyRobot.get();
    private Timing.Timer timer;

    public AutoShoot(double spinUpTime){
        addRequirements(robot.shooter, robot.intake);
        timer = new Timing.Timer((long)spinUpTime);
    }

    @Override
    public void initialize(){
        timer.start();
        robot.shooter.setMode(Shooter.ShooterMode.PIDF);
    }

    @Override
    public void execute(){
        //if (robot.shooter.)

    }

    @Override
    public boolean isFinished(){
        boolean done = timer.done();
        return timer.done();
        //if (done){
            //robot.shooter.();
        }
        //return  done;
    }

