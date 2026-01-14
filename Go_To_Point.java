package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous

public class Go_To_Point extends OpMode{

    private Follower followPath;
    private Timer timerA, timerB;

    public enum pathStuff {
        //Drive to Position 1, Drive to Position 2
        DriveSomeWhere, TurnAndDrive
    }
    pathStuff pathState;

    private final Pose beginCord = new Pose(20.242285714285714,122.7702857142857,Math.toRadians(135));
    private final Pose endCord = new Pose(72.08228571428572, 71.91771428571428, Math.toRadians(45));

    private final Pose newEnd = new Pose(102, 122, Math.toRadians(90));
    private PathChain usingCordsFromAbove;
    private PathChain secondPath;
    public void buildPaths(){
        usingCordsFromAbove = follower.pathBuilder()
                .addPath(new BezierLine(beginCord,endCord))
                .setLinearHeadingInterpolation(beginCord.getHeading(), endCord.getHeading())
                .build()
        ;
        secondPath = follower.pathBuilder()
                .addPath(new BezierLine(endCord, newEnd))
                .setLinearHeadingInterpolation(endCord.getHeading(), newEnd.getHeading())
                .build()
                ;


    }

    public void statePathUpdate(){
        switch(pathState){
            case DriveSomeWhere:
                follower.followPath(usingCordsFromAbove, true);
                transferThePathStuff(pathStuff.TurnAndDrive);
                break;

            case TurnAndDrive:
                //Check if the follower is busy
                if(!follower.isBusy() && timerA.getElapsedTimeSeconds()>5){
                    follower.followPath(secondPath, true);

                    //Run stuff
                    //Wait five seconds because why not?
                    telemetry.addLine("Bonjour, you would run code here");
                    //add to transition to another state if necessary as above
                }
                break;

            default:
                //add what the program should do by default if none of the other switch cases occur
                telemetry.addLine("I have not received a command yet");
                break;
        }
    }
public void transferThePathStuff(pathStuff AnotherState){
        pathState = AnotherState;
        timerA.resetTimer();
}
    @Override
    public void init(){
        pathState = pathStuff.DriveSomeWhere;
        timerA = new Timer();
        timerB = new Timer();
        follower = Constants.createFollower(hardwareMap);

        //Add other initialization mechanisms

        buildPaths();
        follower.setPose(beginCord);
    }

    public void start(){
        timerB.resetTimer();
        transferThePathStuff(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

    }
}
