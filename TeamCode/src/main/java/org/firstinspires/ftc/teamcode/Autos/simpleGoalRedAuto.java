package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.aDrivetrain;


@Autonomous

public class simpleGoalRedAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Intake intake;
    private Spindex spindex;
    private Turret turret;

    public enum PathState{
        //STARTPosition -->EndPosition

        STARTPOSETOSHOOTPOSE,
        SHOOT,
        SHOOTPOSETOENDPOSE,
    }

    PathState pathState;
    private final Pose startPose = new Pose(123, 123.000, Math.toRadians (36));
    private final Pose shootPose = new Pose (102.83,102.40,Math.toRadians(36));
    private final Pose endPose = new Pose (102.08,139.31);
    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    public void buildPaths(){
        //put in coordinates for start pose and end pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading()).build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine (shootPose, endPose))
                .build();
    }
    public void statePathUpdate(){
        switch(pathState){
            case  STARTPOSETOSHOOTPOSE:
                turret.setFlyWheelSpeed(-1200);
                follower.followPath(driveStartPosShootPos);
                opModeTimer.resetTimer();
                setPathState(PathState.SHOOT);

                break;
            case SHOOT:
                intake.shootBalls();
                intake.setIntakePower(1);
                spindex.setSpindexPower(1);

                if (opModeTimer.getElapsedTimeSeconds()>2){
                    spindex.setSpindexPower(0);
                    opModeTimer.resetTimer();
                    follower.followPath(driveShootPosEndPos);
                    setPathState(PathState.SHOOTPOSETOENDPOSE);
                }
                break;
            case SHOOTPOSETOENDPOSE:
                if (!follower.isBusy()){
                    opModeTimer.resetTimer();
                    follower.holdPoint(endPose);
                    telemetry.addLine("Finished");
                }
                break;

            default:
                telemetry.addLine("No state COmmanded");
                break;

        }
    }
    public void setPathState(PathState newState){
        pathState=newState;
        pathTimer.resetTimer();
    }



    @Override
    public void init(){
        pathState=PathState.STARTPOSETOSHOOTPOSE;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        intake = new Intake(hardwareMap);
        spindex = new Spindex (hardwareMap);
        turret = new Turret(hardwareMap, "red",0,true);


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);

    }
    @Override
    public void loop(){
        follower.update();
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = follower.getPose().getHeading();

        telemetry.addData("X:", x);
        telemetry.addData("Y:", y);
        telemetry.addData("H:", h);



        turret.autoHoodAnglelut(x, y);

        turret.aimTurretOriginal(x, y, h);


        statePathUpdate();
        telemetry.addData("pathState", pathState.toString());
        telemetry.addData ("turret Target", turret.getTargetBlue(x,y));

    }
}