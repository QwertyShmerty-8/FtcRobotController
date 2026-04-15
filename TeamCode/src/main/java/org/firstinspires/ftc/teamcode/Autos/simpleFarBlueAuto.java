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

public class simpleFarBlueAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Intake intake;
    private Spindex spindex;
    private Turret turret;

    public enum PathState{
        //STARTPosition -->EndPosition

        SHOOTPRELOAD,
        SHOOTPRELOADTOEND,
        ENDPOSITION,
    }

    PathState pathState;
    private final Pose startPose = new Pose(56.233,8.000, Math.toRadians (90));
    private final Pose endPose = new Pose (56.69,34.60,Math.toRadians(90));
    private PathChain driveStartPosEndPos;

    public void buildPaths(){
        //put in coordinates for start pose and end pose
        driveStartPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading()).build();
    }
    public void statePathUpdate(){
        switch(pathState){
            case  SHOOTPRELOAD:
                turret.setFlyWheelSpeed(-1200);

                if(turret.getFlyWheelSpeed()<0) {
                    intake.shootBalls();
                    intake.setIntakePower(1);
                    spindex.setSpindexPower(1);
                    opModeTimer.resetTimer();
                    setPathState(PathState.SHOOTPRELOADTOEND);
                }
                break;
            case SHOOTPRELOADTOEND:

                if (opModeTimer.getElapsedTimeSeconds()>2){
                    spindex.setSpindexPower(0);
                    follower.followPath(driveStartPosEndPos);
                    setPathState(PathState.ENDPOSITION);

                }
                break;
            case ENDPOSITION:
                if (!follower.isBusy()){
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

        pathState=PathState.SHOOTPRELOAD;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        intake = new Intake(hardwareMap);
        spindex = new Spindex (hardwareMap);
        turret = new Turret(hardwareMap, "blue",0,true);


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
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = follower.getPose().getHeading();

        telemetry.addData("X:", x);
        telemetry.addData("Y:", y);
        telemetry.addData("H:", h);



        turret.autoHoodAnglelut(x, y);

        turret.aimTurret(x, y, h);

        follower.update();
        statePathUpdate();
        telemetry.addData("pathState", pathState.toString());
    }
}



