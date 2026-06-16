package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Helperfunctions.Location;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.aDrivetrain;

import java.util.List;


@Autonomous

public class RedFarAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Intake intake;
    private Spindex spindex;
    private Turret turret;


    public enum fbPathState{
        //STARTPosition -->EndPosition
        START,
        SHOOTPRELOAD,
        SHOOTPRELOAD_STARTINTAKE,
        STARTINTAKE_ENDINTAKE,
        ENDINTAKE_SHOOT,
        SHOOT2,
        OFFLINE,
        DONE,



    }

    private final Pose startPose = new Pose(87.4, 7.76, Math.toRadians (90));
    private final Pose startIntake = new Pose (101.5, 35, Math.toRadians(0));
    private final Pose endIntakeSpike = new Pose (137.6,35,Math.toRadians(0));
    private final Pose shoot = new Pose (86.53, 22.79,Math.toRadians(0));
    private final Pose end = new Pose (86.71,34.8,Math.toRadians(0));


    fbPathState pathState;

    private PathChain  start_startIntake, startIntake_endIntake, endIntakeSpike_shoot, shoot_end;

    public void buildPaths(){
        //put in coordinates for start pose and end pose

        start_startIntake = follower.pathBuilder()
                .addPath(new BezierLine(startPose,startIntake))
                .setLinearHeadingInterpolation(startPose.getHeading(),startIntake.getHeading()).build();
        startIntake_endIntake  = follower.pathBuilder()
                .addPath(new BezierLine(startIntake,endIntakeSpike))
                .setLinearHeadingInterpolation(startIntake.getHeading(),endIntakeSpike.getHeading(),0.4).build();
        endIntakeSpike_shoot = follower.pathBuilder()
                .addPath(new BezierLine(endIntakeSpike,shoot ))
                .setLinearHeadingInterpolation(endIntakeSpike.getHeading(), shoot.getHeading()).build();
        shoot_end = follower.pathBuilder()
                .addPath(new BezierLine(shoot, end))
                .setLinearHeadingInterpolation(shoot.getHeading(), end.getHeading()).build();


    }
    public void statePathUpdate() {
        switch (pathState) {


            case SHOOTPRELOAD:

                intake.shootBalls();
                intake.setIntakePower(1);

                if (turret.getFlywheelVelocity() < -1500) {
                    spindex.runSpindexToggle(1);

                    if (opModeTimer.getElapsedTimeSeconds() > 7) {
                        spindex.goToPosition(344);
                        opModeTimer.resetTimer();
                        follower.followPath(start_startIntake);
                        spindex.goToPosition(344);
                        setPathState(fbPathState.SHOOTPRELOAD_STARTINTAKE);
                    }
                }
                break;

            case SHOOTPRELOAD_STARTINTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(startIntake_endIntake);
                    setPathState(fbPathState.STARTINTAKE_ENDINTAKE);
                    follower.setMaxPower(0.6);
                }
                break;


            case STARTINTAKE_ENDINTAKE:
                spindex.goToPosition(344);
                intake.intakeBalls();

                if (!follower.isBusy()) {
                    opModeTimer.resetTimer();

                    intake.setIntakePower(1);
                    follower.followPath(endIntakeSpike_shoot);
                    follower.setMaxPower(1);
                    setPathState(fbPathState.SHOOT2);


                }
                break;


            case ENDINTAKE_SHOOT:
                if (!follower.isBusy()) {
                    opModeTimer.resetTimer();

                    intake.setIntakePower(1);
                    setPathState(fbPathState.SHOOT2);

                }

                break;

            case SHOOT2:
                spindex.runSpindexToggle(1);
                if (opModeTimer.getElapsedTimeSeconds() > 3.5) {
                    follower.followPath(shoot_end);
                    setPathState(fbPathState.DONE);
                }
                break;


            case DONE:
                if (!follower.isBusy()) {
                    telemetry.addLine("Finished");
                }
                break;

            default:
                telemetry.addLine("No state Commanded");
                break;

        }

    }
    public void setPathState(fbPathState newState){
        pathState=newState;
        pathTimer.resetTimer();
    }



    @Override
    public void init(){
        pathState = fbPathState.SHOOTPRELOAD;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        intake = new Intake(hardwareMap);
        spindex = new Spindex (hardwareMap);
        turret = new Turret(hardwareMap, true,follower,true);
        turret.resetTurret();


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
        Location.START = follower.getPose();
        Location.turretPosition=turret.getTurretPosition();
        follower.update();
        turret.updateFollower(follower);
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = Math.toDegrees(follower.getPose().getHeading());
        turret.runTurret();


        telemetry.addData("X:", x);
        telemetry.addData("Y:", y);
        telemetry.addData("H:", h);

        turret.runTurret();


        statePathUpdate();
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Turret Aim", turret.getTargetAngle (follower.getPose().getX(),follower.getPose().getY()));
        telemetry.addData("pathState", pathState.toString());
        telemetry.addData ("turret Target", turret.getTargetAngle(x,y));
    }
}



