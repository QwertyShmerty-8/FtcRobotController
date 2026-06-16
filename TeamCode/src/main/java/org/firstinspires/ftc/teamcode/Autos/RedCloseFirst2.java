package org.firstinspires.ftc.teamcode.Autos;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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

public class RedCloseFirst2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Intake intake;
    private Spindex spindex;
    private Turret turret;
    private Limelight3A limelight;
    int idb=21;
    double ppgGoToPosition;
    double pgpGoToPosition;
    double gppGoToPosition;

    double shootPreloadSpeed = -1140;
    double preloadHood= 0.35;
    //target 1150
    double ppgSpeed = -1100;
    double ppgHood= 0.53;

    double pgpSpeed= -1200;
    double pgpHood = 0.33;

    double gppSpeed = 1200;
    double gppHood= 0.53;

    double targetFlywheelSpeed;
    double targetHoodAngle;

    public enum PathStateB{
        //STARTPosition -->EndPosition
        START,

        DETECTAPRILTAG,

        START_SHOOTPRELOAD,
        SORTPRELOAD,
        SHOOTPRELOAD_STARTINTAKEPPG,

        STARTINTAKEPPG_ENDINTAKEPPG,
        ENDINTAKEPPG_STARTCLEAR1,
        STARTCLEAR1_ENDCLEAR1,
        ENDCLEAR1_SHOOTPPG,
        ENDINTAKEPPG_CLEAR1,
        CLEAR1_SHOOTPPG,
        SHOOTGPP,
        SHOOTPPG_STARTINTAKEPGP,
        STARTINTAKEPGP_ENDINTAKEPGP,
        ENDINTAKEPGP_SHOOTPGP,
        SHOOTPGP,
        SHOOTPGP_STARTINTAKEGPP,
        STARTINTAKEGPP_ENDINTAKEGPP,
        ENDINTAKEGPP_SHOOTGPP,
        SHOOTPPG,
        SHOOTGPP_END,
        DONE,
    }
    PathStateB pathState;
    private final Pose startPose = new Pose(120, 124.000, Math.toRadians (40));
    private final Pose shootPreloadPose = new Pose (99.887, 99.603,Math.toRadians(0));
    private final Pose startIntakeGPPPose = new Pose (93.648, 84.071,Math.toRadians(0));
    private final Pose endIntakeGPPPose = new Pose (125.816, 83.184,Math.toRadians(0));
    private final Pose startClear1 = new Pose (110, 75,Math.toRadians(0));
    private final Pose endClear1  = new Pose (125,77.5,Math.toRadians(0));
    private final Pose shootGPPPose = new Pose (101.6,88,Math.toRadians(0));
    private final Pose shootPPGstartIntakePGPcontrolPoint = new Pose (96.29,71.302,Math.toRadians(0));
    private final Pose startIntakePGPPose = new Pose (93.156,58.73,Math.toRadians(0));
    private final Pose endIntakePGPPose = new Pose (133,57.119,Math.toRadians(0));
    private final Pose shootPGPPose = new Pose (100,88,Math.toRadians(0));//new Pose (60.923, 84.046,Math.toRadians(180));
    private final Pose startIntakePPG = new Pose (93.156,35.707, Math.toRadians(0));
    private final Pose endIntakePPG = new Pose (130,35.330, Math.toRadians(0));
    private final Pose shootPPG = new Pose (101.6,88,Math.toRadians(0));
    //new Pose (60.476,83.973, Math.toRadians(180));
    private final Pose end = new Pose (95.7,125.9,Math.toRadians(0));

    private PathChain startPos_ShootPreloadPos, shootPreload_startIntakePPG, startIntakePPG_endIntakePPG;
    private PathChain endIntakePPG_shootPPG, shootPPG_startIntakePGP, startIntakePGP_endIntakePGP;
    private PathChain endIntakePGP_shootPGP, shootPGP_startIntakeGPP,startIntakeGPP_endIntakeGPP;
    private PathChain endIntakeGPP_shootGPP,endIntakePPG_startClear1, startClear1_endClear1, endClear1_shootPPG;
    private PathChain shootPPG_end;


    public void buildPaths(){
        //put in coordinates for start pose and end pose
        startPos_ShootPreloadPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPreloadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPreloadPose.getHeading()).build();
        shootPreload_startIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine (shootPreloadPose, startIntakeGPPPose))
                .setLinearHeadingInterpolation(shootPreloadPose.getHeading(), startIntakeGPPPose.getHeading())
                .build();
        startIntakePPG_endIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine (startIntakeGPPPose, endIntakeGPPPose))
                .setLinearHeadingInterpolation(startIntakeGPPPose.getHeading(), endIntakeGPPPose.getHeading())
                .build();
        endIntakePPG_startClear1 = follower.pathBuilder().addPath(new BezierLine (endIntakeGPPPose, startClear1))
                .setLinearHeadingInterpolation(endIntakeGPPPose.getHeading(), startClear1.getHeading())
                .build();
        startClear1_endClear1 = follower.pathBuilder().addPath(new BezierLine (startClear1, endClear1))
                .setLinearHeadingInterpolation(endIntakeGPPPose.getHeading(), endClear1.getHeading())
                .build();
        endClear1_shootPPG = follower.pathBuilder().addPath(new BezierLine (endClear1,shootPPG))
                .setLinearHeadingInterpolation(endClear1.getHeading(),shootPPG.getHeading())
                .build();
        shootPPG_end = follower.pathBuilder()
                .addPath(new BezierLine (shootPPG, end))
                .setLinearHeadingInterpolation(shootPPG.getHeading(), end.getHeading())
                .build();

        shootPPG_startIntakePGP = follower.pathBuilder()
                .addPath(new BezierCurve(shootGPPPose, shootPPGstartIntakePGPcontrolPoint,startIntakePGPPose))
                .setLinearHeadingInterpolation(shootGPPPose.getHeading(), startIntakePGPPose.getHeading())
                .build();
        startIntakePGP_endIntakePGP = follower.pathBuilder()
                .addPath(new BezierLine (startIntakePGPPose, endIntakePGPPose))
                .setLinearHeadingInterpolation(startIntakePGPPose.getHeading(), endIntakePGPPose.getHeading())
                .build();
        endIntakePGP_shootPGP = follower.pathBuilder()
                .addPath(new BezierCurve(endIntakePGPPose, new Pose(31.5,60),shootPGPPose))
                .setLinearHeadingInterpolation(endIntakePGPPose.getHeading(), shootPGPPose.getHeading())
                .build();
        shootPGP_startIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine (shootPGPPose, startIntakePPG))
                .setLinearHeadingInterpolation(shootPGPPose.getHeading(), startIntakePPG.getHeading())
                .build();
        startIntakeGPP_endIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine (startIntakePPG, endIntakePPG))
                .setLinearHeadingInterpolation(startIntakePPG.getHeading(), endIntakePPG.getHeading())
                .build();
        endIntakeGPP_shootGPP = follower.pathBuilder()
                .addPath(new BezierLine (endIntakePPG, end))
                .setLinearHeadingInterpolation(endIntakePPG.getHeading(), end.getHeading())
                .build();
        shootPPG_end = follower.pathBuilder()
                .addPath(new BezierLine (shootPPG, end))
                .setLinearHeadingInterpolation(shootPPG.getHeading(), end.getHeading(),0.5)
                .build();




    }
    public void statePathUpdate() {
        switch (pathState) {
            case START:
                targetFlywheelSpeed = shootPreloadSpeed;
                targetHoodAngle = preloadHood;
                intake.setIntakePower(1);

                if (pathTimer.getElapsedTimeSeconds() < 0.1) {

                    follower.followPath(startPos_ShootPreloadPos);
                }
                if (!follower.isBusy() && turret.getFlywheelVelocity() > turret.getFlywheelVelocity() - 75) {
                    intake.shootBalls();
                    opModeTimer.resetTimer();
                    spindex.startRotate(1);
                    setPathState(PathStateB.START_SHOOTPRELOAD);


                }
                break;
            case START_SHOOTPRELOAD:
                intake.shootBalls();


                if (!follower.isBusy()) {
                    intake.shootBalls();
                    if (opModeTimer.getElapsedTimeSeconds() > 1.5) {
                        spindex.runSpindexToggleAuto(1);
                    }
                    if (opModeTimer.getElapsedTimeSeconds() > 4.5) {
                        opModeTimer.resetTimer();
                        follower.followPath(shootPreload_startIntakePPG);
                        setPathState(PathStateB.SHOOTPRELOAD_STARTINTAKEPPG);
                    }
                }




                break;
            case SHOOTPRELOAD_STARTINTAKEPPG:
                intake.intakeBalls();
                targetFlywheelSpeed = ppgSpeed;
                targetHoodAngle = ppgHood;

                if (!follower.isBusy()){
                    opModeTimer.resetTimer();
                    follower.setMaxPower(0.75);
                    spindex.goToPosition(344);
                    follower.followPath(startIntakePPG_endIntakePPG);

                    intake.setIntakePower(1);
                    setPathState(PathStateB.STARTINTAKEPPG_ENDINTAKEPPG);

                }
                break;
            case STARTINTAKEPPG_ENDINTAKEPPG:
                spindex.goToPosition(344);
                intake.intakeBalls();
                follower.setMaxPower(0.75);
                if (!follower.isBusy()){
                    opModeTimer.resetTimer();
                    follower.followPath(endIntakePPG_startClear1);
                    setPathState(PathStateB.ENDINTAKEPPG_STARTCLEAR1);
                    follower.setMaxPower(1);
                    intake.setIntakePower(0);
                }
                break;

            case ENDINTAKEPPG_STARTCLEAR1:
                intake.setIntakePower(0);
                spindex.goToPosition(344);


                if (!follower.isBusy()){
                    follower.setMaxPower(0.5);
                    opModeTimer.resetTimer();
                    follower.followPath(startClear1_endClear1);
                    setPathState(PathStateB.STARTCLEAR1_ENDCLEAR1);


                }
                break;

            case STARTCLEAR1_ENDCLEAR1:
                intake.setIntakePower(1);
                spindex.goToPosition(344);

                if (!follower.isBusy()&& opModeTimer.getElapsedTimeSeconds()>1.5){
                    opModeTimer.resetTimer();
                    follower.followPath(endClear1_shootPPG);

                    setPathState(PathStateB.ENDCLEAR1_SHOOTPPG);
                    follower.setMaxPower(1);

                }
                break;

            case ENDCLEAR1_SHOOTPPG:





                follower.setMaxPower(1);
                if (!follower.isBusy()){

                    intake.shootBalls();
                    intake.shootBalls();
                    spindex.runSpindexToggleAuto(1);
                    opModeTimer.resetTimer();

                    setPathState(PathStateB.SHOOTPPG);



                }

                break;
            case SHOOTPPG:
                intake.shootBalls();
                spindex.runSpindexToggleAuto(1);

                if (opModeTimer.getElapsedTimeSeconds()>3.5) {
                    spindex.goToPosition(344);
                    targetFlywheelSpeed= ppgSpeed;
                    targetHoodAngle= pgpHood;
                    follower.followPath(shootPPG_startIntakePGP);
                    setPathState(PathStateB.SHOOTPPG_STARTINTAKEPGP);


                }
                break;


            case SHOOTPPG_STARTINTAKEPGP:
                intake.intakeBalls();
                spindex.goToPosition(344);
                if (!follower.isBusy()){
                    follower.setMaxPower(0.75);

                    opModeTimer.resetTimer();
                    follower.followPath(startIntakePGP_endIntakePGP);
                    follower.setMaxPower(0.5);
                    setPathState(PathStateB.STARTINTAKEPGP_ENDINTAKEPGP);
                }
                break;
            case STARTINTAKEPGP_ENDINTAKEPGP:

                spindex.goToPosition(344);
                intake.intakeBalls();
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    opModeTimer.resetTimer();
                    follower.setMaxPower(1);
                    follower.followPath(endIntakePGP_shootPGP);
                    setPathState(PathStateB.ENDINTAKEPGP_SHOOTPGP);


                }
                break;

            case ENDINTAKEPGP_SHOOTPGP:
                spindex.goToPosition(344);

                if (!follower.isBusy()){
                    intake.shootBalls();
                    spindex.startRotate(1);

                    opModeTimer.resetTimer();
                    targetFlywheelSpeed = ppgSpeed;
                    targetHoodAngle = ppgHood;

                    setPathState(PathStateB.SHOOTPGP);


                }
                break;
            case SHOOTPGP:
                intake.shootBalls();
                intake.shootBalls();
                spindex.runSpindexToggleAuto(1);
                if (opModeTimer.getElapsedTimeSeconds() > 2.5 && opModeTimer.getElapsedTimeSeconds() < 3.5) {
                    spindex.runSpindexToggleAuto(1);
                }
                if (opModeTimer.getElapsedTimeSeconds() > 3.5) {
                    spindex.goToPosition(344);
                    targetFlywheelSpeed= ppgSpeed;
                    targetHoodAngle= pgpHood;
                    follower.followPath(shootPPG_end);
                    setPathState(PathStateB.DONE);
                }
                break;
            case SHOOTPGP_STARTINTAKEGPP:
                intake.intakeBalls();
                spindex.goToPosition(344);
                if (!follower.isBusy()){
                    follower.setMaxPower(1);
                    opModeTimer.resetTimer();
                    follower.followPath(startIntakeGPP_endIntakeGPP);
                    setPathState(PathStateB.STARTINTAKEGPP_ENDINTAKEGPP);
                }
                break;
            case STARTINTAKEGPP_ENDINTAKEGPP:
                intake.intakeBalls();
                spindex.goToPosition(344);
                if (!follower.isBusy()){
                    follower.setMaxPower(1);

                    opModeTimer.resetTimer();
                    follower.followPath(endIntakeGPP_shootGPP);
                    spindex.startRotate(1);
                    setPathState(PathStateB.ENDINTAKEGPP_SHOOTGPP);
                }
                break;
            case ENDINTAKEGPP_SHOOTGPP:



                if (!follower.isBusy()){
                    setPathState(PathStateB.DONE);
                    opModeTimer.resetTimer();
                    targetFlywheelSpeed= ppgSpeed;
                    targetHoodAngle= pgpHood;




                }
                break;

            case SHOOTGPP:
                intake.shootBalls();
                spindex.runSpindexToggleAuto(1);

                if (opModeTimer.getElapsedTimeSeconds()>3.5) {

                    follower.followPath(shootPPG_end );
                    setPathState(PathStateB.SHOOTGPP_END);


                }
                break;

            case SHOOTGPP_END:
                spindex.runSpindexToggleAuto(1);


                break;

            case DONE:
                spindex.goToPosition(344);
                telemetry.addLine("Finished");
                break;


            default:
                telemetry.addLine("No state COmmanded");
                break;

        }
    }
    public void setPathState(PathStateB newState){
        pathState=newState;
        pathTimer.resetTimer();
    }



    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        pathState = PathStateB.START;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        intake = new Intake(hardwareMap);
        spindex = new Spindex (hardwareMap);
        turret = new Turret(hardwareMap, true,follower, true);
        turret.resetTurret();
        turret.switchHoodAdjust();
        turret.disableFlywheeladjust();
        turret.disableHoodAdjust();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);



        buildPaths();


    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);

    }
    @Override
    public void loop(){
        follower.update();
        turret.updateFollower(follower);
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = Math.toDegrees(follower.getPose().getHeading());
        turret.setFLyWheelSpeedPID(targetFlywheelSpeed);
        turret.setHoodAngle(targetHoodAngle);

        telemetry.addData("X:", x);
        telemetry.addData("Y:", y);
        telemetry.addData("H:", h);

        turret.runTurret();


        statePathUpdate();
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Turret Aim", turret.getTargetAngle (follower.getPose().getX(),follower.getPose().getY()));
        telemetry.addData("pathState", pathState.toString());
        telemetry.addData ("turret Target", turret.getTargetAngle(x,y));

        Location.START = follower.getPose();
        Location.turretPosition=turret.getTurretPosition();
    }
}



