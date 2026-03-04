package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Autos.BlueCloseAuto.PathState.STARTINTAKEGPP_ENDINTAKEGPP;
import static org.firstinspires.ftc.teamcode.Autos.simpleGoalBlueAuto.PathState.SHOOTPOSETOENDPOSE;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.aDrivetrain;

import java.util.List;


@Autonomous

public class BlueCloseAuto extends OpMode {

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

    public enum PathState{
        //STARTPosition -->EndPosition

        DETECTAPRILTAG,

        START_SHOOTPRELOAD,
        SORTPRELOAD,
        SHOOTPRELOAD_STARTINTAKEGPP,
        STARTINTAKEGPP_ENDINTAKEGPP,
        ENDINTAKEGPP_SHOOTGPP,
        SHOOTGPP_STARTINTAKEPGP,
        STARTINTAKEPGP_ENDINTAKEPGP,
        ENDINTAKEPGP_SHOOTPGP,
        SHOOTPGP_STARTINTAKEPPG,
        STARTINTAKEPPG_ENDINTAKEPPG,
        ENDINTAKEPPG_SHOOTPPG,
        DONE,
    }

    PathState pathState;
    private final Pose startPose = new Pose(20.000, 123.000, Math.toRadians (143));
    private final Pose shootPreloadPose = new Pose (35.49,107.99,Math.toRadians(143));
    private final Pose startIntakeGPPPose = new Pose (50.35,84.07,Math.toRadians(180));
    private final Pose endIntakeGPPPose = new Pose (19.05,84.58,Math.toRadians(180));
    private final Pose shootGPPPose = new Pose (60.47,83.97,Math.toRadians(180));
    private final Pose shootPPGstartIntakePGPcontrolPoint = new Pose (64.71,64.71,Math.toRadians(180));
    private final Pose startIntakePGPPose = new Pose (42.921,59.73,Math.toRadians(180));
    private final Pose endIntakePGPPose = new Pose (22.158,59.81,Math.toRadians(180));
    private final Pose shootPGPPose = new Pose (60.92,84.04,Math.toRadians(190));
    private final Pose startIntakePPG = new Pose (43.122,35.70);
    private final Pose endIntakePPG = new Pose (21.81,35.56, Math.toRadians(180));
    private final Pose shootPPG = new Pose (62.42,107.33, Math.toRadians(180));

    private PathChain startPos_ShootPreloadPos, shootPreload_startIntakeGPP, startIntakeGPP_endIntakeGPP;
    private PathChain endIntakeGPP_shootGPP, shootGPP_startIntakePGP, startIntakePGP_endIntakePGP;
    private PathChain endIntakePGP_shootPGP, shootPGP_startIntakePPG,startIntakePPG_endIntakePPG;
    private PathChain endIntakePPG_shootPPG;

    public void buildPaths(){
        //put in coordinates for start pose and end pose
        startPos_ShootPreloadPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPreloadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPreloadPose.getHeading()).build();
        shootPreload_startIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine (shootPreloadPose, startIntakeGPPPose))
                .setLinearHeadingInterpolation(shootPreloadPose.getHeading(), startIntakeGPPPose.getHeading())
                .build();
        startIntakeGPP_endIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine (startIntakeGPPPose, endIntakeGPPPose))
                .setLinearHeadingInterpolation(startIntakeGPPPose.getHeading(), endIntakeGPPPose.getHeading())
                .build();
        endIntakeGPP_shootGPP = follower.pathBuilder()
                .addPath(new BezierLine (endIntakeGPPPose, shootGPPPose))
                .setLinearHeadingInterpolation(endIntakeGPPPose.getHeading(), shootGPPPose.getHeading())
                .build();
        shootGPP_startIntakePGP = follower.pathBuilder()
                .addPath(new BezierCurve(shootGPPPose, shootPPGstartIntakePGPcontrolPoint,startIntakePGPPose))
                .setLinearHeadingInterpolation(shootGPPPose.getHeading(), startIntakePGPPose.getHeading())
                .build();
        startIntakePGP_endIntakePGP = follower.pathBuilder()
                .addPath(new BezierLine (startIntakePGPPose, endIntakePGPPose))
                .setLinearHeadingInterpolation(startIntakePGPPose.getHeading(), endIntakePGPPose.getHeading())
                .build();
        endIntakePGP_shootPGP = follower.pathBuilder()
                .addPath(new BezierLine (endIntakePGPPose, shootPGPPose))
                .setLinearHeadingInterpolation(endIntakePGPPose.getHeading(), shootPGPPose.getHeading())
                .build();
        shootPGP_startIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine (shootPGPPose, startIntakePPG))
                .setLinearHeadingInterpolation(shootPGPPose.getHeading(), startIntakePPG.getHeading())
                .build();
        startIntakePPG_endIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine (startIntakePPG, endIntakePPG))
                .setLinearHeadingInterpolation(startIntakePPG.getHeading(), endIntakePPG.getHeading())
                .build();
        endIntakePPG_shootPPG = follower.pathBuilder()
                .addPath(new BezierLine (endIntakePPG, shootPPG))
                .setLinearHeadingInterpolation(endIntakePPG.getHeading(), shootPPG.getHeading())
                .build();


    }
    public void statePathUpdate(){
        switch(pathState){

            case DETECTAPRILTAG:
                LLResult result = limelight.getLatestResult();
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                intake.setIntakePower(-0.25);

                double maxPosition=-10000;
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial
                    double x = fiducial.getTargetXDegrees(); // Where it is (left-right)

                    if (x>maxPosition){
                        maxPosition = x;
                        idb =id;
                    }
                }
                telemetry.addData("Fiducial", idb);
                if (idb ==21){
                    opModeTimer.resetTimer();
                    ppgGoToPosition =0;
                    pgpGoToPosition =100;
                    gppGoToPosition= 200;
                    turret.switchTurretState();
                    spindex.goToPosition(gppGoToPosition);
                    follower.followPath(startPos_ShootPreloadPos);
                    setPathState(PathState.START_SHOOTPRELOAD);

                }
                if (idb ==22){
                    turret.switchTurretState();
                    opModeTimer.resetTimer();
                    ppgGoToPosition =200;
                    pgpGoToPosition =0;
                    gppGoToPosition= 100;
                    spindex.goToPosition(gppGoToPosition);
                    follower.followPath(startPos_ShootPreloadPos);
                    setPathState(PathState.START_SHOOTPRELOAD);

                }
                if (idb ==23){
                    turret.switchTurretState();
                    opModeTimer.resetTimer();
                    ppgGoToPosition =100;
                    pgpGoToPosition =200;
                    gppGoToPosition= 0;
                    spindex.goToPosition(gppGoToPosition);
                    follower.followPath(startPos_ShootPreloadPos);
                    setPathState(PathState.START_SHOOTPRELOAD);

                }
                break;




            case  START_SHOOTPRELOAD:

                intake.shootBalls();
                spindex.setSpindexPower(1);
                intake.setIntakePower(1);

                if (!follower.isBusy()) {

                   if(opModeTimer.getElapsedTimeSeconds()>5) {
                       opModeTimer.resetTimer();
                       follower.followPath(shootPreload_startIntakeGPP);
                       setPathState(PathState.SHOOTPRELOAD_STARTINTAKEGPP);
                   }
                }

                break;
            case SHOOTPRELOAD_STARTINTAKEGPP:
                spindex.goToPosition(0);
                intake.intakeBalls();

                if (!follower.isBusy()){
                    opModeTimer.resetTimer();
                    follower.followPath(startIntakeGPP_endIntakeGPP);
                    intake.setIntakePower(-0.25);
                    setPathState(STARTINTAKEGPP_ENDINTAKEGPP);

                }
                break;
            case STARTINTAKEGPP_ENDINTAKEGPP:
                spindex.goToPosition(0);
                intake.intakeBalls();
                if (!follower.isBusy()){
                    opModeTimer.resetTimer();
                    spindex.goToPosition(pgpGoToPosition);
                    follower.followPath(endIntakeGPP_shootGPP);
                    setPathState(PathState.ENDINTAKEGPP_SHOOTGPP);
                }
                break;
                
            case ENDINTAKEGPP_SHOOTGPP:
                spindex.goToPosition(gppGoToPosition);
                if (!follower.isBusy()){
                    intake.shootBalls();
                    spindex.setSpindexPower(1);
                    if (opModeTimer.getElapsedTimeSeconds()>3){
                        opModeTimer.resetTimer();
                        follower.followPath(shootGPP_startIntakePGP);
                        setPathState(PathState.SHOOTGPP_STARTINTAKEPGP);

                    }
                }
                break;

            case SHOOTGPP_STARTINTAKEPGP:
                spindex.goToPosition(0);
                intake.intakeBalls();
                if (!follower.isBusy()){
                    opModeTimer.resetTimer();
                    follower.followPath(startIntakePGP_endIntakePGP);
                    setPathState(PathState.STARTINTAKEPGP_ENDINTAKEPGP);
                }
                break;
            case STARTINTAKEPGP_ENDINTAKEPGP:
                spindex.goToPosition(0);
                intake.intakeBalls();
                if (!follower.isBusy()){
                    opModeTimer.resetTimer();
                    spindex.goToPosition(pgpGoToPosition);
                    follower.followPath(endIntakePGP_shootPGP);
                    setPathState(PathState.ENDINTAKEPGP_SHOOTPGP);
                }
                break;

            case ENDINTAKEPGP_SHOOTPGP:
                spindex.goToPosition(pgpGoToPosition);
                if (!follower.isBusy()){
                    intake.shootBalls();
                    spindex.setSpindexPower(1);
                    if (opModeTimer.getElapsedTimeSeconds()>3){
                        opModeTimer.resetTimer();
                        follower.followPath(shootPGP_startIntakePPG);
                        setPathState(PathState.SHOOTPGP_STARTINTAKEPPG);

                    }
                }
                break;
            case SHOOTPGP_STARTINTAKEPPG:
                spindex.goToPosition(0);
                intake.intakeBalls();
                if (!follower.isBusy()){
                    opModeTimer.resetTimer();
                    follower.followPath(startIntakePPG_endIntakePPG);
                    setPathState(PathState.STARTINTAKEPPG_ENDINTAKEPPG);
                }
                break;
            case STARTINTAKEPPG_ENDINTAKEPPG:
                spindex.goToPosition(0);
                intake.intakeBalls();
                if (!follower.isBusy()){
                    intake.shootBalls();
                    opModeTimer.resetTimer();
                    spindex.goToPosition(ppgGoToPosition);
                    follower.followPath(endIntakePPG_shootPPG);
                    setPathState(PathState.ENDINTAKEPPG_SHOOTPPG);
                }
                break;
            case ENDINTAKEPPG_SHOOTPPG:
                spindex.goToPosition(ppgGoToPosition);
                if (!follower.isBusy()){
                    spindex.setSpindexPower(1);
                    if (opModeTimer.getElapsedTimeSeconds()>3){
                        opModeTimer.resetTimer();

                        setPathState(PathState.DONE);

                    }
                }
                break;

            case DONE:
                telemetry.addLine("Finished");
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
        pathState = PathState.DETECTAPRILTAG;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        intake = new Intake(hardwareMap);
        spindex = new Spindex (hardwareMap);
        turret = new Turret(hardwareMap, "blue",40,false);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);


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
        double h = Math.toDegrees(follower.getPose().getHeading());
        turret.setFlyWheelSpeed(-1200);

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



