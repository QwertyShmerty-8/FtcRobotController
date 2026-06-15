package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.aDrivetrain;

@Disabled
@Autonomous

public class iceAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Intake intake;
    private Spindex spindex;
    private Turret turret;

    public enum icePathState {
        //STARTPosition -->EndPosition

        STARTPOSETOSHOOTPOSE,
        SHOOT,
        SHOOTPOSETOENDPOSE,
    }

    icePathState pathState;
    private final Pose startPose = new Pose(107.72, 136.15, Math.toRadians(0));
    private final Pose shootPose = new Pose(89.08, 136.15, Math.toRadians(0));
    private final Pose endPose = new Pose (89.08, 136.15, Math.toRadians(0));
    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    public void buildPaths() {
        //put in coordinates for start pose and end pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading()).build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case STARTPOSETOSHOOTPOSE:
                turret.setHoodAngle(0.3);
                if (opModeTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveStartPosShootPos);
                }
                if (!follower.isBusy()&&opModeTimer.getElapsedTimeSeconds()>24) {
                    setPathState(icePathState.SHOOT);
                }



                break;
            case SHOOT:
                intake.shootBalls();
                intake.setIntakePower(1);

                spindex.runSpindexToggle(1);
                if (opModeTimer.getElapsedTimeSeconds()>3.5) {
                    telemetry.addLine("Finished");
                }


                break;
            case SHOOTPOSETOENDPOSE:
                spindex.goToPosition(344);
                if (!follower.isBusy()) {
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

    public void setPathState(icePathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        pathState = icePathState.STARTPOSETOSHOOTPOSE;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        intake = new Intake(hardwareMap);
        spindex = new Spindex(hardwareMap);
        turret = new Turret(hardwareMap, false, follower, true);


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

    }

    @Override
    public void loop() {

        follower.update();
        turret.setFLyWheelSpeedPID(-1040);
        turret.updateFollower(follower);
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = follower.getPose().getHeading();

        telemetry.addData("X:", x);
        telemetry.addData("Y:", y);
        telemetry.addData("H:", h);


        statePathUpdate();
        telemetry.addData("pathState", pathState.toString());
        telemetry.addData("turret Target", turret.getTargetAngle(x, y));

    }
}

