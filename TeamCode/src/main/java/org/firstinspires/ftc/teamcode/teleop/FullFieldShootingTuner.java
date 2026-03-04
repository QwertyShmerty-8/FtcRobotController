package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.aDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp
@Config

public class FullFieldShootingTuner extends OpMode{
    


    private Intake intake;
    private Spindex spindex;
    private Turret turret;

    private aDrivetrain drive;

    double flywheelVelocity;
    double hoodAngle;
    double distance;

    private Follower follower;
    public static Pose startingPose = new Pose(72,72,0);

    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;



    @Override
    public void init(){


        intake = new Intake(hardwareMap);
        spindex = new Spindex (hardwareMap);
        turret = new Turret(hardwareMap, "blue",0,false);
        drive = new aDrivetrain(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        turret.disableTurret();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());






    }

        public void start() {
            follower.startTeleopDrive();
            follower.update();
        }


    public void loop() {
        if (gamepad1.dpad_left){
            intake.setIntakePower(1);
        } else if(gamepad1.dpad_right){
            intake.setIntakePower(-1);
        }


        follower.update();

        double distanceBlue = Math.sqrt(follower.getPose().getX()*follower.getPose().getX() + (144-follower.getPose().getY())*(144-follower.getPose().getY()));


        drive.driveIG(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger, gamepad1.right_trigger);

        turret.aimTurret(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));

        if (gamepad1.b) {
            intake.intakeBalls();
        }
        if (gamepad1.a) {
            intake.shootBalls();
        }

        if (gamepad1.x) {
            spindex.setSpindexPower(1);
        } else {
            spindex.setSpindexPower(0);
        }





        if (gamepad1.left_bumper) {
            flywheelVelocity += 10;
        }
        if (gamepad1.right_bumper) {
            flywheelVelocity -= 10;
        }

        if (gamepad1.dpad_up) {
            hoodAngle += 0.01;
        }
        if (gamepad1.dpad_down) {
            hoodAngle -= 0.01;
        }



        turret.setHoodAngle(hoodAngle);
        turret.setFlyWheelSpeed(flywheelVelocity);


        telemetry.addData("Distance", distance);


        telemetry.addData("Hood Angle", hoodAngle);
        telemetry.addData("flywheelVelocity", turret.getFlyWheelSpeed());
        telemetry.addData ("X", follower.getPose().getX());
        telemetry.addData ( "Y", follower.getPose().getY());
        telemetry.addData("H", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData ("Turret Angle", turret.getTurretDeviationOffset());

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());

        turret.updateFlywheelCoefficents();

        telemetry.addData("Aim angle Blue", Math.toDegrees((Math.PI / 2) + Math.atan(follower.getPose().getX()/ (144 - follower.getPose().getY()))));
        telemetry.update();


    }

    }



