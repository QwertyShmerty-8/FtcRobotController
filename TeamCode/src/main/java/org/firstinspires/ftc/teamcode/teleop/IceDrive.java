package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.Helperfunctions.Fullfieldshootingvalues;
import org.firstinspires.ftc.teamcode.Helperfunctions.fancyButton;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.aDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.Timer;

@Config
@TeleOp(name=" Ice Drive")
public class IceDrive extends OpMode {

    private Fullfieldshootingvalues shootingvalues;

    private Intake intake;
    private Spindex spindex;
    SparkFunOTOS otos;
    private Turret turret;
    private fancyButton hoodAdjustOnToggle;
    private fancyButton slowModeToggle;
    private fancyButton turretFreezeToggle;
    private fancyButton sortModeToggle;
    private fancyButton holdModeToggle;
    private Boolean inHoldMode;
    double holdx,holdy,holdh;
    private ElapsedTime spindexTimer;
    private ElapsedTime time;




    private aDrivetrain drive;

    private Follower follower;
    public static Pose startingPose =new Pose (89.08, 136.15, Math.toRadians(0));//56,8,90
    public static Pose resetPose = new Pose(20.34,123.37,Math.toRadians(144));

    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;



    @Override
    public void init(){
        spindexTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        intake = new Intake(hardwareMap);
        spindex = new Spindex (hardwareMap);
        turret = new Turret(hardwareMap, "red",follower,true);
        drive = new aDrivetrain(hardwareMap);
        time = new ElapsedTime();



        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shootingvalues = new Fullfieldshootingvalues("red");
        hoodAdjustOnToggle = new fancyButton( fancyButton.PressType.Toggle);
        slowModeToggle = new fancyButton(fancyButton.PressType.Toggle);
        turretFreezeToggle = new fancyButton (fancyButton.PressType.Toggle);
        sortModeToggle = new fancyButton (fancyButton.PressType.Toggle);
        holdModeToggle = new fancyButton (fancyButton.PressType.Toggle);
        inHoldMode = false;





    }
    public void start(){


        follower.startTeleopDrive();
        intake.setIntakePower(1);
        follower.update();
    }
    public void loop(){
        if (gamepad2.a){
            intake.setIntakePower(-1);
        }else if(gamepad1.right_trigger>0.25){
            intake.setIntakePower(1);
        }else if(gamepad1.left_bumper==true){
            intake.setIntakePower(-1);
        }else{
            intake.setIntakePower(1);
        }
        hoodAdjustOnToggle.checkStatus(gamepad1.y);

        if (hoodAdjustOnToggle.startPress){
            turret.switchHoodAdjust();
        }
        telemetry.addData ("Hood Adjust", turret.getHoodAdjustOn());


        turretFreezeToggle.checkStatus(gamepad2.right_stick_button);
        if (turretFreezeToggle.startPress){
            turret.switchTurretState();
        }
        telemetry.addData ("TurretOn? ", turret.getTurretOn());

        slowModeToggle.checkStatus(gamepad2.touchpad);
        if (slowModeToggle.startPress){
            drive.switchSlowmode();
        }
        telemetry.addData ("Slowmode? ", drive.getSlowmode());

        sortModeToggle.checkStatus(gamepad2.x);

        if (sortModeToggle.startPress){
            intake.switchSortMode();
        }
        telemetry.addData ("SortMode?", intake.getSortMode());

        holdModeToggle.checkStatus(gamepad2.left_bumper||gamepad2.right_bumper);

        if (holdModeToggle.startPress&&inHoldMode==false){
            follower.holdPoint(follower.getPose());
            inHoldMode=true;
            drive.switchHoldMode();
        } else if(holdModeToggle.startPress&&inHoldMode==true ){
            follower.breakFollowing();
            inHoldMode = false;
            drive.switchHoldMode();
        }
        telemetry.addData ("Holding Position?", drive.getHoldMode());



        follower.update();

        turret.runTurret();
        intake.configureSortMode(gamepad1.a,time);





        drive.driveCA(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger, gamepad1.right_trigger);
        if(gamepad1.left_trigger>0.25&& intake.getSortMode()==false){
            intake.shootBalls();
        } else if (gamepad1.right_trigger>0.25&& intake.getSortMode()==false){
            intake.intakeBalls();
        } else if (gamepad1.right_bumper&& intake.getSortMode()==false){
            intake.shootBalls();

        } else if (gamepad2.a){
            intake.setIntakePower(-1);
        } else if(gamepad1.right_trigger>0.25){
            intake.setIntakePower(1);
        }

        //Gamepd2 controls
        if (gamepad2.left_trigger>0.25){
            spindex.runSpindexToggle(1);
            spindexTimer.reset();
        } else if (gamepad2.right_trigger>0.25){
            spindexTimer.reset();
            spindex.runSpindexToggle(-1);
            //  } else if (spindexTimer.seconds()>1) {
            //      spindex.goToPosition(0);
        }else if (spindexTimer.seconds()>1){
            spindex.goToPosition(344);
        }else {
            spindex.setSpindexPower(0);
        }

        if (gamepad2.left_stick_button){
            follower.setPose(resetPose);
        }
        telemetry.addData("Spindex Position", spindex.getPosition());
        Pose pose = turret.getCurrentPose();
        telemetry.addData("X position", pose.getX());
        telemetry.addData("Y position", pose.getY());
        telemetry.addData("H position", pose.getHeading());
        telemetry.addData ("Velocity", turret.getFlywheelVelocity());
        telemetry.addData ("robot heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Target Velocity", turret.getFlywheelTarget());
        telemetry.addData("Turret Position", turret.turretFieldPosition());
        telemetry.addData("error", turret.error(follower.getPose().getX(),follower.getPose().getY()));
        double targetAngle = turret.getTargetAngle(follower.getPose().getX(),follower.getPose().getY());
        telemetry.addData ("target", targetAngle);
        double robotHeading = Math.toDegrees(follower.getHeading());
        double turretNeeded = (targetAngle - robotHeading + 540) % 360 - 180;
        telemetry.addData("Turret deviation Nedded", turretNeeded);







    }
}


