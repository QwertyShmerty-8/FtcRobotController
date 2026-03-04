package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.aDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Config

@TeleOp

public class configTester extends OpMode {

    private Intake intake;
    private Spindex spindex;
    SparkFunOTOS otos;
    private Turret turret;
    private Limelight3A limelight;
    double id;

    private aDrivetrain drive;

    public void init(){
        intake = new Intake(hardwareMap);
        spindex = new Spindex (hardwareMap);
        turret = new Turret(hardwareMap, "blue",0,false);
        drive = new aDrivetrain(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);
    }
    public void loop(){
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId(); // The ID number of the fiducial

        }
        telemetry.addData("Fiducial ", id);

        telemetry.addData("spindex position", spindex.getPosition());

        if (gamepad1.a){
            intake.intakeBalls();
        } else if (gamepad1.b) {
            intake.shootBalls();
        } else {
            intake.switchBallOrder();
        }
        if (gamepad1.b){
            drive.setBackRight(0.25);
        } else {
            drive.setBackRight(0);
        }
        if (gamepad1.x){
            drive.setFrontRight(0.25);
        } else {
            drive.setFrontRight(0);
        }
        if (gamepad1.y){
            drive.setFrontLeft(0.25);
        } else {
            drive.setFrontLeft(0);
        }
        if (gamepad1.left_trigger>0.25){
            spindex.setSpindexPower(0.5);
        }else{
            spindex.setSpindexPower(0);
        }
        if (gamepad1.right_trigger>0.25){
            turret.setFlyWheelSpeed(-900);
        }
        else{
            turret.setFlyWheelSpeed(0);
        }
        if (gamepad1.left_bumper){
            intake.setIntakePower(0.4);
        }
        else{
            intake.setIntakePower(0);
        }

        if (id == 21) {
            spindex.goToPosition(136.35);
        }
        if (id == 22) {
            spindex.goToPosition(351);
        }
        if (id == 23) {
            spindex.goToPosition(234);
        }

        telemetry.addData("spindex position", spindex.getPosition());


    }

}
