package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.teamcode.teleop.spindexMotifTest.ShootModes.CHANGE_SPINDEX_POSITION;
import static org.firstinspires.ftc.teamcode.teleop.spindexMotifTest.ShootModes.SHOOT;
import static org.firstinspires.ftc.teamcode.teleop.spindexMotifTest.ShootModes.WAIT_FOR_SHOOT_BUTTON;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Autos.sampleAutoPathing;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Config
@TeleOp

public class spindexMotifTest extends OpMode {

    private Limelight3A limelight;
    private Intake intake;
    int id;
    private Spindex spindex;
    private Turret turret;
    AnalogInput encoder;
    private Timer pathTimer;
    private double lastLootTime=0;
    private double currentLoopTime;
    double targetSpindexPosition = 0;

    public enum ShootModes {
        //STARTPosition -->EndPosition
        CHANGE_SPINDEX_POSITION,
        SHOOT,
        WAIT_FOR_SHOOT_BUTTON;;
    }

    ShootModes shootModes;


    @Override
    public void init() {

        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap, "blue",0,true);
        encoder = hardwareMap.get(AnalogInput.class, "spindexencoder");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0);
        shootModes = ShootModes.WAIT_FOR_SHOOT_BUTTON;
        pathTimer = new Timer();


    }

    public void loop() {

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId(); // The ID number of the fiducial

        }
        telemetry.addData("Fiducial ", id);

        telemetry.addData("spindex position", spindex.getPosition());
        telemetry.addData("spindex inside class position", encoder.getVoltage() / 3.2 * 360);

        currentLoopTime = System.currentTimeMillis();
        telemetry.addData("loop time", currentLoopTime -lastLootTime);
        lastLootTime = currentLoopTime;

        shootSwitchCase();
    }

    public void shootSwitchCase() {
        switch (shootModes) {
            case WAIT_FOR_SHOOT_BUTTON:
                spindex.goToPosition(47.5);
                intake.intakeBalls();
                intake.forwardIntakeDirection();
                if (id == 21) {
                    targetSpindexPosition =278.75;
                }
                if (id == 22) {
                    targetSpindexPosition=156.25;
                }
                if (id == 23) {
                    targetSpindexPosition =47.5;
                }
                if (gamepad1.a) {
                    intake.switchBallOrder();
                    intake.setIntakePower(0.25);

                    pathTimer.resetTimer();

                    shootModes = CHANGE_SPINDEX_POSITION;
                }

                break;
            case CHANGE_SPINDEX_POSITION:




                //We are Assuming Balls are in GPP order


                spindex.goToPosition(targetSpindexPosition);

                if (Math.abs(spindex.getPosition()-targetSpindexPosition) < 5) {
                    intake.setIntakePower(1);
                    intake.shootBalls();
                    pathTimer.resetTimer();
                    shootModes = SHOOT;


                }
                break;
            case SHOOT:

                intake.setIntakePower(1);
                turret.setFlyWheelSpeed(-900);

                spindex.setSpindexPower(0.5);

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    shootModes = WAIT_FOR_SHOOT_BUTTON;
                }
                break;


        }

    }
}
