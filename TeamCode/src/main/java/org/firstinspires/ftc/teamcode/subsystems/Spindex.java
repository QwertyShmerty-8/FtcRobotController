package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.myConstants.Spindex.KdSpindex;
import static org.firstinspires.ftc.teamcode.myConstants.Spindex.KfSpindex;
import static org.firstinspires.ftc.teamcode.myConstants.Spindex.KiSpindex;
import static org.firstinspires.ftc.teamcode.myConstants.Spindex.KpSpindex;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.KdTurret;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.KiTurret;
import static org.firstinspires.ftc.teamcode.myConstants.Turret.KpTurret;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Helperfunctions.Fullfieldshootingvalues;
import org.firstinspires.ftc.teamcode.Helperfunctions.myPIDF;

public class Spindex {
    DcMotor spindex;
    myPIDF pidf;
    private double lastError = 0;
    private double integralSum = 0;
    private final double dt = 0.02;

    Fullfieldshootingvalues values;

    boolean atPosition = false;


    public Spindex(HardwareMap hardwareMap) {
        spindex = hardwareMap.get(DcMotor.class, "spindexer");
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidf = new myPIDF(KpSpindex,KiSpindex,KdSpindex,KfSpindex,-1,1,3);

    }

    public void setSpindexPower(double power){
        spindex.setPower(power);
    }

    public double getPosition(){
        double raw = (spindex.getCurrentPosition() * 1.25);
        raw = ((raw % 360) + 360) % 360;

        double adjusted = raw;
        return (adjusted + 360) % 360;
    }

    public void goToPosition(double target) {
        double raw = (spindex.getCurrentPosition() * 1.25);
        raw = ((raw % 360) + 360) % 360;

        double adjusted = raw ;
        double position = (adjusted + 360) % 360;
        double errorForward = (target - position + 360) % 360;
        double errorShortest = ((target - position + 540) % 360) - 180;

        double error;

        if (Math.abs(errorShortest) <= 10) {
            error = errorShortest;   // allow small backward correction
        } else {
            error = errorForward;    // otherwise always go forward
        }

        if (Math.abs(error) < 1) {
            lastError = 0;
            integralSum = 0;
            spindex.setPower(0);

        } else {


// PID
            integralSum += error * dt;
            integralSum = Math.max(-50, Math.min(50, integralSum));

            double derivative = (error - lastError) / dt;
            double ff = KfSpindex * Math.signum(error);
            double output = KpSpindex * error + KiSpindex * integralSum + KdSpindex * derivative+ff;


// Cap speed
            output = Math.max(-1, Math.min(1, output));
            spindex.setPower(output);

            lastError = error;
        }

    }
    public void spindexIntake(){
        goToPosition(47.5);
    }


    }

