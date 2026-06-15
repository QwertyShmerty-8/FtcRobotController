package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Helperfunctions.Location;

public class robot {
    public aDrivetrain drivetrain;
    public Turret turret;
    public Intake intake;
    public Spindex spindex;

    public robot(HardwareMap hardwareMap, Follower follower){
        drivetrain = new aDrivetrain(hardwareMap);
        turret = new Turret(hardwareMap, Location.isRed,follower, true);
        intake = new Intake (hardwareMap);
        spindex = new Spindex (hardwareMap);
    }

    public void runRobot(){

    }
}
