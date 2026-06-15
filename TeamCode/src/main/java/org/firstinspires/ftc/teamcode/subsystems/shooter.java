package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class shooter {
    Spindex spindex;
    Turret turret;
    public shooter (HardwareMap hardwareMap, String goalColor, Follower follower){
        spindex = new Spindex (hardwareMap);
       // turret = new Turret(hardwareMap, goalColor,follower, true);
    }

    public void shoot(){

    }
}
