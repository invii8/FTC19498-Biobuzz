package org.firstinspires.ftc.teamcode.OpMode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "🔴 2 TIP", group = "00", preselectTeleOp = "Tele Of All Time")
public class Red2Tip extends TwoTipAuton {
    @Override protected boolean isBlue()     { return false; }
    @Override protected String  autonName()  { return "RED 2 TIP"; }
}
