package org.firstinspires.ftc.teamcode.OpMode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "🔵 2 TIP", group = "00", preselectTeleOp = "Tele Of All Time")
public class Blue2Tip extends TwoTipAuton {
    @Override protected boolean isBlue()     { return true; }
    @Override protected String  autonName()  { return "BLUE 2 TIP"; }
}
