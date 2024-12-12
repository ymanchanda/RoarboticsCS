package org.firstinspires.ftc.teamcode.team;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.team.auto.ITDBaseLACH;

import java.util.Arrays;

/**
 * Motor naming convention:
 *     Drivetrain
 *         Front Left Wheel     -> LF
 *         Back Left Wheel      -> LR
 *         Front Right Wheel    -> RF
 *         Back Right Wheel     -> RR
 *     Arm
 *          Arm motor            -> Arm
 *     Lift
 *          Lift motor          -> Lift
 *     Hang
 *          Hang motor          -> Hang
 *
 * Servo naming convention:
 *     Claw
 *         Claw   servo         -> Claw
 *     ClawArm
 *         ClawArm servo        -> ClawArm
 *     Lock
 *          Lock servo          -> Lock
 *
 */

public abstract class ITDTeleopRobotCHALLC extends Robot {
    private TimeProfiler matchRuntime;
    protected ITDBaseLACH drive;

    @Override
    public void init() {
        super.init();
        setMatchRuntime(new TimeProfiler(false));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        drive.getExpansionHubs().start();
        drive.robot.getDrive().start();
        Arrays.stream(getMotors()).forEach(RevMotor::resetEncoder);
        getMatchRuntime().start();
    }

    @Override
    public void loop() {
        super.loop();
        drive.getExpansionHubs().update(getDt());
        drive.update();
        drive.robot.getITDArmSubsystem().update(getDt());
        drive.robot.getITDLiftSubsystem().update(getDt());
        drive.robot.getITDClawSubsystem().update(getDt());
        drive.robot.getITDClawArmSubsystem().update(getDt());
    }

    @Override
    public void stop() {
        super.stop();
        drive.getExpansionHubs().stop();
        drive.robot.getITDArmSubsystem().stop();
        drive.robot.getITDLiftSubsystem().stop();
        drive.robot.getITDClawSubsystem().stop();
        drive.robot.getITDClawArmSubsystem().stop();
    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

}
