package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team.states.ITDHangStateMachine;

public class ITDHangSubsystem implements ISubsystem<ITDHangStateMachine, ITDHangStateMachine.State> {
    private static ITDHangStateMachine itdHangStateMachine;
    private RevMotor HangWheels;

    public ITDHangSubsystem(RevMotor HangMotor){
        setHangStateMachine(new ITDHangStateMachine());
        setHangWheels(HangMotor);
    }

    @Override
    public ITDHangStateMachine getStateMachine() {
        return itdHangStateMachine;
    }

    @Override
    public ITDHangStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getHangWheels().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getHangWheels().setPower(getState().getPower());
    }

    @Override
    public String getName() {
        return "Hang Subsystem";
    }

    private static void setHangStateMachine(ITDHangStateMachine HangSM) {
        ITDHangSubsystem.itdHangStateMachine = HangSM;
    }

    private void setHangWheels(RevMotor HangMotor){
        this.HangWheels = HangMotor;
    }
    private RevMotor getHangWheels(){
        return HangWheels;
    }
}