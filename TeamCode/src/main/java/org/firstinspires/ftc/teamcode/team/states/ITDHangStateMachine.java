package org.firstinspires.ftc.teamcode.team.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;

public class ITDHangStateMachine extends SimpleState<ITDHangStateMachine.State> {
    public ITDHangStateMachine() {
        super(State.IDLE);
    }

    @Override
    public String getName() {
        return "ITDHang State Machine";
    }

    public enum State implements Namable {
        IDLE("Idle", 0d),
        UP("Clockwise", -1d),
        DOWN("CounterClockwise", 1d);

        private final String name;
        private final double power;

        State(final String name, final double power) {
            this.name  = name;
            this.power = power;
        }

        @Override
        public String getName() {
            return name;
        }

        public double getPower() {
            return power;
        }
    }
}