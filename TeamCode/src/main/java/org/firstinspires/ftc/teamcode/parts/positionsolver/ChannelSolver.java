package org.firstinspires.ftc.teamcode.parts.positionsolver;

import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;

import om.self.ezftc.core.part.ControllablePart;

public abstract class ChannelSolver extends Solver<PositionSolver, DriveControl>{
    public ChannelSolver(PositionSolver positionSolver, String name) {
        super(positionSolver, name);
    }

    @Override
    public ControllablePart<?, ?, ?, DriveControl> getControlled() {
        return parent.parent;
    }
}
