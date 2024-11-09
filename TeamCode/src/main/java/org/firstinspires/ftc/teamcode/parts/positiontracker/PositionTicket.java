package org.firstinspires.ftc.teamcode.parts.positiontracker;

import om.self.ezftc.utils.Vector2;
import om.self.ezftc.utils.Vector3;

public class PositionTicket {
    public final Vector3 position;
    public final Vector3 confidence;
    public final Vector3 accuracy;
    public final Vector2 robotRelative;

    public PositionTicket(Vector3 position, Vector3 confidence, Vector3 accuracy, Vector2 robotRelative) {
        this.position = position;
        this.confidence = confidence;
        this.accuracy = accuracy;
        this.robotRelative = robotRelative;
    }

    public PositionTicket(Vector3 position, Vector3 confidence, Vector3 accuracy) {
        this.position = position;
        this.confidence = confidence;
        this.accuracy = accuracy;
        robotRelative = new Vector2();
    }

    public PositionTicket(Vector3 position, Vector2 robotRelative) {
        this.position = position;
        this.robotRelative = robotRelative;
        this.confidence = new Vector3(.5);
        this.accuracy = new Vector3(.5);
    }

    public PositionTicket(Vector3 position){
        this.position = position;
        this.confidence = new Vector3(.5);
        this.accuracy = new Vector3(.5);
        robotRelative = new Vector2();
    }
}
