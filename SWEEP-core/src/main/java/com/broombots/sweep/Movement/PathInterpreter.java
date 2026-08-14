package com.broombots.sweep.Movement;

import com.broombots.sweep.Builder.MovementPoint;
import com.broombots.sweep.Builder.Path;
import com.broombots.sweep.Classes.Coordinate;
import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.Timer;

public class PathInterpreter {
    private Path currentPath;
    private Timer timer;
    public void startPath(Path path){
        currentPath = path;
        timer = new Timer();
        timer.reset();
    }
    public void update(LocalizationPacket localizationPacket){
        currentPath.updateActions(localizationPacket);
    }
    public String updateSimulation(LocalizationPacket localizationPacket){
        return currentPath.updateActionsInSimulation(localizationPacket);
    }
    public Coordinate getRobotPosition(){
        return getMovementPoint().getPosition();
    }
    public MovementPoint getMovementPoint(){
        return currentPath.getMovement(timer.getSeconds());
    }
}
