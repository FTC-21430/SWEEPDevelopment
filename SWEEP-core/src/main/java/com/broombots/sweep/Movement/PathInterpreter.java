package com.broombots.sweep.Movement;

import com.broombots.sweep.Builder.MovementPoint;
import com.broombots.sweep.Builder.Sequence;
import com.broombots.sweep.Classes.LocalizationPacket;
import com.broombots.sweep.Classes.PathPoint;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.Timer;

public class PathInterpreter {
    private Sequence currentPath;
    private Timer timer;
    public void startPath(Sequence sequence){
        currentPath = sequence;
        timer = new Timer();
        timer.reset();
    }
    public void update(LocalizationPacket localizationPacket){
        currentPath.updateActions(localizationPacket);
    }
    public String updateSimulation(LocalizationPacket localizationPacket){
        return currentPath.updateActionsInSimulation(localizationPacket);
    }
    public Pos2D getRobotPosition(){
        return getPathPoint().position;
    }
    public PathPoint getPathPoint(){
        return currentPath.getMovement(timer.getSeconds());
    }
}
