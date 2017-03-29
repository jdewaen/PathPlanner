package pathplanner.preprocessor;

import java.util.LinkedList;

import pathplanner.common.Pos2D;


public class HPANode extends Node {
    
    public final WorldSegment target;
    public final LinkedList<Node> expandedPath;
    public HPANode(HPANode parent, Pos2D pos, WorldSegment target, LinkedList<Node> expandedPath) {
        super(parent, pos, calculateDistance(parent, expandedPath), calculateDistance(parent, expandedPath));
        this.target = target;
        this.expandedPath = expandedPath;
        
    }
    private static double calculateDistance(HPANode parent, LinkedList<Node> expandedPath){
        double distance = 0;
        if(parent != null) distance += parent.distance;
        if(!expandedPath.isEmpty()) distance += expandedPath.getLast().distance;
        return distance;
    }

    public LinkedList<Node> getPath(){
        if(parent == null) return expandedPath;
        LinkedList<Node> path = new LinkedList<Node>(parent.getPath());
        path.addAll(expandedPath);
        return path; 
    }
    
}
