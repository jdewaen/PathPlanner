package pathplanner.preprocessor;

import java.util.LinkedList;

import pathplanner.common.Pos2D;


public class HPANode extends Node {
    
    public final WorldSegment target;
    public final LinkedList<Node> expandedPath;
    public HPANode(HPANode parent, Pos2D pos, WorldSegment target, LinkedList<Node> expandedPath) {
        super(parent, pos, calculateDistance(expandedPath), calculateDistance(expandedPath));
        this.target = target;
        this.expandedPath = expandedPath;
        
    }
    private static double calculateDistance(LinkedList<Node> expandedPath){
        double distance = 0;
        if(!expandedPath.isEmpty()) distance += expandedPath.getLast().distance;
        return distance;
    }

    public LinkedList<Node> getPath(){
        if(parent == null) return expandedPath;
        LinkedList<Node> path = new LinkedList<Node>(parent.getPath());
        while(!path.isEmpty() && path.getLast().pos.equals(expandedPath.getFirst().pos)){
            path.removeLast();
        }
        if(path.isEmpty()) return expandedPath;
        Node prevLast = path.getLast();
        Node curFirst = expandedPath.getFirst();
        prevLast.setChild(curFirst);
        curFirst.setParent(prevLast);
        path.addAll(expandedPath);
        return path; 
    }
    
}
