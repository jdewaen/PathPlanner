package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Vehicle;


public class PathSegment {
    
    public final Set<Obstacle2DB> obstacles;
    public final Node start;
    public final Node end;
    public double goalVel = Double.NaN;
    
    public PathSegment(Node start, Node end, Set<Obstacle2DB> obstacles){
        this.obstacles = obstacles;
        this.start = start;
        this.end = end;
    }
    
    public PathSegment(Node start, Node end){
        this(start, end, new HashSet<Obstacle2DB>());
    }
    
    public static List<Pos2D> toPositions(List<PathSegment> segments){
        List<Pos2D> result = new ArrayList<Pos2D>();
        for(PathSegment segment : segments){
            result.add(segment.start.pos);
            result.add(segment.end.pos);
        }
        
//        result.add(segments.get(segments.size()-1).end.pos);
        
        return result;
    }
    
    public List<Pos2D> toIndividualPositions(){
        List<Pos2D> result = new ArrayList<Pos2D>();
        Node current = end;
        while(current.cost >= start.cost){
            result.add(current.pos);
            if(current.parent == null) break;
            current = current.parent;
        }
        return result;
    }
    
//    private Node advanceBeyondEnd(Node node, double distance){
//        Node current = node;
//        Node next = node.getChild();
//        while(next != null && next.cost - node.cost < distance){
//            current = next;
//            next = next.getChild();
//        }
//        return current;
//    }
//    
//    private Node advanceBeyondSteps(Node node, int steps){
//        Node current = node;
//        Node next = node.getChild();
//        int i = 0;
//        while(next != null && i < steps){
//            current = next;
//            next = next.getChild();
//            i++;
//        }
//        return current;
//    }
    
    public double estimateTimeNeeded(Vehicle vehicle, double minimum){
        double distance = end.cost - start.cost;
        return Math.max(3 * distance/vehicle.maxSpeed, minimum);
    }
    
    public double getDistance(){
        return end.cost - start.cost;
    }

}
