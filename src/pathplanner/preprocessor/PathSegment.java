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
    public double goalVel = Double.NaN; //FIXME determine based on expansion around corner event!!
    public final List<Node> nodes;
    
    public PathSegment(Node start, Node end, Set<Obstacle2DB> obstacles, List<Node> nodes){
        this.obstacles = obstacles;
        this.start = start;
        this.end = end;
        this.nodes = nodes;
    }
    
    public PathSegment(Node start, Node end, List<Node> nodes){
        this(start, end, new HashSet<Obstacle2DB>(), nodes);
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
    
    public Pos2D getFinishVector(){
        int index = nodes.indexOf(end);
        if(index < 0){ throw new IllegalArgumentException();}
        Node prev = end.parent;
        Node next;
        if(nodes.size() <= index + 1){
            next = end;
        }else{
            next = nodes.get(index + 1);
        }
        
        Pos2D result = next.pos.minus(prev.pos).normalize();
        return result;
    }

}
