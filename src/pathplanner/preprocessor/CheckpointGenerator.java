package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import javafx.util.Pair;
import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;


public class CheckpointGenerator {
    
    Scenario scenario;
    List<Obstacle2DB> obstacles;
    List<List<Pos2D>> vertices;
    
    public CheckpointGenerator(Scenario scenario){
        this.scenario = scenario;
        generateVertices();
    }
    
    private void generateVertices(){
        obstacles = new ArrayList<Obstacle2DB>();
        vertices = new ArrayList<List<Pos2D>>();
        List<Obstacle2DB> regions = scenario.world.getObstacles();
        for(int i = 0; i < regions.size(); i++){
            Obstacle2DB region = regions.get(i);
            obstacles.add(region);
            vertices.add(region.getVertices());
        }
    }
    
    public List<CornerEvent> generateCornerEvents(LinkedList<Node> path, double gridSize, double tolerance){        
//        Map<Node, Set<Obstacle2DB>> cornersNodes = getCornerNodes(path, gridSize);
        List<CornerEvent> corners = CornerEvent.generateEvents3(path, scenario.vehicle.getAccDist() * tolerance, path.getFirst());
        
        return corners;
    }
    
    public List<PathSegment> generateFromPath(LinkedList<Node> path, double gridSize, List<CornerEvent> corners, double margin, double maxTime){
        

           
        List<PathSegment> result = expandCornerEvents(corners, path, scenario.vehicle.getAccDist() * margin, scenario.vehicle.maxSpeed * maxTime);
        
        return result;

        
    };
    
    private Node expandForwards(Node start, double distance, List<Node> nodes){
        double goal = start.cost + distance;
        Node current = start;
        int index = nodes.indexOf(start);//TODO often not in nodes
        if(index < 0){ throw new IllegalArgumentException();}
        while(current.cost < goal && index < nodes.size()){
            index++;
            current = nodes.get(index);
        }
        if(current.cost < goal){
            return current;
        }else{
            Node parent = current.parent;
            Pos2D diff = current.pos.minus(parent.pos);
            diff = diff.normalize();
            diff = diff.multiply(goal - parent.cost);
            Pos2D newPos = parent.pos.plus(diff);
            Node inter = new Node(parent, newPos, goal, goal);
            parent.setChild(inter);
            current.setParent(inter);
            inter.setChild(current);
            nodes.add(index, inter);
            return inter;
        }
        
        
    }
    
    private Node expandBackwards(Node start, double distance, List<Node> nodes){
        double goal = start.cost - distance;
        Node current = start;
        Node last = null;
        
        int index = nodes.indexOf(current);//TODO often not in nodes
        if(index < 0){ throw new IllegalArgumentException();}
        
        while(current.cost > goal && current.parent != null){
            last = current;
            current = current.parent;
            
            index = nodes.indexOf(current);//TODO often not in nodes
            if(index < 0){ throw new IllegalArgumentException();}
//            
        }
        
        if(current.cost > goal){
            return current;
        }else{
            Node child = last;
            Pos2D diff = child.pos.minus(current.pos);
            diff = diff.normalize();
            diff = diff.multiply(goal - current.cost);
            Pos2D newPos = current.pos.plus(diff);
            Node inter = new Node(current, newPos, goal, goal);
            current.setChild(inter);
            child.setParent(inter);
            inter.setChild(child);
            nodes.add(index + 1, inter);
            return inter;
        }
        
    }
    
    private List<PathSegment> expandCornerEvents(List<CornerEvent> events, LinkedList<Node> path, double expansionDist, double maxLength){
        ArrayList<Node> nodes = new ArrayList<Node>(path);
        List<PathSegment> result = new ArrayList<PathSegment>();
        
        Node last = path.getFirst();
        
        if(events.isEmpty()){
        	result.addAll(segmentize(last, path.getLast(), null, maxLength, nodes));
        	return result;
        }
        
        // If there is plenty of room before the first corner:
        // Make a segment from the start to the expanded first corner
        if(events.get(0).start.cost > expansionDist){
            Node currentNode = events.get(0).start;
            
            // expand backwards
//            double goalCost = currentNode.cost - expansionDist;
            currentNode = expandBackwards(currentNode, expansionDist, nodes);
//            while(currentNode.cost > goalCost){
//                currentNode = currentNode.parent;
//            }
            
            
            if(currentNode.cost > last.cost){
                result.addAll(segmentize(last, currentNode, null, maxLength, nodes));
                last = currentNode;
            }
        }
        
        // For each but the last corner corner
        for(int i = 0; i < events.size() - 1; i++){
            Node current = events.get(i).end;
            Node next = events.get(i + 1).start;
            if(next.cost - current.cost > 3 * expansionDist){
                
                
                //expand forwards
                Node currentFirst = expandForwards(current, expansionDist, nodes);
                //expand backwards
                Node currentLast = expandBackwards(next, expansionDist, nodes);
        
                
                result.addAll(segmentize(last, currentFirst, events.get(i), maxLength, nodes));
                last = currentFirst;
                if(currentFirst.cost < currentLast.cost){
                    result.addAll(segmentize(currentFirst, currentLast, null, maxLength, nodes));
                    last = currentLast;
                }

                
            }else{
                double goalCost;
                Node currentNode;
                double diff = Math.abs(next.cost - current.cost);
                if(diff == 0) continue;
                    // expand backwards
                    currentNode = expandBackwards(next, diff/2, nodes);
                    
                
                double approachSpeed = scenario.vehicle.getMaxSpeedFromDistance(diff/4);

                List<PathSegment> segments = segmentize(last, currentNode, events.get(i), maxLength, nodes);
//                segment.goalVel = approachSpeed; //FIXME: IS THIS NEEDED?
                
                result.addAll(segments);
                last = currentNode;
            }
        }
        
        // For the last corner: check if there is enough space between the end of the last corner and the finish
//        if(path.getLast().cost - events.get(events.size() - 1).end.cost > 2*expansionDist){
//            // If yes:
//            Node currentNode = path.getLast();
//            double goalCost = events.get(events.size() - 1).end.cost + expansionDist;
//            while(currentNode.cost > goalCost){
//                currentNode = currentNode.parent;
//            }
//            result.add(segmentize(last, currentNode, null));
//            last = currentNode;
//        }else{
            //If no:
            Node currentNode = path.getLast();
//            double goalCost = currentNode.cost - expansionDist*2;
            currentNode = expandBackwards(currentNode, expansionDist*2, nodes);
            
//            while(currentNode.parent != null && currentNode.cost > goalCost){
//                currentNode = currentNode.parent;
//            }
            if(currentNode == last || currentNode.cost <= last.cost){
                result.addAll(segmentize(last, path.getLast(), null, maxLength, nodes));

            }else{
                result.addAll(segmentize(last, currentNode, null, maxLength, nodes));
                result.addAll(segmentize(currentNode, path.getLast(), null, maxLength, nodes));
            }
//        }
        
        
        Node last2 = null;
        for(int i = 0; i < nodes.size(); i++){
            Node current = nodes.get(i);
            if(i > 0 && current.parent != last2) throw new IllegalArgumentException();
            last2 = current;
        }
        return result;
        
    }
   

    private List<PathSegment> segmentize(Node start, Node end, CornerEvent event, double maxLength, ArrayList<Node> nodes){
        List<PathSegment> result = new ArrayList<PathSegment>();
        
        Node current = start;
        
        int segments = (int) Math.ceil((end.distance - current.distance) / maxLength);
        double targetLength = (end.distance - current.distance) / segments;
        
        for(int i = 0; i < segments - 1; i++){
            Node segmentLast = expandForwards(current, targetLength, nodes);
            result.add(new PathSegment(current, segmentLast));
            current = segmentLast;
        }
        
//        while(end.distance - current.distance > maxLength){
//            double targetLength = maxLength;
//            if(end.distance - current.distance < 2* maxLength){
//                targetLength = (end.distance - current.distance) / 2;
//            }
//            
//            
//            Node segmentLast = expandForwards(current, targetLength);
////            Node segmentLast = current;
////            while(segmentLast.getChild().distance - current.distance < targetLength){
////                segmentLast = segmentLast.getChild();
////            }
//            result.add(new PathSegment(current, segmentLast));
//            current = segmentLast;
//        }
        
        if(event != null){
            result.add(new PathSegment(current, end, event.regions));
        }else{
            result.add(new PathSegment(current, end));
        }
        return result; 
    }

}
