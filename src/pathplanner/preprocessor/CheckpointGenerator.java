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
    
    
    private List<Pair<Node, Obstacle2DB>> getCornerNodesOld(LinkedList<Node> path, double gridSize){
        List<Pair<Node, Obstacle2DB>> result = new ArrayList<Pair<Node, Obstacle2DB>>();
//        if(path.size() < 2) return result;
        Node last = path.get(0);
        Node current = path.get(1);
//        Pos2D lastDelta = current.pos.minus(last.pos);
        last = current;
                
        for(int i = 2; i < path.size(); i++){
            current = path.get(i);
            System.out.println(current.pos.toPrettyString());
//            Pos2D currentDelta = current.pos.minus(last.pos);
//            if(!current.distanceFrom(last)){
                for(int r = 0; r < obstacles.size(); r++){
                    for(int v = 0; v < vertices.get(r).size(); v++){
                        if( current.pos.fuzzyEquals(vertices.get(r).get(v), 1.1*gridSize)){
                            result.add(new Pair<Node, Obstacle2DB>(current, obstacles.get(r)));
                            break;
                        }
                    }
                }
//            }
//            lastDelta = currentDelta;
            last = current;
        }
        
        return result;
    }
    
    private Map<Node, Set<Obstacle2DB>> getCornerNodes(LinkedList<Node> path,
            double gridSize) {
        Map<Node, Set<Obstacle2DB>> result = new HashMap<Node, Set<Obstacle2DB>>();
        Node last = path.get(0);
        Node current = path.get(1);
        Pos2D lastDelta = current.pos.minus(last.pos);
        last = current;
        Set<Obstacle2DB> candidates = new HashSet<Obstacle2DB>();
        for (int i = 2; i < path.size(); i++) {
            current = path.get(i);
            Pos2D currentDelta = current.pos.minus(last.pos);
            if (!currentDelta.fuzzyEquals(lastDelta, 0.001)) {
                candidates.clear();
                for (int r = 0; r < obstacles.size(); r++) {
                    for (int v = 0; v < vertices.get(r).size(); v++) {
                        if (current.pos.fuzzyEquals(vertices.get(r).get(v), 2 * gridSize) ||
                            last.pos.fuzzyEquals(vertices.get(r).get(v), 2 * gridSize)
                                ) {
                            candidates.add(obstacles.get(r));
                            break;
                        }
                    }
                }

                if (isObstacleCorner(current, candidates, 10)) {
                    result.put(last, candidates);
                }

            }
            lastDelta = currentDelta;
            last = current;
        }

        return result;
    }
    
    private boolean isObstacleCorner(Node start, Set<Obstacle2DB> candidates, int maxSteps){
        return false;
//        if(start.isLast()) return false;
//        Node target = start.getChild();
//        Node last = start.parent;
//        Node current = last.parent;
//        Pos2D startDelta = current.pos.minus(last.pos);
//        
//        for(int i = 0; i < maxSteps; i++){
//            last = current;
//            current = current.parent;
//            if(current == null) return false;
//            Pos2D currentDelta = current.pos.minus(last.pos);
//            for(Obstacle2DB obs : candidates){
//                if(obs.intersects(target.pos, current.pos, 0)) return true;
//            }
//            if(!currentDelta.fuzzyEquals(startDelta, 0.001)){
//                return false;
//            }
//            
//        }
//        return false;
            
    }
    
    
    
//    public List<Pos2D> generateFromPath(LinkedList<Node> path, double gridSize){
//        List<Pos2D> result = new ArrayList<Pos2D>();        
//        Node last = path.get(0);
//        Node current = path.get(1);
//        Pos2D lastDelta = current.pos.minus(last.pos);
//        last = current;
//        
//        Region2D lastObs = null;
//        
//        for(int i = 2; i < path.size(); i++){
//            current = path.get(i);
//            Pos2D currentDelta = current.pos.minus(last.pos);
//            if(!currentDelta.fuzzyEquals(lastDelta, 0.001)){
//                for(int r = 0; r < obstacles.size(); r++){
//                    for(int v = 0; v < vertices.get(r).size(); v++){
//                        if( current.pos.fuzzyEquals(vertices.get(r).get(v), 1.1*gridSize)){
//                            result.add(current.pos);
//                            lastObs = obstacles.get(r);
//                            break;
//                        }
//                    }
//                }
//            }
//            lastDelta = currentDelta;
//            last = current;
//        }
//        
//        return result;
//    }
    
    public List<CornerEvent> generateCornerEvents(LinkedList<Node> path, double gridSize, double tolerance){        
//        Map<Node, Set<Obstacle2DB>> cornersNodes = getCornerNodes(path, gridSize);
        List<CornerEvent> corners = CornerEvent.generateEvents3(path, scenario.vehicle.getAccDist() * tolerance, path.getFirst());
        
        return corners;
    }
    
    public List<PathSegment> generateFromPath(LinkedList<Node> path, double gridSize, List<CornerEvent> corners, double margin, double maxTime){
        

           
        List<PathSegment> result = expandCornerEvents(corners, path, scenario.vehicle.getAccDist() * margin, scenario.vehicle.maxSpeed * maxTime);
        
        return result;

        
    };
    
    private Node expandForwards(Node start, double distance){
        double goal = start.cost + distance;
        Node current = start;
        while(current.cost < goal && current.getChild() != null){
            current = current.getChild();
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
            return inter;
        }
        
        
    }
    
    private Node expandBackwards(Node start, double distance){
        double goal = start.cost - distance;
        Node current = start;
        while(current.cost > goal && current.parent != null){
            current = current.parent;
        }
        if(current.cost > goal){
            return current;
        }else{
            Node child = current.getChild();
            Pos2D diff = child.pos.minus(current.pos);
            diff = diff.normalize();
            diff = diff.multiply(goal - current.cost);
            Pos2D newPos = current.pos.plus(diff);
            Node inter = new Node(current, newPos, goal, goal);
            current.setChild(inter);
            child.setParent(inter);
            inter.setChild(child);
            return inter;
        }
        
    }
    
    private List<PathSegment> expandCornerEvents(List<CornerEvent> events, LinkedList<Node> path, double expansionDist, double maxLength){
        List<PathSegment> result = new ArrayList<PathSegment>();
        
        Node last = path.getFirst();
        
        if(events.isEmpty()){
        	result.addAll(segmentize(last, path.getLast(), null, maxLength));
        	return result;
        }
        
        // If there is plenty of room before the first corner:
        // Make a segment from the start to the expanded first corner
        if(events.get(0).start.cost > expansionDist){
            Node currentNode = events.get(0).start;
            
            // expand backwards
//            double goalCost = currentNode.cost - expansionDist;
            currentNode = expandBackwards(currentNode, expansionDist);
//            while(currentNode.cost > goalCost){
//                currentNode = currentNode.parent;
//            }
            
            
            if(currentNode.cost > last.cost){
                result.addAll(segmentize(last, currentNode, null, maxLength));
                last = currentNode;
            }
        }
        
        // For each but the last corner corner
        for(int i = 0; i < events.size() - 1; i++){
            Node current = events.get(i).end;
            Node next = events.get(i + 1).start;
            if(next.cost - current.cost > 3 * expansionDist){
                
                
                //expand forwards
                Node currentFirst = expandForwards(current, expansionDist);
                //expand backwards
                Node currentLast = expandBackwards(next, expansionDist);
        
                
                result.addAll(segmentize(last, currentFirst, events.get(i), maxLength));
                last = currentFirst;
                if(currentFirst.cost < currentLast.cost){
                    result.addAll(segmentize(currentFirst, currentLast, null, maxLength));
                    last = currentLast;
                }

                
            }else{
                double goalCost;
                Node currentNode;
                double diff = Math.abs(next.cost - current.cost);
                if(diff == 0) continue;
                    // expand backwards
                    currentNode = expandBackwards(next, diff/2);
                    
                
                double approachSpeed = scenario.vehicle.getMaxSpeedFromDistance(diff/4);

                List<PathSegment> segments = segmentize(last, currentNode, events.get(i), maxLength);
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
            currentNode = expandBackwards(currentNode, expansionDist*2);
            
//            while(currentNode.parent != null && currentNode.cost > goalCost){
//                currentNode = currentNode.parent;
//            }
            if(currentNode == last || currentNode.cost <= last.cost){
                result.addAll(segmentize(last, path.getLast(), null, maxLength));

            }else{
                result.addAll(segmentize(last, currentNode, null, maxLength));
                result.addAll(segmentize(currentNode, path.getLast(), null, maxLength));
            }
//        }
        
        
        
        return result;
        
    }
   

    private List<PathSegment> segmentize(Node start, Node end, CornerEvent event, double maxLength){
        List<PathSegment> result = new ArrayList<PathSegment>();
        
        Node current = start;
        
        int segments = (int) Math.ceil((end.distance - current.distance) / maxLength);
        double targetLength = (end.distance - current.distance) / segments;
        
        for(int i = 0; i < segments - 1; i++){
            Node segmentLast = expandForwards(current, targetLength);
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
