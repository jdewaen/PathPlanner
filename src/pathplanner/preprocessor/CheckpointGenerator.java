package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

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
    
    
    private List<Pair<Node, Obstacle2DB>> getCornerNodes(LinkedList<Node> path, double gridSize){
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
//                        if(vertices.get(r).get(v).fuzzyEquals(new Pos2D(414, 715), 2)){
//                            System.out.println("R: " + String.valueOf(r));
//                            System.out.println("V: " + String.valueOf(v));
//                        }
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
    
    public List<CornerEvent> generateCornerEvents(LinkedList<Node> path, double gridSize, double margin, double tolerance){
        List<Pair<Node, Obstacle2DB>> cornersNodes = getCornerNodes(path, gridSize);
        List<CornerEvent> corners = CornerEvent.generateEvents(cornersNodes, scenario.vehicle.getAccDist() * tolerance, scenario.vehicle.getAccDist() * margin);
        
        return corners;
    }
    
    public List<PathSegment> generateFromPath(LinkedList<Node> path, double gridSize, List<CornerEvent> corners, double margin){
        

           
        List<PathSegment> result = expandCornerEvents(corners, path, scenario.vehicle.getAccDist() * margin);
        
        return result;

        
    };
    
    private List<PathSegment> expandCornerEvents(List<CornerEvent> events, LinkedList<Node> path, double expansionDist){
        List<PathSegment> result = new ArrayList<PathSegment>();
        
        Node last = path.getFirst();
        
        if(events.isEmpty()){
        	result.add(segmentize(last, path.getLast(), null));
        	return result;
        }
        
        // If there is plenty of room before the first corner:
        // Make a segment from the start to the expanded first corner
        if(events.get(0).start.cost > expansionDist){
            Node currentNode = events.get(0).start;
            double goalCost = currentNode.cost - expansionDist;
            while(currentNode.cost > goalCost){
                currentNode = currentNode.parent;
            }
            if(currentNode.cost > last.cost){
                result.add(segmentize(last, currentNode, null));
                last = currentNode;
            }
        }
        
        // For each but the last corner corner
        for(int i = 0; i < events.size() - 1; i++){
            Node current = events.get(i).end;
            Node next = events.get(i + 1).start;
            if(next.cost - current.cost > 2 * expansionDist){
                double goalFirst = current.cost + expansionDist;
                Node currentFirst = current;
                while(currentFirst.cost < goalFirst){
                    currentFirst = currentFirst.getChild();
                }
                
                double goalLast = next.cost - expansionDist;
                Node currentLast = next;
                while(currentLast.cost > goalLast){
                    currentLast = currentLast.parent;
                }           
                
                result.add(segmentize(last, currentFirst, events.get(i)));
                last = currentFirst;
                if(currentFirst.cost < currentLast.cost){
                    result.add(segmentize(currentFirst, currentLast, null));
                    last = currentLast;
                }

                
            }else{
                double goalCost;
                Node currentNode;
                double diff = Math.abs(next.cost - current.cost);
                if(diff == 0) continue;
                if(next.cost > current.cost){
                    goalCost = current.cost + diff/2;
                    currentNode = next;
                    while(currentNode.cost > goalCost){
                        currentNode = currentNode.parent;
                    }
                }else{
                    goalCost = current.cost - diff/2;
                    currentNode = current;
                    while(currentNode.cost > goalCost){
                        currentNode = currentNode.parent;
                    }
                }
                
                double approachSpeed = scenario.vehicle.getMaxSpeedFromDistance(diff/4);

                PathSegment segment = segmentize(last, currentNode, events.get(i));
//                segment.goalVel = approachSpeed; //FIXME: IS THIS NEEDED?
                
                result.add(segment);
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
            double goalCost = currentNode.cost - expansionDist*2;
            while(currentNode.parent != null && currentNode.cost > goalCost){
                currentNode = currentNode.parent;
            }
            if(currentNode == last || currentNode.cost <= last.cost){
                result.add(segmentize(last, path.getLast(), null));

            }else{
                result.add(segmentize(last, currentNode, null));
                result.add(segmentize(currentNode, path.getLast(), null));
            }
//        }
        
        
        
        return result;
        
    }
   

    private PathSegment segmentize(Node start, Node end, CornerEvent event){
//        Set<Node> positions = segmentizeNodes(start, end, event);
//        positions.add(start);
//        positions.add(end);
//        List<Node> nodeList = new ArrayList<Node>(positions);
//        Collections.sort(nodeList);
//        List<Pos2D> result = new ArrayList<Pos2D>();
//        for(Node node : nodeList){
//            result.add(node.pos);
//        }
        
        if(event != null){
            return new PathSegment(start, end, event.regions, scenario.vehicle.getAccDist());
        }else{
            return new PathSegment(start, end, scenario.vehicle.getAccDist());
        }
//        return result; 
    }
//    private Set<Node> segmentizeNodes(Node start, Node end, CornerEvent event){
//        Set<Node> positions = new HashSet<Node>();
//        
//        if(end.cost < start.cost) return positions;
//        
//        if(start == end || event == null || end.parent == start){
//            return positions;
//        }
//        
////        for(Region2D region : event.regions){
////            if(!region.intersects(start.pos, end.pos)){
////                if(event.parents.size() == 2){
////                    CornerEvent parentA = event.parents.get(0);
////                    CornerEvent parentB = event.parents.get(1);
////                    Node middle = Node.split(parentB.start, parentA.end);
////                    positions.addAll(segmentizeNodes(start, middle, parentA));
////                    positions.addAll(segmentizeNodes(middle, end, parentB));
////                    positions.add(start);
////                    positions.add(end);
////                }else if(event.parents.isEmpty()){
////                    Node middle = Node.split(start, end);
////                    positions.addAll(segmentizeNodes(start, middle, event));
////                    positions.addAll(segmentizeNodes(middle.getChild(), end, event));
////                    positions.add(start);
////                    positions.add(end);
////                }else{
////                    throw new RuntimeException("Unexpected amount of parents for CornerEvent");
////                }
////              break;  
////            }
////        }
//        
//        return positions;
//    }

}
