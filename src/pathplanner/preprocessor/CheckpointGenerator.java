package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import javafx.util.Pair;
import pathplanner.common.Pos2D;
import pathplanner.common.Region2D;
import pathplanner.common.Scenario;


public class CheckpointGenerator {
    
    Scenario scenario;
    List<Region2D> obstacles;
    List<List<Pos2D>> vertices;
    
    public CheckpointGenerator(Scenario scenario){
        this.scenario = scenario;
        generateVertices();
    }
    
    private void generateVertices(){
        obstacles = new ArrayList<Region2D>();
        vertices = new ArrayList<List<Pos2D>>();
        List<Region2D> regions = scenario.world.getRegions();
        for(int i = 0; i < regions.size(); i++){
            Region2D region = regions.get(i);
            obstacles.add(region);
            vertices.add(region.getVertices());
        }
    }
    
    
    private List<Pair<Node, Region2D>> getCornerNodes(LinkedList<Node> path, double gridSize){
        List<Pair<Node, Region2D>> result = new ArrayList<Pair<Node, Region2D>>();        
        Node last = path.get(0);
        Node current = path.get(1);
        Pos2D lastDelta = current.pos.minus(last.pos);
        last = current;
                
        for(int i = 2; i < path.size(); i++){
            current = path.get(i);
            Pos2D currentDelta = current.pos.minus(last.pos);
            if(!currentDelta.fuzzyEquals(lastDelta, 0.001)){
                for(int r = 0; r < obstacles.size(); r++){
                    for(int v = 0; v < vertices.get(r).size(); v++){
                        if( current.pos.fuzzyEquals(vertices.get(r).get(v), 1.1*gridSize)){
                            result.add(new Pair<Node, Region2D>(current, obstacles.get(r)));
                            break;
                        }
                    }
                }
            }
            lastDelta = currentDelta;
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
    
    public List<CornerEvent> generateCornerEvents(LinkedList<Node> path, double gridSize){
        List<Pair<Node, Region2D>> cornersNodes = getCornerNodes(path, gridSize);
        List<CornerEvent> corners = CornerEvent.generateEvents(cornersNodes, getAccDist() * 2);
        
        return corners;
    }
    
    public List<Pos2D> generateFromPath(LinkedList<Node> path, double gridSize, List<CornerEvent> corners){
        

           
        List<Pos2D> result = expandCornerEvents(corners, path, getAccDist() * 3);
        
        return result;

        
    };
    
    private List<Pos2D> expandCornerEvents(List<CornerEvent> events, LinkedList<Node> path, double expansionDist){
        List<Pos2D> result = new ArrayList<Pos2D>();
        result.add(scenario.startPos);
        
        if(events.get(0).start.cost > expansionDist){
            Node currentNode = events.get(0).start;
            double goalCost = currentNode.cost - expansionDist;
            while(currentNode.cost > goalCost){
                currentNode = currentNode.parent;
            }
            result.add(currentNode.pos);
        }
        
        for(int i = 0; i < events.size() - 1; i++){
            Node first = events.get(i).end;
            Node last = events.get(i + 1).start;
            if(last.cost - first.cost > 2 * expansionDist){
                double goalFirst = first.cost + expansionDist;
                Node currentFirst = first;
                while(currentFirst.cost < goalFirst){
                    currentFirst = getChild(currentFirst);
                }
                result.add(currentFirst.pos);
                
                double goalLast = last.cost - expansionDist;
                Node currentLast = last;
                while(currentLast.cost > goalLast){
                    currentLast = currentLast.parent;
                }
                result.add(currentLast.pos);
                
            }else{
                double diff = last.cost - first.cost;
                double goalCost = first.cost + diff/2;
                Node currentNode = last;
                while(currentNode.cost > goalCost){
                    currentNode = currentNode.parent;
                }
                result.add(currentNode.pos);
            }
        }
        
        
        if(path.getLast().cost - events.get(events.size() - 1).end.cost > expansionDist){
            Node currentNode = path.getLast();
            double goalCost = events.get(events.size() - 1).end.cost + expansionDist;
            while(currentNode.cost > goalCost){
                currentNode = currentNode.parent;
            }
            result.add(currentNode.pos);
        }
        
        result.add(scenario.goal);
        
        return result;
        
    }
    
    private double getAccTime(){
        return scenario.vehicle.maxSpeed / scenario.vehicle.acceleration;
    }
    
    private double getAccDist(){
        return scenario.vehicle.acceleration *  Math.pow(getAccTime(), 2) / 2;

    }
    
    private Node getChild(Node node){
        for(Node child : node.children){
            return child;
        }
        return null;
    }

}
