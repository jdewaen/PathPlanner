package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import com.sun.prism.shape.ShapeRep.InvalidationType;

import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.World2D;


public class WorldSegment extends World2D{
    
    int ENTRANCE_SPLIT_SIZE = 3;
    
    Map<Neighbor, WorldSegment> neighbors= new HashMap<Neighbor, WorldSegment>();
    Map<WorldSegment, Neighbor> neighborsInv = new HashMap<WorldSegment, Neighbor>();
    Map<Neighbor, List<Pos2D>> entrances = new HashMap<Neighbor,List<Pos2D>>();
    Map<Pos2D,Map<Pos2D, LinkedList<Node>>> generatedPaths;
    final World2D parent;

    public WorldSegment(Pos2D minPos, Pos2D maxPos, World2D parent) {
        super(minPos, maxPos);
        this.parent = parent;
        
        parent.getObstacles().stream().filter(obs -> isInside(obs)).forEach(obs -> addObstacle(obs));
    }
    
    public void setNeighbor(Neighbor direction, WorldSegment segment){
        if(neighbors.containsKey(direction) && neighbors.get(direction) != segment)
            throw new IllegalArgumentException("Neighbor already set to something else!");
        neighbors.put(direction, segment);
        neighborsInv.put(segment, direction);
    }
    
    public Optional<WorldSegment> getNeighbor(Neighbor direction){
        WorldSegment result = neighbors.get(direction);
        return Optional.ofNullable(result);
    }
    
    public void generateEntrances(double gridSize){
        entrances.clear();
        neighbors.keySet().stream().forEach(dir -> generateEntrances(dir, gridSize));
    }
    
    public Set<WorldSegment> getNeighbors(){
        return neighborsInv.keySet();
    }
    
    private void generateEntrances(Neighbor direction, double gridSize){
        List<Pos2D> result = new ArrayList<Pos2D>();
        List<Pos2D> candidates = getEdgePositions(direction, gridSize);
        for(int i = 0; i < candidates.size(); i++){
            Pos2D current = candidates.get(i);
            if(isInsideAnyObstacle(current)) continue; //TODO: check with vehicle size
            List<Pos2D> currentEntrance = new ArrayList<Pos2D>();
            currentEntrance.add(current);
            i++;
            while(i < candidates.size()){
                current = candidates.get(i);
                if(isInsideAnyObstacle(current)) break;
                currentEntrance.add(current);
                i++;
            }
            if(currentEntrance.size() >= ENTRANCE_SPLIT_SIZE * gridSize){
                result.add(currentEntrance.get(1));
//                result.add(currentEntrance.get(currentEntrance.size() / 2));
                result.add(currentEntrance.get(currentEntrance.size() - 2));
            }else{
                result.add(currentEntrance.get(currentEntrance.size() / 2));
            }
                
        }
        entrances.put(direction, result);
    }
    
    public boolean isConnectedTo(WorldSegment segment){
        if(!neighborsInv.keySet().contains(segment)) return false;
        Neighbor dir = neighborsInv.get(segment);
        return !entrances.get(dir).isEmpty();
    }
    
    public List<Pos2D> getEdgePositions(Neighbor direction, double gridSize){
        double min = direction.horizontalEdge ? minPos.x : minPos.y;
        double max = direction.horizontalEdge ? maxPos.x : maxPos.y;
        
        double otherCoord = direction.horizontalEdge ? (direction.minEdge ? minPos.y : maxPos.y) : (direction.minEdge ? minPos.x : maxPos.x);
        
        List<Pos2D> result = new ArrayList<Pos2D>();
        
        for(double i = min; i <= max; i+= gridSize){
            result.add(direction.horizontalEdge ? new Pos2D(i ,otherCoord) : new Pos2D(otherCoord, i));
        }
        return result;
    }
    
//    public Map<Pos2D, List<Pos2D>> getPermutations(WorldSegment prev, WorldSegment next){
//        Map<Pos2D, List<Pos2D>> result = new HashMap<Pos2D, List<Pos2D>>();
//        if(prev == next) return result;
//        if(!isConnectedTo(prev) || !isConnectedTo(next)) return result;
//        List<Pos2D> goals = entrances.get(neighborsInv.get(next));
//        return entrances.get(neighborsInv.get(prev)).stream().collect(Collectors.toMap(pos -> pos, pos -> goals)); 
//    }
    
//    private void generatePaths(Scenario scenario, double gridSize){
//        ThetaStar algo = new ThetaStar(scenario, this);
//        generatedPaths = new HashMap<Pos2D, Map<Pos2D,LinkedList<Node>>>();
//        for(Map.Entry<Neighbor, List<Pos2D>> entry : entrances.entrySet()){
//            Neighbor direction = entry.getKey();
//            List<Pos2D> starts = entry.getValue();
//            List<Pos2D> goals = entrances.entrySet().stream()
//                    .filter(e -> e.getKey() != direction)
//                    .flatMap(e -> e.getValue().stream())
//                    .collect(Collectors.toList());
//            
//            starts.stream().forEach( start -> {
//                Map<Pos2D, LinkedList<Node>> paths = goals.stream().collect(Collectors.toMap(g -> g, g -> new LinkedList<Node>()));
//                algo.solve(gridSize, start, paths);
//                generatedPaths.put(start, paths);
//            });
//            
//        }
//    }
    

    
    public WorldSegment getNeighborSegmentFromPos(Pos2D pos){
        return neighbors.get(entrances.entrySet().stream()
                .filter(e -> e.getValue().contains(pos))
                .map(e -> e.getKey())
                .findAny().get());
    }
}

