package pathplanner.preprocessor;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;

import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.World2D;


public class HPAStar<A extends GridSearchAlgorithm> {
    
    public final Scenario scenario;
    public WorldSegment[][] segments;
    public WorldSegment startSegment = null;
    public WorldSegment endSegment = null;
    public final A algoPrototype;
    public HPAStar(Scenario scenario, double gridSize, int size, A algoPrototype){
        this.scenario = scenario;
        this.algoPrototype = algoPrototype;
        segments = initSegments(gridSize, size);
    }

    
    private WorldSegment[][] initSegments(double gridSize, int size){

        World2D world = scenario.world;

        double xStart = world.getMinPos().x;
        double yStart = world.getMinPos().y;
        
        double xSize = world.getMaxPos().x - world.getMinPos().x;
        double ySize = world.getMaxPos().y - world.getMinPos().y;
        
        int divisionsX = (int) (xSize / (gridSize * size));
        int divisionsY = (int) (ySize / (gridSize * size));
        
        double xStep = xSize / divisionsX;
        double yStep = ySize / divisionsY;
        
        WorldSegment[][] segments = new WorldSegment[divisionsX][divisionsY];
        
        for(int i = 0; i < divisionsX; i++){
            for(int j = 0; j < divisionsY; j++){
                Pos2D minPos = new Pos2D(xStart + xStep*i, yStart + yStep*j);
                Pos2D maxPos = new Pos2D(xStart + xStep*(i+1), yStart + yStep*(j+1));
                WorldSegment current = new WorldSegment(minPos, maxPos, world);
                segments[i][j] = current;
                if(current.isInside(scenario.startPos)) startSegment = current;
                if(current.isInside(scenario.goal)) endSegment = current;
            }
        }
        
        for(int i = 0; i < divisionsX; i++){
            for(int j = 0; j < divisionsY; j++){
//                System.out.println(divisions * i + j);
                WorldSegment current = segments[i][j];
                getNeighbors(i, j, divisionsX, divisionsY, segments).entrySet().stream().forEach(e -> current.setNeighbor(e.getKey(), e.getValue()));
//                System.out.println("entrances");
                current.generateEntrances(gridSize);
                //System.out.println("paths");
                //current.generatePaths(scenario, gridSize);
            }
        }
        return segments;
    }

    private Map<Neighbor, WorldSegment> getNeighbors(int i, int j, int divisionsX, int divisionsY, WorldSegment[][] segments){
        Map<Neighbor, WorldSegment> result = new HashMap<Neighbor, WorldSegment>();
        if(i > 0) result.put(Neighbor.LEFT, segments[i-1][j]);
        if(i < divisionsX - 1) result.put(Neighbor.RIGHT, segments[i+1][j]);
        if(j > 0) result.put(Neighbor.DOWN, segments[i][j-1]);
        if(j < divisionsY - 1) result.put(Neighbor.UP, segments[i][j+1]);
        return result;
    }

    public LinkedList<Node> solve(double gridSize){        
        PriorityQueue<HPANode> queue = new PriorityQueue<HPANode>();
        Set<Pos2D> alreadyDone = new HashSet<Pos2D>();
        queue.addAll(initialPath(startSegment, scenario, gridSize));
        Node last = null;
//        alreadyDone.add(scenario.startPos);
        while(queue.size() != 0){
            HPANode current = queue.poll();
            if(alreadyDone.contains(current.pos)) continue;
            alreadyDone.add(current.pos);
            if(current.target == endSegment){
                HPANode finalNode = finalPath(current.target, scenario, gridSize, current);
                if(finalNode != null)
                    return finalNode.getPath();
            }
            
            Set<HPANode> neighbors = expand(current.target, scenario, gridSize, current, alreadyDone);
            queue.addAll(neighbors);
            last = current;
        }
        
        return last.getPath();
        
    }
    
    public HPANode finalPath(WorldSegment segment, Scenario scenario, double gridSize, HPANode prev){
        @SuppressWarnings("unchecked")
        A algo = (A) algoPrototype.buildAlgo(scenario, segment); // theta2 !!!
        LinkedList<Node> path = algo.solve(gridSize, prev.pos);
        if(!path.isEmpty())
            return new HPANode(prev, scenario.goal, null, path);
        return null;
    }
    
    public Set<HPANode> initialPath(WorldSegment segment, Scenario scenario, double gridSize){ 
        @SuppressWarnings("unchecked")
        A algo = (A) algoPrototype.buildAlgo(scenario, segment);
        List<Pos2D> goals = segment.entrances.entrySet().stream()
                .flatMap(e -> e.getValue().stream())
                .collect(Collectors.toList());
        
        Map<Pos2D, LinkedList<Node>> paths = goals.stream().distinct().collect(Collectors.toMap(g -> g, g -> new LinkedList<Node>()));
        algo.solve(gridSize, scenario.startPos, paths);
        
        HPANode root = new HPANode(null, scenario.startPos, segment, new LinkedList<Node>());
        return paths.entrySet().stream()
                .filter(e -> !e.getValue().isEmpty())
                .map(e -> new HPANode(root, e.getKey(), segment.getNeighborSegmentFromPos(e.getKey()), e.getValue()))
                .collect(Collectors.toSet());
    }
    
    public Set<HPANode> expand(WorldSegment segment, Scenario scenario, double gridSize, HPANode prev, Set<Pos2D> alreadyDone){
        @SuppressWarnings("unchecked")
        A algo = (A) algoPrototype.buildAlgo(scenario, segment);
        List<Pos2D> goals = segment.entrances.entrySet().stream()
                .flatMap(e -> e.getValue().stream())                
                .filter(e -> !alreadyDone.contains(e))
                .collect(Collectors.toList());
        
        Map<Pos2D, LinkedList<Node>> paths = goals.stream().distinct().collect(Collectors.toMap(g -> g, g -> new LinkedList<Node>()));
        algo.solve(gridSize, prev.pos, paths);
        
        return paths.entrySet().stream()
                .filter(e -> !e.getValue().isEmpty())
                .map(e ->new HPANode(prev, e.getKey(), segment.getNeighborSegmentFromPos(e.getKey()), e.getValue()))
                .collect(Collectors.toSet());
    }
    

}

