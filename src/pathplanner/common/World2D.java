package pathplanner.common;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;


public class World2D {

    private ArrayList<Obstacle2D> obstacles = new ArrayList<Obstacle2D>();
    private Map<Integer, Map<Integer, Set<Obstacle2D>>> obsIndex = new HashMap<Integer, Map<Integer,Set<Obstacle2D>>>();
    public final double INDEX_GRID_SIZE = 50;
    
    protected final Vector2D maxPos;
    protected final Vector2D minPos;
    
    private final Rectangle2D shape;
    
    public World2D(Vector2D maxPos){
        this(new Vector2D(0, 0), maxPos);
    }
    
    
    public World2D(Vector2D minPos, Vector2D maxPos){
        this.maxPos = maxPos;
        this.minPos = minPos;
        
        Vector2D diff = maxPos.minus(minPos);
        this.shape = new Rectangle2D.Double(minPos.x, minPos.y, diff.x, diff.y);
    }
    
    
    public void addObstacle(Obstacle2D obs){
            obstacles.add(obs);
            Vector2D minCoords = new Vector2D(obs.boundingBox.getMinX(), obs.boundingBox.getMinY());
            Vector2D maxCoords = new Vector2D(obs.boundingBox.getMaxX(), obs.boundingBox.getMaxY());
            getObstacleSetsForPositions(minCoords, maxCoords).stream().forEach(set -> set.add(obs));
    }
    
    public List<Set<Obstacle2D>> getObstacleSetsForPositions(Vector2D pos1, Vector2D pos2){
        List<Set<Obstacle2D>> result = new ArrayList<Set<Obstacle2D>>();
        Set<Integer> xIndices = interPolateBetween(indexFromCoord(pos1.x), indexFromCoord(pos2.x));
        Set<Integer> yIndices = interPolateBetween(indexFromCoord(pos1.y), indexFromCoord(pos2.y));
        for(Integer x : xIndices){
            for(Integer y : yIndices){
                result.add(getSetAtIndices(x, y));
            }
        }
        return result;
    }
    
    public Set<Obstacle2D> getObstaclesForPositions(Vector2D pos1, Vector2D pos2){
        return getObstacleSetsForPositions(pos1, pos2).stream().flatMap(set -> set.stream()).collect(Collectors.toSet());
    }
    
    private Set<Obstacle2D> getSetAtIndices(Integer x, Integer y){
        if(obsIndex.containsKey(x)){
            Map<Integer, Set<Obstacle2D>> xMap = obsIndex.get(x);
            if(xMap.containsKey(y)){
                return xMap.get(y);
            }else{
                Set<Obstacle2D> obsSet = new HashSet<Obstacle2D>();
                xMap.put(y, obsSet);
                return obsSet;
            }
        }else{
            Map<Integer, Set<Obstacle2D>> xMap = new HashMap<Integer, Set<Obstacle2D>>();
            Set<Obstacle2D> obsSet = new HashSet<Obstacle2D>();
            xMap.put(y, obsSet);
            obsIndex.put(x, xMap);
            return obsSet;
        }
    }
    
    public Integer indexFromCoord(double coord){
        return (int) (coord/INDEX_GRID_SIZE);
    }
    
    public Set<Integer> interPolateBetween(Integer a, Integer b){
        Set<Integer> result = new HashSet<Integer>();
        result.add(a);
        if( a == b) return result;
        int cur = Math.min(a, b) - 1;
        int max = Math.max(a, b) + 1;
        while(cur <= max){
            result.add(cur);
            cur++;
        }
        return result;
        
    }
   
    
    public List<Obstacle2D> getObstacles(){
        return (List<Obstacle2D>) Collections.unmodifiableList(obstacles);
    }
    
    public Vector2D getMinPos(){
        return minPos;
    } 
    
    public Vector2D getMaxPos(){
        return maxPos;
    }
    
    public boolean isInside(Vector2D pos){
        if(pos == null) return false;
        return (pos.x >= minPos.x && pos.x <= maxPos.x && pos.y >= minPos.y &&  pos.y <= maxPos.y);
    }
  
    
    public boolean isInside(Obstacle2D obstacle){
        return obstacle.shape.intersects(shape);
    }
    
    public boolean isInsideAnyObstacle(Vector2D pos) {
        for (Obstacle2D obs : obstacles) {
            if (obs.contains(pos)) { return true; }
        }
        return false;
    }
    
    public boolean intersectsAnyObstacle(Vector2D pos1, Vector2D pos2){
        for(Obstacle2D obs : obstacles){
            if(obs.intersects(pos1, pos2, 0)){
                return true;
            }
        }
        return false;
    }

}
