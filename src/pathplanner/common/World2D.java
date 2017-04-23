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

    private ArrayList<Obstacle2DB> obstacles = new ArrayList<Obstacle2DB>();
    private Map<Integer, Map<Integer, Set<Obstacle2DB>>> obsIndex = new HashMap<Integer, Map<Integer,Set<Obstacle2DB>>>();
    public final double INDEX_GRID_SIZE = 50;
    
    protected final Pos2D maxPos;
    protected final Pos2D minPos;
    
    private final Rectangle2D shape;
    
    public World2D(Pos2D maxPos){
        this(new Pos2D(0, 0), maxPos);
    }
    
    
    public World2D(Pos2D minPos, Pos2D maxPos){
        this.maxPos = maxPos;
        this.minPos = minPos;
        
        Pos2D diff = maxPos.minus(minPos);
        this.shape = new Rectangle2D.Double(minPos.x, minPos.y, diff.x, diff.y);
    }
    
    
    public void addObstacle(Obstacle2DB obs){
            obstacles.add(obs);
            Pos2D minCoords = new Pos2D(obs.boundingBox.getMinX(), obs.boundingBox.getMinY());
            Pos2D maxCoords = new Pos2D(obs.boundingBox.getMaxX(), obs.boundingBox.getMaxY());
            getObstacleSetsForPositions(minCoords, maxCoords).stream().forEach(set -> set.add(obs));
    }
    
    public List<Set<Obstacle2DB>> getObstacleSetsForPositions(Pos2D pos1, Pos2D pos2){
        List<Set<Obstacle2DB>> result = new ArrayList<Set<Obstacle2DB>>();
        Set<Integer> xIndices = interPolateBetween(indexFromCoord(pos1.x), indexFromCoord(pos2.x));
        Set<Integer> yIndices = interPolateBetween(indexFromCoord(pos1.y), indexFromCoord(pos2.y));
        for(Integer x : xIndices){
            for(Integer y : yIndices){
                result.add(getSetAtIndices(x, y));
            }
        }
        return result;
    }
    
    public Set<Obstacle2DB> getObstaclesForPositions(Pos2D pos1, Pos2D pos2){
        return getObstacleSetsForPositions(pos1, pos2).stream().flatMap(set -> set.stream()).collect(Collectors.toSet());
    }
    
    private Set<Obstacle2DB> getSetAtIndices(Integer x, Integer y){
        if(obsIndex.containsKey(x)){
            Map<Integer, Set<Obstacle2DB>> xMap = obsIndex.get(x);
            if(xMap.containsKey(y)){
                return xMap.get(y);
            }else{
                Set<Obstacle2DB> obsSet = new HashSet<Obstacle2DB>();
                xMap.put(y, obsSet);
                return obsSet;
            }
        }else{
            Map<Integer, Set<Obstacle2DB>> xMap = new HashMap<Integer, Set<Obstacle2DB>>();
            Set<Obstacle2DB> obsSet = new HashSet<Obstacle2DB>();
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
   
    
    public List<Obstacle2DB> getObstacles(){
        return (List<Obstacle2DB>) Collections.unmodifiableList(obstacles);
    }
    
    public Pos2D getMinPos(){
        return minPos;
    } 
    
    public Pos2D getMaxPos(){
        return maxPos;
    }
    
    public boolean isInside(Pos2D pos){
        if(pos == null) return false;
        return (pos.x >= minPos.x && pos.x <= maxPos.x && pos.y >= minPos.y &&  pos.y <= maxPos.y);
    }
  
    
    public boolean isInside(Obstacle2DB obstacle){
        return obstacle.shape.intersects(shape);
    }
    
    public boolean isInsideAnyObstacle(Pos2D pos) {
        for (Obstacle2DB obs : obstacles) {
            if (obs.contains(pos)) { return true; }
        }
        return false;
    }
    
    public boolean intersectsAnyObstacle(Pos2D pos1, Pos2D pos2){
        for(Obstacle2DB obs : obstacles){
            if(obs.intersects(pos1, pos2, 0)){
                return true;
            }
        }
        return false;
    }

}
