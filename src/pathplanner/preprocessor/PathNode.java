package pathplanner.preprocessor;

import java.util.ArrayList;

import pathplanner.common.Vector2D;


public class PathNode implements Comparable<PathNode>{
    public final Vector2D pos;
    protected PathNode parent;
    protected PathNode child;
    public final double distance;
    public final PathNodeType type;
    
    public PathNode(PathNode parent, PathNode child, Vector2D pos, double distance, PathNodeType type){
        this.pos = pos;
        this.parent = parent;
        this.child = child;
        this.distance = distance;
        this.type = type;
    }
    public PathNode(PathNode parent, Vector2D pos, double distance, PathNodeType type){
        this(parent, null, pos, distance, type);
    }
    
    public double distanceFrom(PathNode other){
        return other.pos.distanceFrom(pos);
    }
    
    public double pathDistanceFrom(PathNode start){
        return this.distance - start.distance;
    }

    @Override
    public int compareTo(PathNode other) {
        return Double.compare(distance, other.distance);
    }
    
    public boolean isBefore(PathNode other){
        return this.distance < other.distance;
    }
    
    public boolean isAfter(PathNode other){
        return this.distance > other.distance;
    }
    
    
    public PathNode getParent(){
        return this.parent;
    }
    
    public PathNode getChild(){
        return this.child;
    }
    public void insertAfter(PathNode other){
        other.parent = this;
        
        if(child != null){
            other.child = this.child;
            this.child.parent = other;
        }

        this.child = other;
    }
    
    public void insertBefore(PathNode other){
        if(parent != null){
            parent.insertAfter(other);
        }else{
            this.parent = other;
            other.child = this;
        }
    }
    
    public void cleanUp(){
        if(type == PathNodeType.ESSENTIAL) return;
        if(parent != null){
            parent.child = child;
        }
        if(child != null){
            child.parent = parent;
        }
    }
    
    public PathNode getFirst(){
        if(parent == null) return this;
        return parent.getFirst();
    }
    
    public PathNode getLast(){
        if(child == null) return this;
        return child.getLast();
    }
    
    public ArrayList<PathNode> toArrayList(){
        PathNode first = getFirst();
        PathNode current = first;
        int size = 1;
        while(current.getChild() != null){
            current = current.getChild();
            size++;
        }
        
        current = first;
        ArrayList<PathNode> result = new ArrayList<PathNode>(size);
        for(int i = 0; i < size; i++){
            result.add(current);
            current = current.getChild();
        }
        
        return result;    
        
    }
    
    
    
    public int getTurnDirection(){
        if(parent == null || child == null) return 0;
        Vector2D lastDelta = pos.minus(parent.pos);
        Vector2D currentDelta = child.pos.minus(pos);
        if(currentDelta.fuzzyEquals(lastDelta, 0.001)) return 0;
        double dp = lastDelta.x * currentDelta.y - lastDelta.y * currentDelta.x;
        if (dp > 0)
            return 1;
        else{
            return -1;
        }
    }
    
    public String toString(){
        return pos.toPrettyString() + ":" + String.valueOf(distance);
    }
    
    public enum PathNodeType{
        ESSENTIAL, TRANSITION
    }
}

