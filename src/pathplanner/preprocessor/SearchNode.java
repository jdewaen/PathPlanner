package pathplanner.preprocessor;

import java.util.HashSet;
import java.util.Set;

import pathplanner.common.Pos2D;


public class SearchNode implements Comparable<SearchNode>{
    Set<SearchNode> children = new HashSet<SearchNode>();
    public final Pos2D pos;
    public SearchNode parent;
    public final double cost;
    public final double distance;
    
    public SearchNode(SearchNode parent, Pos2D pos, double cost, double distance){
        this.pos = pos;
        this.parent = parent;
        this.cost = cost;
        this.distance = distance;
        if(parent != null){
            parent.children.add(this);
        }
    }
    public SearchNode(SearchNode parent, Pos2D pos, double distance){
        this(parent, pos, distance, distance);
    }
    
    public double distanceFrom(SearchNode other){
        return other.pos.distanceFrom(pos);
    }
    
    public PathNode getPath(){
        PathNode result = new PathNode(null, pos, distance);
        SearchNode last = this;
        SearchNode current = parent;
        while(current != null){
            result.insertBefore(new PathNode(null, current.pos, current.distance));
            result = result.getParent();
            Set<SearchNode> currentChildren = new HashSet<SearchNode>(current.children);
            for(SearchNode child : currentChildren){
                if(child == last) {
                    last = current;
                    current = current.parent;
                    break;
                }
            }
        }
        return result;
    }

    @Override
    public int compareTo(SearchNode other) {
        return Double.compare(cost, other.cost);
    }

    
//    public Node getChild(){
//        for(Node child : children){
//            return child;
//        }
//        return null;
//    }
    
    public void setChild(SearchNode other){
        children.clear();
        children.add(other);
    }
    
    public void setParent(SearchNode other){
        this.parent = other;
    }
    
   
    
    
    public int getTurnDirection(SearchNode next){
        if(parent == null || next == null) return 0;
        SearchNode last = parent;
//        Node next = getChild();
        Pos2D lastDelta = pos.minus(last.pos);
        Pos2D currentDelta = next.pos.minus(pos);
        if(currentDelta.fuzzyEquals(lastDelta, 0.001)) return 0;
        double dp = lastDelta.x * currentDelta.y - lastDelta.y * currentDelta.x;
        if (dp > 0)
            return 1;
        else{
            return -1;
        }
    }
    
    public String toString(){
        return pos.toPrettyString() + ":" + String.valueOf(cost);
    }
}
