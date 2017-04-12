package pathplanner.preprocessor;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.Set;

import pathplanner.common.Pos2D;


public class Node implements Comparable<Node>{
    Set<Node> children = new HashSet<Node>();
    public final Pos2D pos;
    public Node parent;
    public final double cost;
    public final double distance;
    
    public Node(Node parent, Pos2D pos, double cost, double distance){
        this.pos = pos;
        this.parent = parent;
        this.cost = cost;
        this.distance = distance;
        if(parent != null){
            parent.children.add(this);
        }
    }
    public Node(Node parent, Pos2D pos, double distance){
        this(parent, pos, distance, distance);
    }
    
    public double distanceFrom(Node other){
        return other.pos.distanceFrom(pos);
    }
    
    public LinkedList<Node> getPath(){
        LinkedList<Node> result = new LinkedList<Node>();
        result.addFirst(this);
        Node last = this;
        Node current = parent;
        while(current != null){
            result.addFirst(current);
            Set<Node> currentChildren = new HashSet<Node>(current.children);
            for(Node child : currentChildren){
                if(child != last) current.children.remove(child);
            }
            last = current;
            current = current.parent;
        }
        return result;
    }

    @Override
    public int compareTo(Node other) {
        return Double.compare(cost, other.cost);
    }

    
    public Node getChild(){
        for(Node child : children){
            return child;
        }
        return null;
    }
    
    public void setChild(Node other){
        children.clear();
        children.add(other);
    }
    
    public void setParent(Node other){
        this.parent = other;
    }
    
    
    public static Node split(Node start, Node end){
        double halfCost = (end.cost + start.cost)/2;
        Node current = end.parent;
        if(current == null){
            return end;
        }
        while(current.cost > halfCost){
            current = current.parent;
        }
        return current;
    }
    
    
    public int getTurnDirection(Node next){
        if(parent == null || next == null) return 0;
        Node last = parent;
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
