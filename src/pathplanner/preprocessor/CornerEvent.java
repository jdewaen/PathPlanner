package pathplanner.preprocessor;

import java.util.ArrayList;
import java.util.List;


public class CornerEvent implements Comparable<CornerEvent>{
    public final PathNode start;
    public final PathNode end;    
    
    public CornerEvent(PathNode start, PathNode end){
        this.start = start;
        this.end = end;
    }

    @Override
    public int compareTo(CornerEvent o) {
        int startComp = Double.compare(start.distance, o.start.distance);
        if(startComp == 0){
            return Double.compare(end.distance, o.end.distance);
        }else{
            return startComp;
        }
    }
    
    public boolean overlaps(CornerEvent o) {
        if(this.start.distance >= o.start.distance && this.start.distance <= o.end.distance) return true;
        if(o.start.distance >= this.start.distance && o.start.distance <= this.end.distance) return true;
        return false;
    }
   
    
    public static List<CornerEvent> generateEvents(PathNode path, double maxDeltaCost){
        ArrayList<PathNode> list = path.toArrayList();
        List<CornerEvent> result = new ArrayList<CornerEvent>();
        int i = 0;
        
        while(i < list.size() - 1){
            PathNode current = list.get(i);

            PathNode lastNodeOfCorner = current;
            PathNode currentCornerNode = list.get(i+1);
            int turnDirection = lastNodeOfCorner.getTurnDirection();
            if(turnDirection == 0){
                System.out.println("NO TURN AT: " + lastNodeOfCorner.pos.toPrettyString());
                i++;
                continue;
            }
            i++;
            while(i < list.size() - 1){
                currentCornerNode = list.get(i);
                PathNode nextNode = list.get(i+1);
                if(currentCornerNode.distance - lastNodeOfCorner.distance > maxDeltaCost){
                    break;
                }
                if(currentCornerNode.getTurnDirection() != turnDirection){
                    break; //TODO: check distance,  don't break if really close (half? quarter?)
                    // DIRECTION CHANGED!!! MAKE EVENT AND KEEP GOING HERE
                }else{
                    lastNodeOfCorner = currentCornerNode;
                    currentCornerNode = nextNode;
                    i++;
                    continue;
                    // SAME DIRECTION: UPDATE NODE
                }

            }
            
            // Corner is fully expanded
            CornerEvent event = new CornerEvent(current, lastNodeOfCorner);
            current = currentCornerNode;
            result.add(event);
        }
        return result;
        
    }
    
 
//    public static List<CornerEvent> generateEvents(List<Pair<Node, Obstacle2DB>> nodes, double maxDeltaCost, double expansionDist){
//        List<CornerEvent> result = new ArrayList<CornerEvent>();
//        
//        Map<Obstacle2DB, List<Node>> eventNodes = new HashMap<Obstacle2DB, List<Node>>();
//        
//        
//        for(Pair<Node, Obstacle2DB> pair : nodes){
//            Node node = pair.getKey();
//            Obstacle2DB region = pair.getValue();
//            
//            if(! eventNodes.containsKey(region)) eventNodes.put(region, new ArrayList<Node>());
//            eventNodes.get(region).add(node);
//        }
//        
//        
//        List<CornerEvent> tempEvents = new ArrayList<CornerEvent>();
//        for(Obstacle2DB region : eventNodes.keySet()){
//            List<Node> currentNodes = eventNodes.get(region);
//            Collections.sort(currentNodes);
//            
//            Node startNode = currentNodes.get(0);
//            Node lastNode = currentNodes.get(0);
//            double lastCost = startNode.cost;
//            int i = 1;
//            while(i < currentNodes.size()){
//                Node currentNode = currentNodes.get(i);
//                if(currentNode.cost > lastCost + maxDeltaCost){
//                    tempEvents.addAll(splitIfNeeded(startNode, lastNode, region, expansionDist));
//                    startNode = currentNode;
//                }
//                i++;
//                lastNode = currentNode;
//                lastCost = currentNode.cost;
//            }
//            
//            tempEvents.addAll(splitIfNeeded(startNode, currentNodes.get(currentNodes.size()-1), region, expansionDist));
//        }
//        
//        Collections.sort(tempEvents);
//        if(tempEvents.isEmpty()) return tempEvents;
//        CornerEvent lastEvent = tempEvents.get(0);
//        for( int i = 1; i < tempEvents.size(); i++){
//            CornerEvent currentEvent = tempEvents.get(i);
//            
//            if(lastEvent.overlaps(currentEvent)){
//                
//                CornerEvent merged = lastEvent.merge(currentEvent);
//                boolean foundIntersect = false;
//                for(Obstacle2DB region : merged.regions){
//                    if(!region.intersects(merged.start.pos, merged.end.pos, 0)){
//                        //  region doesn't intersect line between start and end of event
//                        CornerEvent first = new CornerEvent(lastEvent.start, currentEvent.start, lastEvent.regions);
//                        CornerEvent second = new CornerEvent(currentEvent.start, currentEvent.end, currentEvent.regions);
//                        result.add(first);
//                        lastEvent = second;
//                        foundIntersect = true;
//                        break;
//                    }
//                }
//                
//                if(!foundIntersect){
//                    lastEvent = merged;
//                }
//                
//            }else{
//                result.add(lastEvent);
//                lastEvent = currentEvent;
//            }
//            
//
//        }
//        result.add(lastEvent);
//        Collections.sort(result);
//        return result;
//        
//    }
//    
//    private static List<CornerEvent> splitIfNeeded(Node start, Node end, Obstacle2DB region, double expansionDist){
//        List<CornerEvent> result = new ArrayList<CornerEvent>();
//        if(start.cost > end.cost) return result;
//        CornerEvent event = new CornerEvent(start, end, region, expansionDist);
//        if(start == end || end.parent == start){
//            result.add(event);
//            return result;
//        }
//        if(!region.intersects(event.start.pos, event.end.pos, 0)){
//            Node middle = Node.split(start, end);
//            result.addAll(splitIfNeeded(start, middle, region, expansionDist));
//            result.addAll(splitIfNeeded(middle.getChild(), end, region, expansionDist));
//        }else{
//            result.add(event);
//        }
//        return result;
//    }


}
