package pathplanner;

import java.util.ArrayList;
import java.util.List;

import ilog.concert.IloException;
import pathplanner.common.*;
import pathplanner.milpplanner.*;
import pathplanner.ui.ResultWindow;

public class Main {
       
    public static World2D generateBenchmarkWorld(){
        World2D world = new World2D(new Pos2D(40, 20));
        world.addRegion(new Obstacle2D(new Pos2D(2, 0), new Pos2D(4, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(6, 5), new Pos2D(8, 20)));
        world.addRegion(new Obstacle2D(new Pos2D(10, 0), new Pos2D(12, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(14, 5), new Pos2D(16, 20)));
        world.addRegion(new Obstacle2D(new Pos2D(18, 0), new Pos2D(20, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(22, 5), new Pos2D(24, 20)));
        world.addRegion(new Obstacle2D(new Pos2D(26, 0), new Pos2D(28, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(30, 5), new Pos2D(32, 20)));
        world.addRegion(new Obstacle2D(new Pos2D(34, 0), new Pos2D(36, 13)));
        
        return world;
    }
    
    
    public static World2D generateSpiralWorld(){
        World2D world = new World2D(new Pos2D(30, 30));
        world.addRegion(new Obstacle2D(new Pos2D(13, 12), new Pos2D(16, 13)));
        world.addRegion(new Obstacle2D(new Pos2D(13, 12), new Pos2D(14, 18)));
        world.addRegion(new Obstacle2D(new Pos2D(13, 17), new Pos2D(21, 18)));
        world.addRegion(new Obstacle2D(new Pos2D(20, 7), new Pos2D(21, 18)));
        world.addRegion(new Obstacle2D(new Pos2D(8, 7), new Pos2D(21, 8)));
        world.addRegion(new Obstacle2D(new Pos2D(8, 7), new Pos2D(9, 23)));
        world.addRegion(new Obstacle2D(new Pos2D(8, 22), new Pos2D(26, 23)));
        world.addRegion(new Obstacle2D(new Pos2D(25, 2), new Pos2D(26, 23)));
        world.addRegion(new Obstacle2D(new Pos2D(3, 2), new Pos2D(26, 3)));
        world.addRegion(new Obstacle2D(new Pos2D(3, 2), new Pos2D(4, 28)));
        world.addRegion(new Obstacle2D(new Pos2D(3, 27), new Pos2D(26, 28)));

        return world;
    }
    
    public static List<Scenario2D> benchmarkCheckpoints(Vehicle vehicle, World2D world){
        
        List<Pos2D> checkpoints = new ArrayList<Pos2D>();
        checkpoints.add(new Pos2D(1, 1));
        checkpoints.add(new Pos2D(1, 8));
        checkpoints.add(new Pos2D(5, 8));
        checkpoints.add(new Pos2D(9, 8));
        checkpoints.add(new Pos2D(13, 8));
        checkpoints.add(new Pos2D(17, 8));
        checkpoints.add(new Pos2D(21, 8));
        checkpoints.add(new Pos2D(25, 8));
        checkpoints.add(new Pos2D(29, 8));
        checkpoints.add(new Pos2D(33, 8));
        checkpoints.add(new Pos2D(37, 8));
        checkpoints.add(new Pos2D(37, 1));

        return Scenario2D.generateScenarios(checkpoints, world, vehicle);

    }
    
    public static List<Scenario2D> spiralCheckpoints(Vehicle vehicle, World2D world){
        List<Pos2D> checkpoints = new ArrayList<Pos2D>();
        checkpoints.add(new Pos2D(15, 15));
        checkpoints.add(new Pos2D(15, 10));
        checkpoints.add(new Pos2D(10, 15));
        checkpoints.add(new Pos2D(16, 20));
        checkpoints.add(new Pos2D(23, 13));
        checkpoints.add(new Pos2D(16, 6));
        checkpoints.add(new Pos2D(6, 15));
        checkpoints.add(new Pos2D(17, 25));
        checkpoints.add(new Pos2D(28, 25));

        return Scenario2D.generateScenarios(checkpoints, world, vehicle);
        

    }

    public static void main(String[] args) {
        
        long startTime = System.currentTimeMillis();

        
        Vehicle vehicle = new Vehicle(3, 5, 0.5);        
//        World2D world = generateBenchmarkWorld();
//        List<Scenario2D> scenarios = benchmarkCheckpoints(vehicle, world);
        
        World2D world = generateSpiralWorld();
        List<Scenario2D> scenarios = spiralCheckpoints(vehicle, world);
        
        List<Solution> solutions = new ArrayList<Solution>();
            Pos2D lastSpeed = new Pos2D(0, 0);
            Pos2D lastPos = scenarios.get(0).startPos;
            int runNum = 0;
            for(Scenario2D scen: scenarios){
                try {
                scen.startVel = lastSpeed;
                scen.startPos = lastPos;
                Solution sol = solve(scen);
                lastSpeed = sol.vel[sol.score];
                lastPos = sol.pos[sol.score];
                solutions.add(sol);
                } catch (Exception e) {
                    e.printStackTrace();
                    Solution empty = new Solution(0, 0);
                    empty.highlightPoints.add(scen.startPos);
                    empty.highlightPoints.add(scen.goal);
                    solutions.add(empty);
                } finally{
                    System.out.println("RUN " + String.valueOf(runNum));
                    runNum++;
                }
            }

        
        long endTime   = System.currentTimeMillis();
        double totalTime = endTime - startTime;
        totalTime /= 1000;
        Solution result = Solution.combine(solutions);
        ResultWindow test = new ResultWindow(result, world, vehicle, totalTime);
        test.setVisible(true);
        
    }
    
    private static Solution solve(Scenario2D scen) throws Exception{
        
        CPLEXSolver solver = new CPLEXSolver(scen);
        solver.generateConstraints();
        solver.solve();
        Solution result = null;
        try {
            result = solver.getResults();
            return result;
        } catch (IloException e) {
            e.printStackTrace();
            throw new Exception();
        } finally{
            solver.end();
        }
        
    }

}
