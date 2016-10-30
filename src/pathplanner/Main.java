package pathplanner;

import ilog.concert.IloException;
import pathplanner.common.*;
import pathplanner.milpplanner.*;
import pathplanner.ui.ResultWindow;

public class Main {

    public static void main(String[] args) {
        

        World2D world = new World2D(new Pos2D(10, 10));
        world.addObstacle(new Obstacle2D(new Pos2D(2, 0), new Pos2D(3, 8)));
//        world.addObstacle(new Obstacle2D(new Pos2D(5, 2), new Pos2D(7, 10)));
        Vehicle vehicle = new Vehicle(0.2, 5);
        Scenario2D scen = new Scenario2D(world, vehicle, new Pos2D(1, 5), new Pos2D(9, 4), 10, 100);

        
        CPLEXSolver solver = new CPLEXSolver(scen);
        solver.generateConstraints();
        solver.solve();
        Solution result = null;
        try {
            result = solver.getResults();
        } catch (IloException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        ResultWindow test = new ResultWindow(result, scen);
        solver.end();
        test.setVisible(true);
        
    }

}
