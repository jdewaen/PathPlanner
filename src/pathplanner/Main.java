package pathplanner;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

import pathplanner.common.Scenario;
import pathplanner.common.Solution;
import pathplanner.milpplanner.CPLEXSolverConfigFactory;
import pathplanner.preprocessor.PathSegment;
import pathplanner.ui.ResultWindow;
import test.Scenarios;

public class Main {	    
    public static void main(String[] args) {
            Scenario scenario = Scenarios.SanFranciscoSmall();
            
            PathPlannerFactory fact = new PathPlannerFactory();
            PathPlanner planner = fact.build(scenario);
//            NaivePathPlanner planner = new NaivePathPlanner(CPLEXSolverConfigFactory.DEFAULT, scenario, 15);
            PlannerResult result = planner.solve();
            
            System.out.println(result.stats);

            ResultWindow test = new ResultWindow(scenario, result);
            test.setVisible(true);

    }
    
    
//    public static void saveSolution(Solution sol, String filename) throws IOException{
//        FileOutputStream fout = new FileOutputStream(filename);
//        ObjectOutputStream oos = new ObjectOutputStream(fout);
//        oos.writeObject(sol);
//        oos.close();
//    }
//    
//    public static Solution loadSolution(String filename) throws ClassNotFoundException, IOException{
//        FileInputStream fout = new FileInputStream(filename);
//        ObjectInputStream oos = new ObjectInputStream(fout);
//        Solution result = (Solution) oos.readObject();
//        oos.close();
//        return result;
//    }
   

}
