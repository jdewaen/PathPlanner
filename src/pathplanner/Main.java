package pathplanner;

import pathplanner.common.Scenario;
import pathplanner.common.Vehicle;
import pathplanner.milpplanner.CPLEXSolverConfigFactory;
import pathplanner.preprocessor.boundssolver.BoundsSolverConfigFactory;
import pathplanner.preprocessor.cornerheuristic.ThetaStarConfigFactory;
import pathplanner.preprocessor.segments.SegmentGeneratorConfigFactory;
import pathplanner.ui.ResultWindow;
import test.Scenarios;

public class Main {	    
    public static void main(String[] args) {
            ScenarioFactory scenFact = Scenarios.benchmarkSmall();
//            Vehicle vehicle = new Vehicle(3, Double.NaN, 30, 2.5);
//            scenFact.vehicle = vehicle;
            Scenario scenario = scenFact.build();
            
            ThetaStarConfigFactory cornerConfigFact = new ThetaStarConfigFactory();
            cornerConfigFact.verbose = true;
            
            SegmentGeneratorConfigFactory segmentConfigFact = new SegmentGeneratorConfigFactory();
            segmentConfigFact.verbose = true;
            
            BoundsSolverConfigFactory boundsConfigFact = new BoundsSolverConfigFactory();
            boundsConfigFact.verbose = true;
//            boundsConfigFact.convexGrowMultiplier = 1.02;
            
            CPLEXSolverConfigFactory solverConfigFact = new CPLEXSolverConfigFactory();
            solverConfigFact.verbose = true;
            solverConfigFact.timeLimit = Double.NaN;
            solverConfigFact.preventCornerCutting = false;
//            solverConfigFact.fps = 2;
            
            PathPlannerFactory fact = new PathPlannerFactory();
            fact.cornerConfig = cornerConfigFact.build();
            fact.segmentConfig = segmentConfigFact.build();
            fact.boundsConfig = boundsConfigFact.build();
            fact.cplexConfig = solverConfigFact.build();
            fact.verbose = true;
            
//            PathPlanner planner = fact.build(scenario);
            NaivePathPlanner planner = new NaivePathPlanner(solverConfigFact.build(), scenario, 27);
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
