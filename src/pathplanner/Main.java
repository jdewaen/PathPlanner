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
import pathplanner.preprocessor.boundssolver.BoundsSolverConfigFactory;
import pathplanner.preprocessor.cornerheuristic.CornerHeuristicConfig;
import pathplanner.preprocessor.cornerheuristic.ThetaStarConfigFactory;
import pathplanner.preprocessor.segments.SegmentGeneratorConfig;
import pathplanner.preprocessor.segments.SegmentGeneratorConfigFactory;
import pathplanner.ui.ResultWindow;
import test.Scenarios;

public class Main {	    
    public static void main(String[] args) {
            ScenarioFactory scenFact = Scenarios.benchmarkLarge();
        
            Scenario scenario = scenFact.build();
            
            ThetaStarConfigFactory cornerConfigFact = new ThetaStarConfigFactory();
            cornerConfigFact.verbose = true;
            
            SegmentGeneratorConfigFactory segmentConfigFact = new SegmentGeneratorConfigFactory();
            segmentConfigFact.verbose = true;
            
            BoundsSolverConfigFactory boundsConfigFact = new BoundsSolverConfigFactory();
            boundsConfigFact.verbose = true;
            
            CPLEXSolverConfigFactory solverConfigFact = new CPLEXSolverConfigFactory();
            solverConfigFact.verbose = true;
            
            PathPlannerFactory fact = new PathPlannerFactory();
            fact.cornerConfig = cornerConfigFact.build();
            fact.segmentConfig = segmentConfigFact.build();
            fact.boundsConfig = boundsConfigFact.build();
            fact.cplexConfig = solverConfigFact.build();
            fact.verbose = true;
            
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
