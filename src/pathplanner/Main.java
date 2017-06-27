package pathplanner;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

import pathplanner.common.Scenario;
import pathplanner.common.Solution;
import pathplanner.milpplanner.CPLEXSolverConfigFactory;
import pathplanner.preprocessor.boundssolver.BoundsSolverConfigFactory;
import pathplanner.preprocessor.cornerheuristic.ThetaStarConfigFactory;
import pathplanner.preprocessor.segments.PathSegmentGeneratorConfigFactory;
import pathplanner.ui.ResultWindow;
import test.Scenarios;

public class Main {	    
    public static void main(String[] args) throws IOException, ClassNotFoundException {
        
      ScenarioFactory scenFact = Scenarios.benchmark2();
        
        boolean loopUntilFail = false;
        PlannerResult result;
        Scenario scenario;
        while(true){
        
//            scenFact.start = new Vector2D(1, 1);
//            scenFact.goal = new Vector2D(1, 5);
            scenario = scenFact.build();
            
            ThetaStarConfigFactory cornerConfigFact = new ThetaStarConfigFactory();
//            cornerConfigFact.gridSize = 1;
//            cornerConfigFact.verbose = true;
            
            PathSegmentGeneratorConfigFactory segmentConfigFact = new PathSegmentGeneratorConfigFactory();
//            segmentConfigFact.verbose = true;
//            segmentConfigFact.maxSegmentTime = 2.5;
            
//            segmentConfigFact.approachMargin = 1;
            
            BoundsSolverConfigFactory boundsConfigFact = new BoundsSolverConfigFactory();
//            boundsConfigFact.verbose = true;
//            boundsConfigFact.convexGrowMultiplier = 1.02;
            
            CPLEXSolverConfigFactory solverConfigFact = new CPLEXSolverConfigFactory();
            solverConfigFact.verbose = true;
//            solverConfigFact.absMIPgap = Double.NaN;
//            solverConfigFact.useFinishLine = false;
//            solverConfigFact.timeLimit = 60*10;
//            solverConfigFact.preventCornerCutting = false;
//            solverConfigFact.ignoreVehicleSize = true;
//            solverConfigFact.useIndicatorConstraints = false;
//            solverConfigFact.fps = 2;
            
//            solverConfigFact.positionTolerance = 1.1;
//            solverConfigFact.timeLimitMultiplier = 3;
//            
            PathPlannerFactory fact = new PathPlannerFactory();
            fact.cornerConfig = cornerConfigFact.build();
            fact.segmentConfig = segmentConfigFact.build();
            fact.boundsConfig = boundsConfigFact.build();
            fact.cplexConfig = solverConfigFact.build();
            fact.verbose = true;
//            fact.useStopPoints = false;
//            fact.overlap = 3;
            
            PathPlanner planner = fact.build(scenario);
//            NaivePathPlanner planner = new NaivePathPlanner(solverConfigFact.build(), scenario, 30);
            result = planner.solve();
            
            if(loopUntilFail){
                if(result.failed) break;
                System.out.println("SUCCESS: REPEATING...");
            }else{
                break;
            }
            
        }
            System.out.println(result.stats);

        
//            PlannerResult result = loadSolution("save1.dat");
            ResultWindow test = new ResultWindow(result);
            test.setVisible(true);
//            saveSolution(result, "save1.dat");

    }
    
    
    public static void saveSolution(PlannerResult result, String filename) throws IOException{
        FileOutputStream fout = new FileOutputStream(filename);
        ObjectOutputStream oos = new ObjectOutputStream(fout);
        oos.writeObject(result);
        oos.close();
    }
    
    public static PlannerResult loadSolution(String filename) throws ClassNotFoundException, IOException{
        FileInputStream fout = new FileInputStream(filename);
        ObjectInputStream oos = new ObjectInputStream(fout);
        PlannerResult result = (PlannerResult) oos.readObject();
        oos.close();
        return result;
    }
   

}
