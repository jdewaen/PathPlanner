package pathplanner;

import pathplanner.common.Pos2D;
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
//        ScenarioFactory scenFact = Scenarios.benchmarkSmall();
        
      ScenarioFactory scenFact = Scenarios.leuvenSmall();
//      ScenarioFactory scenFact = Scenarios.sanFranciscoSmall();
//      ScenarioFactory scenFact = Scenarios.leuvenLarge();
//      ScenarioFactory scenFact = Scenarios.cornerTooSharp();
//      ScenarioFactory scenFact = Scenarios.spiral();
//      Vehicle vehicle = new Vehicle(5, Double.NaN, 15, 0.5);
//      scenFact.vehicle = vehicle;
//      scenFact.start = new Pos2D(733.4728519937894, 918.250711980963);
//      scenFact.startVel = new Pos2D(-2.6734615382188207, 11.212111007758624).multiply(0.95);
//      scenFact.goal = new Pos2D(768.96, 945.05);
//      scenFact.goalVel = null;
        
        boolean loop = false;
        PlannerResult result;
        Scenario scenario;
        while(true){
        
//            scenFact.start = new Pos2D(1, 1);
//            scenFact.goal = new Pos2D(1, 5);
            scenario = scenFact.build();
            
            ThetaStarConfigFactory cornerConfigFact = new ThetaStarConfigFactory();
//            cornerConfigFact.gridSize = 1;
//            cornerConfigFact.verbose = true;
            
            SegmentGeneratorConfigFactory segmentConfigFact = new SegmentGeneratorConfigFactory();
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
            
            if(loop){
                if(result.failed) break;
                System.out.println("SUCCESS: REPEATING...");
            }else{
                break;
            }
            
        }
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
