package test;

import static org.junit.Assert.fail;

import java.util.function.Supplier;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import pathplanner.PathPlanner;
import pathplanner.PathPlannerFactory;
import pathplanner.PlannerResult;
import pathplanner.ScenarioFactory;
import pathplanner.common.Scenario;
import pathplanner.common.Vehicle;


public class AgilityTest extends ParentTest{
    public static final int RUNS = 5;

    // 3 cases: more less standard acc
    // 3 cases: more less standard max speed
    // 3 scenarios: benchmark, leuven sf
    @BeforeClass
    public static void setUpBeforeClass() throws Exception {
        
    }

    @AfterClass
    public static void tearDownAfterClass() throws Exception {}

    @Before
    public void setUp() throws Exception {}

    @After
    public void tearDown() throws Exception {}
    
    // BENCHMARK

    @Test
    public void benchmarkLowAccLowSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(1, Double.NaN, 2, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK LOW ACC LOW SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkMedAccLowSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 2, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK MED ACC LOW SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkHighAccLowSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(6, Double.NaN, 2, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK HIGH ACC LOW SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkLowAccMedSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(1, Double.NaN, 4, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK LOW ACC MED SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkMedAccMedSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 4, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK MED ACC MED SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkHighAccMedSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(6, Double.NaN, 4, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK HIGH ACC MED SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkLowAccHighSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(1, Double.NaN, 8, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK LOW ACC HIGH SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkMedAccHighSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 8, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK MED ACC HIGH SPEED", RUNS, func)) fail();
    }
    @Test
    public void benchmarkHighAccHighSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.benchmarkLarge();
            Vehicle vehicle = new Vehicle(6, Double.NaN, 8, 0.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("BENCHMARK HIGH ACC HIGH SPEED", RUNS, func)) fail();
    }
    
    // LEUVEN
    
    @Test
    public void leuvenLowAccLowSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 5, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN LOW ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenMedAccLowSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 5, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN MED ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenHighAccLowSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 5, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN HIGH ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenLowAccMedSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 15, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN LOW ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenMedAccMedSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN MED ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenHighAccMedSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 15, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN HIGH ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenLowAccHighSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 30, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN LOW ACC HIGH SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenMedAccHighSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 30, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN MED ACC HIGH SPEED", RUNS, func)) fail();
    }    
    @Test
    public void leuvenHighAccHighSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.leuvenSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 30, 1);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("LEUVEN HIGH ACC HIGH SPEED", RUNS, func)) fail();
    }
    
    
    // SAN FRANCISCO
    
    @Test
    public void sanFranciscoLowAccLowSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 5, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF LOW ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoMedAccLowSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 5, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF MED ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoHighAccLowSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 5, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF HIGH ACC LOW SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoLowAccMedSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 15, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF LOW ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoMedAccMedSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 15, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF MED ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoHighAccMedSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 15, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF HIGH ACC MED SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoLowAccHighSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(3, Double.NaN, 30, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF LOW ACC HIGH SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoMedAccHighSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(10, Double.NaN, 30, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF MED ACC HIGH SPEED", RUNS, func)) fail();
    }    
    @Test
    public void sanFranciscoHighAccHighSpeed(){
        Supplier<PlannerResult> func = ()->{
            ScenarioFactory fact = Scenarios.sanFranciscoSmall();
            Vehicle vehicle = new Vehicle(20, Double.NaN, 30, 2.5);
            fact.vehicle = vehicle;
            return solve(()->fact);};
        if(!measurePerformance("SF HIGH ACC HIGH SPEED", RUNS, func)) fail();
    }
    
    private PlannerResult solve(Supplier<ScenarioFactory> factProvider){
        Scenario scenario = factProvider.get().build();
        PathPlannerFactory plannerFact = new PathPlannerFactory();
        PathPlanner planner = plannerFact.build(scenario);
        PlannerResult result = planner.solve();
        return result;
    }

}
