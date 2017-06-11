package pathplanner;


import ilog.concert.IloException;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.NoSuchElementException;

import pathplanner.common.Obstacle2DB;
import pathplanner.common.Pos2D;
import pathplanner.common.Scenario;
import pathplanner.common.ScenarioSegment;
import pathplanner.common.ScenarioSegmentFactory;
import pathplanner.common.Solution;
import pathplanner.common.StatisticsTracker;
import pathplanner.milpplanner.CPLEXSolver;
import pathplanner.milpplanner.CPLEXSolverConfig;
import pathplanner.milpplanner.CPLEXSolverConfigFactory;
import pathplanner.milpplanner.ObstacleConstraint;
import pathplanner.milpplanner.PolygonConstraint;
import pathplanner.preprocessor.CornerEvent;
import pathplanner.preprocessor.PathNode;
import pathplanner.preprocessor.PathSegment;
import pathplanner.preprocessor.boundssolver.BoundsSolverConfig;
import pathplanner.preprocessor.cornerheuristic.CornerHeuristic;
import pathplanner.preprocessor.segments.PathSegmentGenerator;


/**
 * The "core" class of the algorithm which controls the flow during the different steps It contains the Scenario to be solved, as well as all configuration
 *
 */
public class PathPlanner {

    public final CornerHeuristic      cornerHeuristic;
    public final PathSegmentGenerator pathSegmentGenerator;
    public final BoundsSolverConfig   boundsConfig;
    public final CPLEXSolverConfig    cplexConfig;
    public final Scenario             scenario;
    public final StatisticsTracker    stats = new StatisticsTracker();
    public final boolean              enableBacktracking;
    public final boolean              useStopPoints;
    public final boolean              verbose;
    public final int                  overlap;

    public PathPlanner(CornerHeuristic cornerHeuristic,
            PathSegmentGenerator pathSegmentGenerator,
            BoundsSolverConfig boundsConfig,
            CPLEXSolverConfig cplexConfig,
            Scenario scenario,
            boolean enableBacktracking,
            boolean useStopPoints,
            int overlap,
            boolean verbose) {
        this.cornerHeuristic = cornerHeuristic;
        this.pathSegmentGenerator = pathSegmentGenerator;
        this.boundsConfig = boundsConfig;
        this.cplexConfig = cplexConfig;
        this.scenario = scenario;
        this.enableBacktracking = enableBacktracking;
        this.useStopPoints = useStopPoints;
        this.overlap = overlap;
        this.verbose = verbose;
    }

    /**
     * Solves the trajectory planning problem
     * 
     * @return
     */
    public PlannerResult solve() {
        long totalTimer = stats.startTimer();

        // Find the initial path
        long timer = stats.startTimer();
        PathNode prePath = cornerHeuristic.solve();
        stats.prePathTime = stats.stopTimer(timer);

        // Find the corners in the path
        timer = stats.startTimer();
        List<CornerEvent> corners = cornerHeuristic.generateEvents(prePath);
        stats.cornerTime = stats.stopTimer(timer);

        // Generate the PathSegments
        timer = stats.startTimer();
        List<PathSegment> pathSegments = pathSegmentGenerator.generateFromPath(
                prePath, corners);
        stats.pathSegmentTime = stats.stopTimer(timer);

        // Generate the factories for the ScenarioSegments
        timer = stats.startTimer();
        List<ScenarioSegmentFactory> scenariosegmentFacts = generateScenarioSegments(pathSegments);
        stats.scenSegmentTime = stats.stopTimer(timer);

        Solution sol;
        boolean fail = false;
        List<ScenarioSegment> scenSegments = new ArrayList<ScenarioSegment>();
        try {
            // Attempt to solve the segments
            sol = solveSegments(scenariosegmentFacts, scenSegments);
        } catch (PlanException e) {
            println(e.parent.getMessage());
            sol = e.sol;
            fail = true;
        }

        stats.totalTime = stats.stopTimer(totalTimer);
        stats.score = ((double) sol.score) / cplexConfig.fps;
        PlannerResult result = new PlannerResult(this, sol, prePath, corners,
                pathSegments, scenSegments, stats, fail);
        return result;
    }

    /**
     * Generates the ScenarioSegments for the current Scenario based on the PathSegments. The PathSegments are just a few nodes on the initial path. This method turns those into full-fledged
     * ScenarioSegments which contain all the information to generate the MILP trajectory planning problem
     * 
     * @param pathSegments
     *            The segments of the initial path which should be used to construct ScenarioSegments
     * @return
     */
    public List<ScenarioSegmentFactory> generateScenarioSegments(
            List<PathSegment> pathSegments) {
        List<ScenarioSegmentFactory> segments = new ArrayList<ScenarioSegmentFactory>();

        for (int i = 0; i < pathSegments.size(); i++) {
            PathSegment current = pathSegments.get(i);
            ScenarioSegmentFactory segment;

            // Estimate the time needed for the ScenarioSegment
            int time = (int) current.estimateTimeNeeded(scenario.vehicle,
                    cplexConfig.timeLimitMultiplier) + overlap;
            println("RUN " + String.valueOf(i));
            println("TIME GUESS: " + String.valueOf(time));

            // Build the factory for the ScenarioSegment
            segment = new ScenarioSegmentFactory(scenario, current.end.pos,
                    cplexConfig.fps, cplexConfig.positionTolerance, time,
                    current);

            // Set a preliminary starting position. Unless this is the first segment, this will be overwritten later
            segment.startPos = current.start.pos;

            // If the segment is the last one
            if (i == pathSegments.size() - 1) {
                segment.isFinal = true;

                // Update the position tolerance around the goal, this is probably smaller than default
                segment.positionTolerance = cplexConfig.positionToleranceFinal;

                // Update the goal velocity to match the goal velocity for the Scenario
                segment.goalVel = scenario.goalVel;
            }

            // If the maximum goal velocity is set in the PathSegment, add that to the ScenarioSegment
            if (!Double.isNaN(current.goalVel)) {
                println("Limiting speed to " + String.valueOf(current.goalVel));
                segment.maxGoalVel = current.goalVel;
            }

            segments.add(segment);
        }

        return segments;

    }

    /**
     * Solves all ScenarioSegments and returns the solution.
     * 
     * @param segmentFacts
     *            The factories for the ScenarioSegments to be solved
     * @param scenarioSegments
     *            The list which will be populated with the ScenarioSegments as they are solved. This list should be empty at first
     * @return
     * @throws PlanException
     *             When at least one segment could not be solved
     */
    public Solution solveSegments(List<ScenarioSegmentFactory> segmentFacts,
            List<ScenarioSegment> scenarioSegments) throws PlanException {
        LinkedList<Solution> solutions = new LinkedList<Solution>();

        for (int i = 0; i < segmentFacts.size(); i++) {
            ScenarioSegment scen = null;
            ScenarioSegmentFactory scenFact = segmentFacts.get(i);
            
            // Try to get the final state of the last segment, otherwise use the start values
            try {
                Solution last = solutions.getLast();
                if (scenario.vehicle.hasMaxJerk()) scenFact.startAcc = last.acc[last.score
                        - overlap + 1];
                scenFact.startVel = last.vel[last.score - overlap + 1];
                scenFact.startPos = last.pos[last.score - overlap + 1];
            } catch (NoSuchElementException e) {
                scenFact.startAcc = new Pos2D(0, 0);
                scenFact.startVel = scenario.startVel;
                scenFact.startPos = scenario.startPos;
            }

            try {
                println("RUN " + String.valueOf(i) + " START");
                
                // scen = preSolve(scenFact);
                // System.out.println("PRE DONE");
                
                // Build the ScenarioSegment from the factory and solve it
                scen = scenFact.build();
                scenarioSegments.add(scen);
                Solution sol = solve(scen);

                addConstraintsToSol(scen, sol);
                solutions.addLast(sol);

            // If solving didn't work, construct an "empty" solution so the constraints can still be visualized
            } catch (Exception e) {
                Solution empty;
                if (solutions.isEmpty()) {
                    int time = 10;
                    int timesteps = cplexConfig.fps * time;
                    empty = new Solution(time, timesteps);
                    empty.highlightPoints.add(scen.startPos);
                    empty.highlightPoints.add(scen.goal);
                    for (int j = 0; j < timesteps; j++) {
                        empty.nosol[j] = true;
                        empty.time[j] = ((double) j) / cplexConfig.fps;
                        empty.pos[j] = scen.startPos;
                        empty.vel[j] = scen.startVel;
                    }
                    empty.score = 10;
                } else {
                    Solution last = solutions.getLast();
                    int timesteps = last.timeSteps - last.score + overlap - 1;
                    empty = new Solution((double) timesteps / cplexConfig.fps,
                            timesteps);
                    empty.highlightPoints.add(scen.startPos);
                    empty.highlightPoints.add(scen.goal);
                    for (int j = 0; j < timesteps; j++) {
                        empty.nosol[j] = true;
                        empty.time[j] = ((double) j) / cplexConfig.fps;
                        empty.pos[j] = last.pos[last.score - overlap + 1 + j];
                        empty.vel[j] = last.vel[last.score - overlap + 1 + j];
                    }
                    empty.score = timesteps - 1;
                }
                addConstraintsToSol(scen, empty);
                solutions.add(empty);
                throw new PlanException(Solution.combine(solutions, overlap), e);
            } finally {
                println("RUN " + String.valueOf(i) + " COMPLETED");
            }
        }

        Solution result = Solution.combine(solutions, overlap);

        return result;
    }

    /**
     * Adds the information about the preprocessing done on the Scenario to the Solution, so it can be displayed later
     * 
     * @param scen
     * @param sol
     */
    private void addConstraintsToSol(ScenarioSegment scen, Solution sol) {
        HashSet<Obstacle2DB> activeObs = new HashSet<Obstacle2DB>();

        for (ObstacleConstraint cons : scen.activeSet) {
            if (cons instanceof PolygonConstraint) {
                PolygonConstraint obs = (PolygonConstraint) cons;
                activeObs.add((Obstacle2DB) obs.region);
            }
        }

        for (int i = 0; i < sol.timeSteps; i++) {
            sol.activeArea.add(scen.activeRegion);
            sol.boundsDebugData.add(scen.boundsDebugData);
            sol.activeObstacles[i] = activeObs;
        }
    }

    /**
     * Solves a specific ScenarioSegment
     * @param segment
     * @return
     * @throws Exception
     */
    private Solution solve(ScenarioSegment segment)
            throws Exception {
        
        // First, select the obstacles to be modeled and build the convex safe area
        long geneticStart = stats.startTimer();
        segment.generateActiveSet(scenario, boundsConfig, useStopPoints);
        long geneticDuration = stats.stopTimer(geneticStart);
        
        // Set up the CPLEX solver
        long setupStart = stats.startTimer();
        CPLEXSolver solver = new CPLEXSolver(scenario, segment,
                cplexConfig);
        solver.generateConstraints();
        long setupDuration = stats.stopTimer(setupStart);
        
        // Solve the problem
        long solveStart = stats.startTimer();
        solver.solve();
        long solveDuration = stats.stopTimer(solveStart);
        Solution result = null;
        
        // Throw an exception if the problem could not be solved, otherwise return the Solution
        try {
            result = solver.getResults();
            stats.geneticTimes.add(geneticDuration);
            stats.setupTime.add(setupDuration);
            stats.solveTime.add(solveDuration);
            return result;
        } catch (IloException e) {
            throw e;
        } finally {
            solver.end();
        }

    }

    /**
     * This is the pre-solve step using a rough time step as described in Section 6.3.1 on page 78 of the thesis. This was merely a quick experiment and is by no means stable
     * 
     * @param fact
     *            The factory which produces the ScenarioSegment to be solved
     * @return The ScenarioSegment generated by fact with a very tight maximum time
     * @throws Exception
     */
    private ScenarioSegment preSolve(ScenarioSegmentFactory fact)
            throws Exception {
        fact.fps = 2;
        ScenarioSegment seg1 = fact.build();
        long geneticStart = stats.startTimer();
        seg1.generateActiveSet(scenario, boundsConfig, useStopPoints);
        long geneticDuration = stats.stopTimer(geneticStart);

        CPLEXSolver solver = new CPLEXSolver(scenario, seg1, cplexConfig);
        solver.generateConstraints();
        solver.solve();
        Solution result = null;
        try {
            result = solver.getResults();
            double maxTime = 1 + ((double) result.score) / 2;
            System.out.println("Maxtime changed from " + fact.maxTime + " to "
                    + maxTime);
            fact.fps = cplexConfig.fps;
            fact.maxTime = maxTime;
            ScenarioSegment seg2 = fact.build();
            stats.geneticTimes.add(geneticDuration);
            seg2.activeRegion = seg1.activeRegion;
            seg2.activeSet.addAll(seg1.activeSet);
            seg2.boundsDebugData = seg1.boundsDebugData;
            return seg2;

        } catch (IloException e) {
            throw e;
        } finally {
            solver.end();
        }

    }

    private void println(String text) {
        if (verbose) {
            System.out.println(text);
        }
    }

}
