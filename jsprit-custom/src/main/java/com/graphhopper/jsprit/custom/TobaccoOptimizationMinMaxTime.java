package com.graphhopper.jsprit.custom;

import java.io.IOException;
import java.util.Collection;

import com.graphhopper.jsprit.core.algorithm.VehicleRoutingAlgorithm;
import com.graphhopper.jsprit.core.algorithm.recreate.VariableTransportCostCalculator;
import com.graphhopper.jsprit.core.algorithm.state.InternalStates;
import com.graphhopper.jsprit.core.algorithm.state.StateManager;
import com.graphhopper.jsprit.core.algorithm.state.StateUpdater;
import com.graphhopper.jsprit.core.problem.Location;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem;
import com.graphhopper.jsprit.core.problem.constraint.ConstraintManager;
import com.graphhopper.jsprit.core.problem.constraint.SoftActivityConstraint;
import com.graphhopper.jsprit.core.problem.cost.VehicleRoutingTransportCosts;
import com.graphhopper.jsprit.core.problem.job.Job;
import com.graphhopper.jsprit.core.problem.misc.JobInsertionContext;
import com.graphhopper.jsprit.core.problem.solution.SolutionCostCalculator;
import com.graphhopper.jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import com.graphhopper.jsprit.core.problem.solution.route.VehicleRoute;
import com.graphhopper.jsprit.core.problem.solution.route.activity.ActivityVisitor;
import com.graphhopper.jsprit.core.problem.solution.route.activity.TourActivity;

import com.graphhopper.jsprit.core.algorithm.state.StateId;

import com.graphhopper.jsprit.core.reporting.SolutionPrinter;
import com.graphhopper.jsprit.core.util.ActivityTimeTracker;
import com.graphhopper.jsprit.core.algorithm.selector.SelectBest;
import jsprit.util.Custom;
import com.graphhopper.jsprit.analysis.toolbox.StopWatch;
import com.graphhopper.jsprit.core.algorithm.box.Jsprit;
import com.graphhopper.jsprit.core.algorithm.listener.VehicleRoutingAlgorithmListeners;
import com.graphhopper.jsprit.io.problem.VrpXMLWriter;

public class TobaccoOptimizationMinMaxTime {

    /*
     * This updates the state "max-work-hours" which is introduced below. Once either the insertion procedure starts or a job has
     * been inserted, UpdateMaxWorkHours is called for the route that has been changed.
     *
     * It must not only be an ActivityVisitor which indicates that the update procedure starts at the beginning of route all the way to end
     * (in contrary to the ReverseActivityVisitor) but also be a StateUpdater which is just a marker to register it in the StateManager.
     *
     * You do not need to declare this as static inner class. You can just choose your preferred approach. However, be aware
     * that this requires the stateName "max-work-hours" you define below. If you choose to define this as class in a new file,
     * you might define "max-work-hours" as static id in another file, to make sure you do not have type errors etc..
     */
    static class UpdateMaxWorkHours implements ActivityVisitor, StateUpdater {

        private StateManager stateManager;

        private ActivityTimeTracker timeTracker;

        private VehicleRoute route;

        private VehicleRoutingProblem vrp;

        public UpdateMaxWorkHours(StateManager stateManager, VehicleRoutingProblem vrp) {
            super();
            this.stateManager = stateManager;
            this.vrp = vrp;
            this.timeTracker = new ActivityTimeTracker(vrp.getTransportCosts(), vrp.getActivityCosts());
        }

        @Override
        public void begin(VehicleRoute route) {
            this.route = route;
            timeTracker.begin(route);
        }

        @Override
        public void visit(TourActivity activity) {
            timeTracker.visit(activity);
        }

        @Override
        public void finish() {
            timeTracker.finish();
            int last = this.route.getActivities().size() - 1;
            if (last >= 0) {
                Location last_loc = this.route.getActivities().get(last).getLocation();
                Location depot_loc = this.route.getEnd().getLocation();
                double newRouteWorkHours = this.vrp.getTransportCosts()
                    .getTransportTime(last_loc, depot_loc, timeTracker.getActEndTime(), this.route.getDriver(), this.route.getVehicle());
                if(stateManager.getProblemState(stateManager.createStateId("max-work-hours"), Double.class) == null) {
                    stateManager.putProblemState(stateManager.createStateId("max-work-hours"), Double.class, newRouteWorkHours);
                } else {
                    double currentMaxWorkHours = stateManager.getProblemState(stateManager.createStateId("max-work-hours"), Double.class);
                    if (newRouteWorkHours > currentMaxWorkHours) {
                        stateManager.putProblemState(stateManager.createStateId("max-work-hours"), Double.class, newRouteWorkHours);
                    }
                }
            }
        }

    }

    public static void main(String[] args) throws IOException {

        /*
        * some preparation - create output folder
        */
        Custom.createOutputFolder();

        /*
        * check args
        * (iter = the number of iterations)
        */
        int max_iter;
        if (args.length > 0) {
            max_iter = Integer.parseInt(args[0]);
        } else {
            max_iter = 100;
        }

        /*
        * Problem builder
         */
        VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();

        /*
         * Use the custom reader to set up the problem (import shops, vehicles, distances)
         */
        new CustomReader3(vrpBuilder).read("input/temp");

        /*
         * Build the problem.
         */
        final VehicleRoutingProblem vrp = vrpBuilder.build();

        // get StateManager
        final StateManager stateManager = new StateManager(vrp);
        //introduce a new state called "max-work-hours"
        StateId max_transport_time_state = stateManager.createStateId("max-work-hours");
        //add a default-state for "max-work-hours"
        stateManager.putProblemState(max_transport_time_state, Double.class, 0.);
        //
        stateManager.addStateUpdater(new UpdateMaxWorkHours(stateManager,vrp));

        /*
		 * Your custom objective function that min max transport times. Additionally you can try to consider overall transport times
		 * in your objective as well. Thus you minimize max transport times first, and second, you minimize overall transport time.
		 *
		 * If you choose to consider overall transport times, makes sure you scale it appropriately.
		 *
		 * DENES: some changes - consider a broader set of costs
		 */
        SolutionCostCalculator objectiveFunction = new SolutionCostCalculator() {

            //private double scalingParameter_v = 10; // it seems there is no need to include this
            private double scalingParameter_t = 1.;
            private double scalingParameter_u = 0.2;

            @Override
            public double getCosts(VehicleRoutingProblemSolution solution) {
                double nr_of_routes = solution.getRoutes().size();
                double nr_of_orig_vehicles = vrp.getVehicles().size();
                double driver_work_hours;
                double max_work_hours = 0.;
                double sum_work_hours = 0.;
                double c = 0.0;
                for(VehicleRoute r : solution.getRoutes()){
                    c += stateManager.getRouteState(r, InternalStates.COSTS, Double.class);
                    c += r.getVehicle().getType().getVehicleCostParams().fix;
                    driver_work_hours = r.getEnd().getArrTime() - r.getDepartureTime();
                    sum_work_hours += driver_work_hours;
                    if(driver_work_hours > max_work_hours){
                        max_work_hours = driver_work_hours;
                    }
                }
                double base_cost = c;
                //c += base_cost * scalingParameter_v * (nr_of_orig_vehicles - nr_of_routes)/nr_of_orig_vehicles;
                c += base_cost * scalingParameter_t * (max_work_hours/sum_work_hours*nr_of_orig_vehicles - 1);

                // The cost of unassigned jobs (i.e., shops not visited)
                for(Job j : solution.getUnassignedJobs()){
                    c += base_cost * scalingParameter_u * (11 - j.getPriority());
                }

                return c;
                //return max_work_hours; // this would be the most extreme
            }
        };

		/*
		 * The insertion heuristics is controlled with your constraints
		 */
        ConstraintManager constraintManager = new ConstraintManager(vrp, stateManager);
        // soft constraint that calculates additional transport costs when inserting a job(activity) at specified position
        constraintManager.addConstraint(new VariableTransportCostCalculator(vrp.getTransportCosts(), vrp.getActivityCosts()));
		/*
		 *  soft constraint that penalizes a shift of max working hours, i.e. once the insertion heuristic
		 *  tries to insert a jobActivity at position which results in a shift of max-work-hours, it is penalyzed with
		 *  penaltyForEachTimeUnitAboveCurrentMaxTime
		 *
		 */
        SoftActivityConstraint penalyzeShiftOfMaxTransportTime = new SoftActivityConstraint() {
            private final VehicleRoutingTransportCosts routingCosts = vrp.getTransportCosts();

            private final double penaltyForEachTimeUnitAboveCurrentMaxTime = 3.;

            @Override
            public double getCosts(JobInsertionContext iFacts, TourActivity prevAct, TourActivity newAct, TourActivity nextAct, double depTimeAtPrevAct) {
                /*
				 * determines maximum of all routes' transport times, which is here basically a state that can be fetched via the stateManager
				 */
                double maxTime = stateManager.getProblemState(stateManager.createStateId("max-work-hours"), Double.class);
				/*
				 * determines additional time of route when inserting newAct between prevAct and nextAct
				 *
				 */
                double tp_time_prevAct_newAct = routingCosts.getTransportTime(prevAct.getLocation(), newAct.getLocation(), depTimeAtPrevAct, iFacts.getNewDriver(), iFacts.getNewVehicle());
                double newAct_arrTime = depTimeAtPrevAct + tp_time_prevAct_newAct;
                double newAct_endTime = newAct_arrTime + newAct.getOperationTime();
                double tp_time_newAct_nextAct = routingCosts.getTransportTime(newAct.getLocation(), nextAct.getLocation(), newAct_endTime, iFacts.getNewDriver(), iFacts.getNewVehicle());
                double nextAct_arrTime = newAct_endTime + tp_time_newAct_nextAct;
                double oldTime;
                //if(iFacts.getRoute().isEmpty()){
                //    oldTime = (nextAct.getArrTime() - depTimeAtPrevAct);
                //}
                //else{
                    oldTime = (nextAct.getArrTime() - iFacts.getRoute().getDepartureTime());
                //}
                double additionalTime = (nextAct_arrTime - iFacts.getNewDepTime()) - oldTime;
                double newTime = iFacts.getRoute().getEnd().getArrTime() - iFacts.getRoute().getDepartureTime() + additionalTime;
                //System.out.println("Max: " + maxTime + ", additional: " + additionalTime);
                return penaltyForEachTimeUnitAboveCurrentMaxTime*Math.max(0,newTime-maxTime);
            }
        };
        constraintManager.addConstraint(penalyzeShiftOfMaxTransportTime);

        // build the algorithm
        Jsprit.Builder builder = Jsprit.Builder.newInstance(vrp)
            .setProperty(Jsprit.Parameter.THREADS, "3")
            //.setProperty(Jsprit.Strategy.CLUSTER_BEST, "0.5")
            .setStateAndConstraintManager(stateManager, constraintManager)
            .setObjectiveFunction(objectiveFunction);
        VehicleRoutingAlgorithm vra = builder.buildAlgorithm();

        // initial solution


        //
        vra.getAlgorithmListeners().addListener(new StopWatch(), VehicleRoutingAlgorithmListeners.Priority.HIGH);
        vra.setMaxIterations(max_iter);

        /*
         * Solve the problem.
         *
         *
         */
        Collection<VehicleRoutingProblemSolution> solutions = vra.searchSolutions();
        new VrpXMLWriter(vrp, solutions).write("output/temp_solution.xml");

        /*
         * Retrieve best solution.
         */
        VehicleRoutingProblemSolution solution = new SelectBest().selectSolution(solutions);

        /*
         * print solution to console
         */
        SolutionPrinter.print(vrp, solution, SolutionPrinter.Print.VERBOSE);

        for (VehicleRoute route : solution.getRoutes()) {
            System.out.println("------");
            System.out.println("vehicleId: " + route.getVehicle().getId());
            System.out.println("vehicleCapacity: " + route.getVehicle().getType().getCapacityDimensions());
        }

    }

}
