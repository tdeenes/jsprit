/*******************************************************************************
 * Copyright (C) 2014  Stefan Schroeder
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/
package com.graphhopper.jsprit.custom;

import com.graphhopper.jsprit.core.algorithm.box.GreedySchrimpfFactory;
import com.graphhopper.jsprit.core.algorithm.state.StateManager;
import com.graphhopper.jsprit.core.algorithm.state.InternalStates;
import com.graphhopper.jsprit.core.algorithm.VehicleRoutingAlgorithm;
import com.graphhopper.jsprit.core.algorithm.selector.SelectBest;
import com.graphhopper.jsprit.core.algorithm.termination.IterationWithoutImprovementTermination;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem;
import com.graphhopper.jsprit.core.problem.job.Job;
import com.graphhopper.jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import com.graphhopper.jsprit.core.reporting.SolutionPrinter;
import jsprit.util.Custom;
import java.util.Collection;
import com.graphhopper.jsprit.analysis.toolbox.StopWatch;
import com.graphhopper.jsprit.core.algorithm.box.Jsprit;
import com.graphhopper.jsprit.core.algorithm.listener.VehicleRoutingAlgorithmListeners;
import com.graphhopper.jsprit.core.problem.constraint.ConstraintManager;
import com.graphhopper.jsprit.io.problem.VrpXMLWriter;
import com.graphhopper.jsprit.core.problem.solution.route.VehicleRoute;
import com.graphhopper.jsprit.core.problem.solution.SolutionCostCalculator;

/*
* author: Denes Toth - solve the vehicle routing problem
*/

public class TobaccoOptimization {

    public static void main(String[] args) {
        /*
         * some preparation - create output folder
         */
        Custom.createOutputFolder();

        /*
        * check args
        * (iter = the number of iterations [default: 1000, suggested >5000])
        */
        int max_iter;
        final double scale_unassigned_cost;
        if (args.length > 0) {
            max_iter = Integer.parseInt(args[0]);
        } else {
            max_iter = 1000;
        }
        if (args.length > 1) {
            scale_unassigned_cost = Integer.parseInt(args[1]);
        } else {
            scale_unassigned_cost = 0.2;
        }

        /*
         * Build the problem.
         *
         * But define a problem-builder first.
         */
        VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();

        /*
         * Use the custom reader
         */
        new CustomReader3(vrpBuilder).read("input/temp");

        /*
         * Finally, the problem can be built.
         */
        final VehicleRoutingProblem vrp = vrpBuilder.build();

         /*
         * Define the required vehicle-routing algorithms to solve the above problem.
         *
         * The algorithm can be defined and configured in an xml-file.
         */
        final StateManager stateManager = new StateManager(vrp);
        ConstraintManager constraintManager = new ConstraintManager(vrp, stateManager);

        /*
         * Set the objective function
         */
        SolutionCostCalculator objectiveFunction = new SolutionCostCalculator() {

            @Override
            public double getCosts(VehicleRoutingProblemSolution solution) {
                double c = 0.0;
                for (VehicleRoute r : solution.getRoutes()) {
                    c += stateManager.getRouteState(r, InternalStates.COSTS, Double.class);
                    c += r.getVehicle().getType().getVehicleCostParams().fix;
                }

                // The cost of unassigned jobs (i.e., shops not visited)
                for(Job j : solution.getUnassignedJobs()){
                    c += c * scale_unassigned_cost * (11 - j.getPriority());
                }

                return c;
            }
        };

        // build the algorithm
        VehicleRoutingAlgorithm vra = Jsprit.Builder.newInstance(vrp)
            .setProperty(Jsprit.Parameter.THREADS, "3")
            .setStateAndConstraintManager(stateManager, constraintManager)
            .setObjectiveFunction(objectiveFunction)
            .buildAlgorithm();
        vra.getAlgorithmListeners().addListener(new StopWatch(), VehicleRoutingAlgorithmListeners.Priority.HIGH);
        vra.setMaxIterations(max_iter);
        vra.setPrematureAlgorithmTermination(new IterationWithoutImprovementTermination(100));

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
