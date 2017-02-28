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

import java.util.ArrayList;
import com.graphhopper.jsprit.core.algorithm.state.StateManager;
import com.graphhopper.jsprit.core.algorithm.state.InternalStates;
import com.graphhopper.jsprit.core.algorithm.VehicleRoutingAlgorithm;
import com.graphhopper.jsprit.core.algorithm.selector.SelectBest;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem;
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

    public static double mean(double[] v) {
        double tot = 0.0;
        for (int i = 0; i < v.length; i++)
            tot += v[i];
        return tot / v.length;
    }

    public static double sqr(double x) {
        return x * x;
    }

    public static double variance(double[] v) {
        double mu = mean(v);
        double sumsq = 0.0;
        for (int i = 0; i < v.length; i++)
            sumsq += sqr(mu - v[i]);
        return sumsq / (v.length);
    }

    public static double sdev(double[] v) {
        return Math.sqrt(variance(v));
    }

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
            if (args.length > 0) {
                max_iter = Integer.parseInt(args[0]);
            } else {
                max_iter = 1000;
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
                    double nr_of_routes = solution.getRoutes().size();
                    double nr_of_orig_vehicles = vrp.getVehicles().size();
                    double[] driving_times = new double[(int)(nr_of_routes)];
                    double c = 0.0;
                    int i = 0;
                    for(VehicleRoute r : solution.getRoutes()){
                        c += stateManager.getRouteState(r, InternalStates.COSTS, Double.class);
                        c += r.getVehicle().getType().getVehicleCostParams().fix;
                        driving_times[i] = r.getEnd().getArrTime() - r.getDepartureTime();
                        i += 1;
                    }
                    double base_cost = c;
                    c += base_cost * 10 * (nr_of_orig_vehicles - nr_of_routes)/nr_of_orig_vehicles;
                    c += base_cost * 10 * sdev(driving_times)/mean(driving_times);

                    // The cost of unassigned jobs (i.e., shops not visited)
                    // This setting (0.5) is quite aggressive
                    c += solution.getUnassignedJobs().size() * c * .5;
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
