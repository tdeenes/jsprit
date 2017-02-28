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

import com.graphhopper.jsprit.core.problem.Location;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem;
import com.graphhopper.jsprit.core.problem.job.Shipment;
import com.graphhopper.jsprit.core.problem.job.Shipment.Builder;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleImpl;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleTypeImpl;
import com.graphhopper.jsprit.core.util.Coordinate;
import com.graphhopper.jsprit.core.util.FastVehicleRoutingTransportCostsMatrix;

import java.io.*;
import java.util.ArrayList;
import java.util.List;
import com.graphhopper.jsprit.core.problem.solution.route.activity.TimeWindow;

/**
 * @author Denes Toth - load custom input files (located in the 'input' folder)
 */
public class CustomReader3 {

    private VehicleRoutingProblem.Builder vrpBuilder;

    public CustomReader3(VehicleRoutingProblem.Builder vrpBuilder) {
        this.vrpBuilder = vrpBuilder;
    }


    /*
    * main method
    */
    public void read(String file) {
        readStation(file + "_station.txt");
        readVehicle(file + "_vehicle.txt");
        readDistance(file + "_dist.txt", file + "_time.txt", false);
    }

    public void read(String file, boolean isSymmetric) {
        readStation(file + "_station.txt");
        readVehicle(file + "_vehicle.txt");
        readDistance(file + "_dist.txt", file + "_time.txt", isSymmetric);
    }


    /*
    * import station data (coordinates, demands, time windows)
    */

    // the file contains 10 (space separated) columns:
    // id, coordx, coordy, ready time, due time, duration, demand_box, demand_w, skill, day
    private void readStation(String file) {
        BufferedReader reader = getBufferedReader(file);
        String line_ = getLine(reader); //header
        // depot
        line_ = getLine(reader).trim();
        String[] tokens = line_.split("\\s+");
        String depoId = tokens[0];
        Coordinate coord_depo = makeCoord(tokens[1], tokens[2]);
        double start_depo = Double.parseDouble(tokens[3]);
        double end_depo = Double.parseDouble(tokens[4]);
        double serviceTime_depo = Double.parseDouble(tokens[5]);
        String skill_depo = tokens[7];
        int index = 1;
        // shops
        while ((line_ = getLine(reader)) != null) {
            line_ = line_.trim();
            tokens = line_.split("\\s+");
            String customerId = tokens[0];
            Coordinate coord = makeCoord(tokens[1], tokens[2]);
            String[] start = tokens[3].split("\\,");
            String[] end = tokens[4].split("\\,");
            double serviceTime = Double.parseDouble(tokens[5]);
            int demand_box = Integer.parseInt(tokens[6]);
            int demand_w = Integer.parseInt(tokens[7]);
            String[] skill_shop = tokens[8].split("\\,");
            String day = tokens[9];
            String shipmentId = customerId; // + "_day" + day;
            Builder shipmentBuilder = Shipment.Builder.newInstance(shipmentId)
                    .addSizeDimension(0, demand_box)
                    .addSizeDimension(1, demand_w)
                    .setDeliveryLocation(
                            Location.Builder.newInstance()
                                    .setCoordinate(coord).setIndex(index)
                                    .setId(customerId).build())
                    .setDeliveryServiceTime(serviceTime)
                    .setPickupLocation(
                            Location.Builder.newInstance()
                                    .setCoordinate(coord_depo)
                                    .setIndex(0).setId(depoId).build())
                    .setPickupServiceTime(serviceTime_depo)
                    .setPickupTimeWindow(TimeWindow.newInstance(start_depo, end_depo));
            for(int i=0; i<start.length; i++){
                shipmentBuilder.addDeliveryTimeWindow(
                        Double.parseDouble(start[i]),
                        Double.parseDouble(end[i]));
            }
            for(int j=0; j<skill_shop.length; j++){
                shipmentBuilder.addRequiredSkill(skill_shop[j]);
            }
            Shipment shipment = shipmentBuilder.build();
            vrpBuilder.addJob(shipment);
//            Delivery delivery = Delivery.Builder.newInstance(customerId).addSizeDimension(0, demand)
//                .setLocation(Location.Builder.newInstance().setCoordinate(coord).setIndex(index).setId(customerId).build())
//                .setServiceTime(serviceTime)
//                .setTimeWindow(TimeWindow.newInstance(start, end))
//                .build();
//            vrpBuilder.addJob(delivery);
            index++;
        }
        close(reader);
    }

    /*
    * import vehicle data
    */

    // the file contains 12 (space separated) columns, that is, for EACH vehicle:
    // id, capacity, depo_coordx, depo_coordy, start, arrival, dist_cost,
    // time_cost, fixed_cost, skill, day
    private void readVehicle(String file) {
        vrpBuilder.setFleetSize(VehicleRoutingProblem.FleetSize.FINITE);
        BufferedReader reader = getBufferedReader(file);
        String line_ = getLine(reader); //header
        while ((line_ = getLine(reader)) != null) {
            line_ = line_.trim();
            String[] tokens = line_.split("\\s+");
            String vid = tokens[0];
            int capacity_box = Integer.parseInt(tokens[1]);
            int capacity_w = Integer.parseInt(tokens[2]);
            Coordinate coord = makeCoord(tokens[3], tokens[4]);
            double start = Double.parseDouble(tokens[5]);
            double end = Double.parseDouble(tokens[6]);
            double distcost = Double.parseDouble(tokens[7]);
            double timecost = Double.parseDouble(tokens[8]);
            double fixedcost = Double.parseDouble(tokens[9]);
            String[] vehicle_skills = tokens[10].split("\\,");
            String day = tokens[11];
            VehicleTypeImpl.Builder typeBuilder = VehicleTypeImpl.Builder
                    .newInstance("type" + capacity_box)
                    .addCapacityDimension(0, capacity_box)
                    .addCapacityDimension(1, capacity_w)
                    .setCostPerDistance(distcost)
                    //.setCostPerTime(timecost)
                    .setCostPerTransportTime(timecost)
                    .setCostPerWaitingTime(timecost)
                    .setFixedCost(fixedcost);
            VehicleTypeImpl vehicleType = typeBuilder.build();
            VehicleImpl.Builder vehicleBuilder = VehicleImpl.Builder
                    .newInstance(vid)
                    .setReturnToDepot(true)
                    .setEarliestStart(start).setLatestArrival(end)
                    .setStartLocation(
                            Location.Builder.newInstance()
                                    .setIndex(0).setId("0")
                                    .setCoordinate(coord).build())
                    .setType(vehicleType);
            for(String sk : vehicle_skills) {
                vehicleBuilder = vehicleBuilder.addSkill(sk);
            }
            VehicleImpl vehicle = vehicleBuilder.build();
            vrpBuilder.addVehicle(vehicle);
        }
        close(reader);
    }

    /*
    * import distance matrices (road distance and travel time)
    */

    // workhorse fn
    private void readDistance(String matrixDist, String matrixTime, boolean isSymmetric) {
        BufferedReader reader = getBufferedReader(matrixDist);
        List<Double> distances = new ArrayList<Double>();
        String line_;
        int dimensions = 0;
        while ((line_ = getLine(reader)) != null) {
            String line = line_.trim();
            String[] tokens = line.trim().split("\\s+");
            for (String s : tokens) distances.add(Double.parseDouble(s));
            dimensions++;
        }
        close(reader);
        BufferedReader reader2 = getBufferedReader(matrixTime);
        List<Double> times = new ArrayList<Double>();
        line_ = "";
        while ((line_ = getLine(reader2)) != null) {
            String line = line_.trim();
            String[] tokens = line.trim().split("\\s+");
            for (String s : tokens) times.add(Double.parseDouble(s));
        }
        close(reader2);
        FastVehicleRoutingTransportCostsMatrix.Builder matrixBuilder = FastVehicleRoutingTransportCostsMatrix.Builder.newInstance(dimensions, isSymmetric);
        int fromIndex = 0;
        int toIndex = 0;
        for (int i = 0; i < distances.size(); i++) {
            if (toIndex == dimensions) {
                fromIndex++;
                toIndex = 0;
            }
            matrixBuilder.addTransportDistance(fromIndex, toIndex, distances.get(i));
            matrixBuilder.addTransportTime(fromIndex, toIndex, times.get(i));
            toIndex++;
        }
        vrpBuilder.setRoutingCost(matrixBuilder.build());
    }


    /*
    * helper functions
    */

    // get coordinates
    private Coordinate makeCoord(String xString, String yString) {
        double x = Double.parseDouble(xString);
        double y = Double.parseDouble(yString);
        return new Coordinate(x, y);
    }

    // close buffered file
    private void close(BufferedReader reader) {
        try {
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // read in a line
    private String getLine(BufferedReader reader) {
        String s = null;
        try {
            s = reader.readLine();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return s;
    }

    // open a file
    private BufferedReader getBufferedReader(String filename) {
        BufferedReader bufferedReader = null;
        try {
            bufferedReader = new BufferedReader(new FileReader(new File(filename)));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        return bufferedReader;
    }

}
