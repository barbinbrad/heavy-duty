"""
Problem:    
    Given a list of packages that need delivered from 
    one location to another, and a common starting point
    of couriers, find the a path that delivers all the 
    packages in the least amount of time

Initial State:
    All couriers at home with no packages

Goal State:
    All couriers at home with no packages, and all 
    packages delivered

Minimization:
    Total travel time of all couriers

Constraints:
    1) One courier per customer origin => (no requests > capacity)
    2) Couriers start from same location, with same capacity
    3) Courriers into a location equal couriers out of a location
    4) Couriers do not exceed their capacity at any time
    5) Couriers' routes include both a customer origins and destinations
    6) Couriers' go to origins before destinations 
    7) All nodes must be visited (no subtours)
"""

import numpy as np
import math
from random import random
from conversions import clip
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import gmaps
import googlemaps



GOOGLE_MAPS_API_KEY = ''
HOME_LATITUDE = 40.4785235
HOME_LONGITUDE = -80.0798291
STANDARD_CAPACITY = 100
CUSTOMERS = 10
COURIERS = 3


class Courier:
    def __init__(self, start, end, capacity=STANDARD_CAPACITY):
        self.capacity = capacity
        self.start = start
        self.end = end

class TransportRequest:
    def __init__(self, weight, origin, destination):
        self.weight = weight
        self.destination = destination
        self.destination_idx = -1
        self.origin = origin
        self.origin_idx = -1

class Location:
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude
        self.csv = str(latitude) + ',' + str(longitude)

DEFAULT_LOCATION = Location(HOME_LONGITUDE, HOME_LATITUDE)

class Planner:
    def __init__(self):
        self.requests = []
        self.couriers = []
        self.nodes = []

    def add_node_to_set(self, node):
        try:
            return self.nodes.index(node)
        except ValueError:
            self.nodes.append(node) 
            return len(self.nodes) - 1
            

    def add_request(self,request):
        request.origin_idx = self.add_node_to_set(request.origin)
        request.destination_idx = self.add_node_to_set(request.destination)
        self.requests.append(request)     
        
    def add_courier(self, courier=Courier(start=DEFAULT_LOCATION, end=DEFAULT_LOCATION)):
        self.couriers.append(courier)
        self.add_node_to_set(courier.start)
        self.add_node_to_set(courier.end)

    def create_distance_matrix(self, bypass_api=False, cache=True):
        
        if cache and self.loaded_from_cache():
            return

        self.distance_matrix = np.zeros((len(self.nodes),len(self.nodes)))        

        for i in range(len(self.nodes)):
            for j in range(len(self.nodes)):
                if i == j:
                    self.distance_matrix[i][j] = 0
                else:
                    if bypass_api:
                        distance = random() * 20
                        self.distance_matrix[i][j] = distance
                        self.distance_matrix[j][i] = distance
                    elif self.distance_matrix[i][j] != 0.0: 
                        google_maps_api_result = googlemaps.directions(
                        self.nodes[i].destination.csv,
                        self.requests[j].origin.csv,
                        mode = 'driving')
            
                        self.distance_matrix[i][j] = google_maps_api_result[0]['legs'][0]['distance']['value']
                        self.distance_matrix[j][i] = google_maps_api_result[0]['legs'][0]['distance']['value']


    def create_model(self):
        self.manager = pywrapcp.RoutingIndexManager(len(self.distance_matrix), len(self.couriers), 0)
        self.routing = pywrapcp.RoutingModel(self.manager)

    def add_constraints(self):
        # Define cost of each arc
        transit_callback_idx = self.routing.RegisterTransitCallback(self.distance_callback)
        self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_idx)

        # Add distance constraint
        dimension_name = 'Distance'
        self.routing.AddDimension(
            transit_callback_idx,
            0, # no slack
            3000, # maximum distance
            True, # start cumul to 0
            dimension_name
        )

        self.distance_dimension = self.routing.GetDimensionOrDie(dimension_name)
        self.distance_dimension.SetGlobalSpanCostCoefficient(100)
     
        for request in self.requests:
            pickup_index = self.manager.NodeToIndex(request.origin_idx)
            delivery_index = self.manager.NodeToIndex(request.destination_idx)
            self.routing.AddPickupAndDelivery(pickup_index, delivery_index)
            self.routing.solver().Add(
                self.routing.VehicleVar(pickup_index) == self.routing.VehicleVar(
                    delivery_index))
            self.routing.solver().Add(
                self.distance_dimension.CumulVar(pickup_index) <=
                self.distance_dimension.CumulVar(delivery_index))

    def distance_callback(self, from_index, to_index):
        """Returns the manhattan distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        return self.distance_matrix[from_node][to_node]

    def loaded_from_cache(self):
        # Load distance matrix from the file system and set property, returning success 
        return False

    def generate_couriers(self, c):
        for _ in range(c):
            self.add_courier()
        
    def generate_requests(self, c):
        customer_lats = np.random.normal(HOME_LATITUDE, 0.007, 2*c)
        customer_lons = np.random.normal(HOME_LONGITUDE, 0.007, 2*c)
        customer_cargo = np.random.normal(10, 5, c)

        for customer in range(c):
            cargo = customer_cargo[customer]
            origin = Location(customer_lats[customer*2], customer_lons[customer*2])
            destination = Location(customer_lats[customer*2+1], customer_lons[customer*2+1])
            request = TransportRequest(cargo, origin, destination)
            self.add_request(request)

    def print_solution(self, solution):
        """Prints solution on console."""
        print(f'Objective: {solution.ObjectiveValue()}')
        total_distance = 0
        for courier_id in range(COURIERS):
            index = self.routing.Start(courier_id)
            plan_output = 'Route for vehicle {}:\n'.format(courier_id)
            route_distance = 0
            while not self.routing.IsEnd(index):
                plan_output += ' {} -> '.format(self.manager.IndexToNode(index))
                previous_index = index
                index = solution.Value(self.routing.NextVar(index))
                route_distance += self.routing.GetArcCostForVehicle(
                    previous_index, index, courier_id)
            plan_output += '{}\n'.format(self.manager.IndexToNode(index))
            plan_output += 'Distance of the route: {}m\n'.format(route_distance)
            print(plan_output)
            total_distance += route_distance
        print('Total Distance of all routes: {}m'.format(total_distance))

def main():
    planner = Planner()

    planner.generate_couriers(COURIERS)
    planner.generate_requests(CUSTOMERS)
    
    planner.create_distance_matrix(bypass_api=True)
    planner.create_model()
    planner.add_constraints()

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)

    # Solve the problem.
    solution = planner.routing.SolveWithParameters(search_parameters)

    if solution:
        planner.print_solution(solution)

if __name__ == "__main__":
    # execute only if run as a script
    main()