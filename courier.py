"""
Problem:    
    Given a list of packages that need delivered from 
    one location to another, and a common starting point
    of couriers, find the a path(es) that delivers all the 
    packages in the least amount of time.

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

import os
from tabulate import tabulate
from random import random
from conversions import clip
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import gmaps
import googlemaps


GOOGLE_MAPS_API_KEY = os.environ['GOOGLE_MAPS_API_KEY']
GOOGLE_MAPS_MODE = 'driving' # {'driving', 'walking', 'cycling'}
HOME_LATITUDE = 40.4785235
HOME_LONGITUDE = -80.0798291
LIMITED_CAPACITY = False
MAXIMUM_CAPACITY = 100
CUSTOMERS = 10
COURIERS = 3

maps = googlemaps.Client(key=GOOGLE_MAPS_API_KEY)

class Courier:
    def __init__(self, start, end, capacity=MAXIMUM_CAPACITY):
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
        self.deliveries = [0]

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
        self.deliveries.extend([request.weight, -request.weight])     
        
    def add_courier(self, courier=Courier(start=DEFAULT_LOCATION, end=DEFAULT_LOCATION)):
        self.couriers.append(courier.capacity)
        self.add_node_to_set(courier.start)
        self.add_node_to_set(courier.end)

    def create_distance_matrix(self, bypass_api=False, cache=True):               
        print('Creating distance matrix...')
        print(GOOGLE_MAPS_API_KEY)
        if cache and self.loaded_from_cache():
            return

        self.distance_matrix = np.zeros((len(self.nodes),len(self.nodes)))        

        for i in range(len(self.nodes)):
            for j in range(len(self.nodes)):
                if i == j:
                    self.distance_matrix[i][j] = 0
                else:
                    if bypass_api:
                        distance = random() * 100
                        self.distance_matrix[i][j] = distance
                        self.distance_matrix[j][i] = distance
                    elif self.distance_matrix[i][j] != 0.0: 
                        google_maps_api_result = maps.directions(
                                            self.nodes[i].csv,
                                            self.nodes[j].csv,
                                            mode=GOOGLE_MAPS_MODE)
            
                        self.distance_matrix[i][j] = google_maps_api_result[0]['legs'][0]['distance']['value']
                        self.distance_matrix[j][i] = google_maps_api_result[0]['legs'][0]['distance']['value']


    def create_model(self):
        print('Creating the model...')
        self.manager = pywrapcp.RoutingIndexManager(len(self.distance_matrix), len(self.couriers), 0)
        self.routing = pywrapcp.RoutingModel(self.manager)

    def add_constraints(self):
        print('Adding constraints...')
        # Define cost of each arc
        transit_callback_idx = self.routing.RegisterTransitCallback(self.distance_callback)
        self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_idx)

        # Add distance constraint
        distance_label = 'Distance'
        self.routing.AddDimension(
            transit_callback_idx,
            0, # no slack
            3000, # maximum distance
            True, # start cumul to 0
            distance_label
        )

        self.distance_dimension = self.routing.GetDimensionOrDie(distance_label)
        self.distance_dimension.SetGlobalSpanCostCoefficient(100)

        if LIMITED_CAPACITY:
            # Define the capacity amounts
            capacity_callback_idx = self.routing.RegisterUnaryTransitCallback(self.capacity_callback)
            
            # Add the capacity constraint
            capacity_label = 'Capacity'
            self.routing.AddDimensionWithVehicleCapacity(
                capacity_callback_idx,
                0, # no slack
                self.couriers,
                True,
                capacity_label
            )

            self.capacity_dimension = self.routing.GetDimensionOrDie(capacity_label)

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
            if LIMITED_CAPACITY:
                self.routing.solver().Add(
                    self.capacity_dimension.CumulVar(pickup_index) >= 0
                )
                self.routing.solver().Add(
                    self.capacity_dimension.CumulVar(delivery_index) >= 0
                )

    def distance_callback(self, from_index, to_index):
        """Returns the manhattan distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        return self.distance_matrix[from_node][to_node]

    def capacity_callback(self, from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = self.manager.IndexToNode(from_index)
        return self.deliveries[from_node]

    def loaded_from_cache(self):
        # Load distance matrix from the file system and set property, returning success 
        return False

    def generate_couriers(self, c):
        print(f'Generating {c} couriers...')
        for _ in range(c):
            self.add_courier()
        
    def generate_requests(self, N):
        print(f'Generating {N} requests...')
        generated_lats = np.random.normal(HOME_LATITUDE, 0.007, 2*N)
        generated_lons = np.random.normal(HOME_LONGITUDE, 0.007, 2*N)
        generated_cargo = np.random.normal(10, 5, N)

        for customer in range(N):
            cargo = int(generated_cargo[customer])
            origin = Location(generated_lats[customer*2], generated_lons[customer*2])
            destination = Location(generated_lats[customer*2+1], generated_lons[customer*2+1])
            request = TransportRequest(cargo, origin, destination)
            self.add_request(request)

    def print_solution(self, solution):
        """Prints solution on console."""
        print(f'Objective: {solution.ObjectiveValue()}')
        total_distance = 0
        
        headers = ['Node', 'Route Distance']
        if LIMITED_CAPACITY: headers.append('Route Load')
            
        for courier_id in range(COURIERS):
            print('\nCOURIER {}:'.format(courier_id + 1))
            index = self.routing.Start(courier_id)
            rows = []
            route_distance = 0
            route_load = 0
            while not self.routing.IsEnd(index):
                node_index = self.manager.IndexToNode(index)
                previous_index = index
                index = solution.Value(self.routing.NextVar(index))
                route_distance += self.routing.GetArcCostForVehicle(
                    previous_index, index, courier_id)

                if LIMITED_CAPACITY:
                    route_load += self.deliveries[node_index]
                    rows.append([node_index, route_distance, route_load])
                else:    
                    rows.append([node_index, route_distance])

            print(tabulate(rows, headers, tablefmt='orgtbl'))
            
            total_distance += route_distance
        print('Total Distance of all routes: {}m'.format(total_distance))

    def can_connect_to_google_maps(self):
        try:
            google_maps_api_result = maps.directions(
                self.nodes[0].csv,
                self.nodes[1].csv,
                mode=GOOGLE_MAPS_MODE)
            _ = google_maps_api_result[0]['legs'][0]['distance']['value']
            return True
        except:
            return False

def GetSearchParams():
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)
    return search_parameters


def example():
    planner = Planner()

    planner.generate_couriers(COURIERS)
    planner.generate_requests(CUSTOMERS)
    bypass_api = not planner.can_connect_to_google_maps()
    planner.create_distance_matrix(bypass_api=bypass_api)
    planner.create_model()
    planner.add_constraints()

    params = GetSearchParams()

    # Solve the problem.
    print('Solving...')
    solution = planner.routing.SolveWithParameters(params)

    if solution:
        planner.print_solution(solution)

if __name__ == "__main__":
    # execute only if run as a script
    example()
