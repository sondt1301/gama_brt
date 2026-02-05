model BRT

import "Traffic.gaml"

global {
	file shp_roads <- file("../includes/roads.shp");
	file shp_nodes <- file("../includes/nodes.shp");
	
	geometry shape <- envelope(shp_roads) + 50; 
	
	graph road_network;
	
	int bus_stop_duration <- 5;
	float prob_vehicle_spawn <- 0.5;
	float prob_car_motor_spawn <- 0.3;
	int traffic_signal_time <- 30;
	int brt_cycle <- 20;
	
	init {
		create road from: shp_roads with: [num_lanes::int(read("lanes"))] {
			// Create another road in the opposite direction
			create road {
				num_lanes <- myself.num_lanes;
				shape <- polyline(reverse(myself.shape.points));
				maxspeed <- myself.maxspeed;
				linked_road <- myself;
			}
		}
		
		create intersection from: shp_nodes
				with: [is_traffic_signal::(read("type") = "traffic_signals;crossing")] {
			time_to_change <- traffic_signal_time#s;
		}
        
		// Create a graph representing the road network, with road lengths as weights
		map edge_weights <- road as_map (each::each.shape.perimeter);
		road_network <- as_driving_graph(road, intersection) with_weights edge_weights;
		
		// Initialize the traffic lights
		ask intersection {
			do initialize;
		}
	}
	
	reflex spawn_traffic {
		if (flip(prob_vehicle_spawn)) {
			if flip(prob_car_motor_spawn) {
				create car;
			} else {
				create motorbike;
			}
		}
	}
	
	reflex spawn_brt {
		if (every(brt_cycle #cycle)) {
			create brt_bus;
			write "spawn bus";
		}
	}
}

species car parent: base_vehicle {
	init {
		vehicle_length <- 5.0 #m;
		max_speed <- rnd(40, 60) #km / #h;
		max_acceleration <- rnd(1.5, 3.5);
		right_side_driving <- true;
		allowed_lanes <- [0, 1, 2];
		num_lanes_occupied <- 2;
		color <- #yellow;
	}
	
	reflex select_next_path when: current_path = nil {
		list<intersection> dst_nodes <- [intersection[264], intersection[281]];
		do compute_path graph: road_network nodes: dst_nodes;
	}
	
	reflex commute when: current_path != nil {
		do drive;
		
		if (current_path = nil) {
			do unregister;
			do die;
		}
	}
}

species motorbike parent: base_vehicle {
	init {
		vehicle_length <- 2.0 #m;
		max_speed <- rnd(30, 50) #km / #h;
		max_acceleration <- rnd(1.5, 3.5);
		right_side_driving <- true;
		allowed_lanes <- [0, 1, 2, 3];
		num_lanes_occupied <- 1;
		color <- #red;
	}
	
	reflex select_next_path when: current_path = nil {
		list<intersection> dst_nodes <- [intersection[263], intersection[281]];
		do compute_path graph: road_network nodes: dst_nodes;
	}
	
	reflex commute when: current_path != nil {
		do drive;
		
		if (current_path = nil) {
			do unregister;
			do die;
		}
	}
}

species brt_bus parent: base_vehicle {
	graph road_graph;        
	int waiting <- 0;              
	list<intersection> stops <- [intersection[124], intersection[256]];
	int stop_index <- 0;      
	
	init {
		road_graph <- road_network;
		vehicle_length <- 18.0 #m;
		max_speed <- 60 #km / #h;
		max_acceleration <- 3.5;
		color <- #green;
		right_side_driving <- false;
		lowest_lane <- 4;
		num_lanes_occupied <- 2;
	}
	
	reflex select_next_path when: current_path = nil {
		// Ensure buses go through stops
		list<intersection> dst_nodes <- [intersection[118]] + stops + [intersection[281]];
		do compute_path graph: road_graph nodes: dst_nodes;
	}
	
	reflex commute when: current_path != nil {
		// Check if bus is at the stop
		if (stop_index < length(stops)) {
			intersection target_stop <- stops[stop_index];
			if ((self.location distance_to target_stop.location) < 10) {
				waiting <- bus_stop_duration;
				stop_index <- stop_index + 1;
				write "Bus stopping at stop " + target_stop.index;
				return;
			}
		}
		// If bus is at the stop, stops
		if (waiting > 0) {
			waiting <- waiting - 1;
			return;
		}
		
		do drive;
		
		if (current_path = nil) {
			do unregister;
			do die;
		}
	}
}

experiment BRT type: gui {
	parameter "Waiting time at each bus stop" var:bus_stop_duration min:0 max:10;
	parameter "Prob spawn for both car and motorbike at a step" var:prob_vehicle_spawn min:0.0 max: 1.0;
	parameter "Prob spawn car/motorbike" var:prob_car_motor_spawn min:0.0 max: 1.0;
	parameter "Traffic light change time" var:traffic_signal_time min:0 max:200;
	parameter "Number of cycles the BRT bus spawn" var:brt_cycle min:1;
	
	output synchronized: true {
		display map type: 2d background: #gray {
			species road aspect: base;
			species intersection aspect: base;
			species car aspect: base;
			species motorbike aspect: base;
			species brt_bus aspect: base;
		}
	}
}
