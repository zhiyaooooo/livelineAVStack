#Todd's Branch
struct VehicleCommand
    steering_angle::Float64
    velocity::Float64
    controlled::Bool
end


function get_c()
    c = 'x'
    try
        ret = ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, true)
        ret == 0 || error("unable to switch to raw mode")
        c = read(stdin, Char)
        ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, false)
    catch e
    end
    c
end

function keyboard_client(host::IPAddr=IPv4(0), port=4444; v_step = 1.0, s_step = π/10)
    socket = Sockets.connect(host, port)
    (peer_host, peer_port) = getpeername(socket)
    msg = deserialize(socket) # Visualization info
    @info msg

    @async while isopen(socket)
        sleep(0.001)
        state_msg = deserialize(socket)
        measurements = state_msg.measurements
        print("\nmeasurement start #\n")
        print(measurements)
        print("\nmeasurement end #\n")
        num_cam = 0
        num_imu = 0
        num_gps = 0
        num_gt = 0
        for meas in measurements
            if meas isa GroundTruthMeasurement
                num_gt += 1
            elseif meas isa CameraMeasurement
                num_cam += 1
            elseif meas isa IMUMeasurement
                num_imu += 1
            elseif meas isa GPSMeasurement
                num_gps += 1
            end
        end
        @info "Measurements received: $num_gt gt; $num_cam cam; $num_imu imu; $num_gps gps"
    end
    
    target_velocity = 0.0
    steering_angle = 0.0
    controlled = true
    
    client_info_string = 
        "********************
      Keyboard Control (manual mode)
      ********************
        -Press 'q' at any time to terminate vehicle.
        -Press 'i' to increase vehicle speed.
        -Press 'k' to decrease vehicle speed.
        -Press 'j' to increase steering angle (turn left).
        -Press 'l' to decrease steering angle (turn right)."
    @info client_info_string
    while controlled && isopen(socket)
        key = get_c()
        if key == 'q'
            # terminate vehicle
            controlled = false
            target_velocity = 0.0
            steering_angle = 0.0
            @info "Terminating Keyboard Client."
        elseif key == 'i'
            # increase target velocity
            target_velocity += v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'k'
            # decrease forward force
            target_velocity -= v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'j'
            # increase steering angle
            steering_angle += s_step
            @info "Target steering angle: $steering_angle"
        elseif key == 'l'
            # decrease steering angle
            steering_angle -= s_step
            @info "Target steering angle: $steering_angle"
        end
        cmd = (steering_angle, target_velocity, controlled)
        serialize(socket, cmd)
    end
end

function example_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = training_map()
    (; chevy_base) = load_mechanism()

    @async while isopen(socket)
        state_msg = deserialize(socket)
    end
   
    shutdown = false
    persist = true
    while isopen(socket)
        position = state_msg.q[5:7]
        @info position
        if norm(position) >= 100
            shutdown = true
            persist = false
        end
        cmd = (0.0, 2.5, persist, shutdown)
        serialize(socket, cmd) 
    end

end

# using Graphs

# function build_road_network(segments::Vector{Tuple{Int, Int, Float64}})
#     # Create an empty weighted graph. The vertices represent intersections,
#     # and the edges represent road segments. The weight represents the distance
#     # or cost of traversing the segment.
#     g = SimpleWeightedGraph()

#     # Add edges to the graph based on the provided segments
#     # Each segment is a tuple of (start_vertex, end_vertex, weight)
#     for segment in segments
#         start_vertex, end_vertex, weight = segment
#         if !has_vertex(g, start_vertex)
#             add_vertex!(g)
#         end
#         if !has_vertex(g, end_vertex)
#             add_vertex!(g)
#         end
#         add_edge!(g, start_vertex, end_vertex, weight)
#     end

#     return g
# end

# function find_shortest_path(graph, start_id, finish_id)
#     # Compute shortest paths from start_id using Dijkstra's algorithm
#     path = dijkstra_shortest_paths(graph, start_id)

#     # Retrieve the shortest path to finish_id
#     shortest_path = enumerate_paths(path, finish_id)

#     return shortest_path
# end

# # Example road segments: (start_point, end_point, distance)
# road_segments = [
#     (1, 2, 7.0),
#     (2, 3, 10.0),
#     (2, 4, 15.0),
#     (1, 4, 20.0),
#     (3, 4, 11.0),
#     (3, 5, 2.0),
#     (4, 5, 9.0)
# ]

# # Build the road network graph
# g = build_road_network(road_segments)

# # Find the shortest path from start to finish
# start_id = 1
# finish_id = 5
# shortest_path = find_shortest_path(g, start_id, finish_id)

# println("Shortest path from $start_id to $finish_id: $shortest_path")

# const RoadNetwork = Dict{Tuple{Int, Int}, RoadSegment}()

# function add_straight_segments!(all_segs, base, direction; length=40.0, ...)

#     start_node = base.id
#     end_node = seg_id
#     RoadNetwork[(start_node, end_node)] = seg
#     all_segs[end_node] = seg
# end

# using DataStructures

# function dijkstra(graph, start_id, end_id)
#     dist = Dict{Int, Float64}(start_id => 0)
#     prev = Dict{Int, Int}()
#     pq = PriorityQueue()
#     enqueue!(pq, start_id, 0)

#     while !isempty(pq)
#         current_id = dequeue!(pq)
        
#         if current_id == end_id
#             break
#         end

#         for (adj_id, seg) in graph
#             if adj_id[1] == current_id
#                 alt = dist[current_id] + seg.distance  
#                 if alt < get(dist, adj_id[2], Inf)
#                     dist[adj_id[2]] = alt
#                     prev[adj_id[2]] = current_id
#                     enqueue!(pq, adj_id[2], alt)
#                 end
#             end
#         end
#     end

#     # Reconstruct the path
#     path = []
#     u = end_id
#     while haskey(prev, u)
#         prepend!(path, u)
#         u = prev[u]
#     end
#     prepend!(path, start_id)
#     path
# end

# function navigate_vehicle(socket, start_id, end_id)
#     path = dijkstra(RoadNetwork, start_id, end_id)
#     for node in path

#         cmd = calculate_command_for_segment(RoadNetwork[node])
#         serialize(socket, cmd) 
#     end
# end

