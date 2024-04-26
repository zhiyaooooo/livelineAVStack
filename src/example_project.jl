struct MyLocalizationType
    time::Float64
    vehicle_id::Int
    position::SVector{3, Float64} # position of center of vehicle
    orientation::SVector{4, Float64} # represented as quaternion
    velocity::SVector{3, Float64}
    angular_velocity::SVector{3, Float64} # angular velocity around x,y,z axes
    size::SVector{3, Float64} # length, width, height of 3d bounding box centered at (position/orientation)
end

struct MyPerceptionType
    time::Float64
    vehicle_id::Int
    position::SVector{3, Float64} # position of center of vehicle
    orientation::SVector{4, Float64} # represented as quaternion
    velocity::SVector{3, Float64}
    steering_angle::Float64
    size::SVector{3, Float64} # length, width, height of 3d bounding box centered at (position/orientation)
end

function localize(gps_channel, imu_channel, localization_state_channel)
    # Set up algorithm / initialize variables
    current_time = time()
    previous_time = current_time
    time_step = 0.1 # 10 hertz

    time_estimate = time()
    position_estimate = [fresh_gps_meas.lat, fresh_gps_meas.long, 0]
    orientation_estimate = Quaternion{Float64}(heading_to_quaternion(fresh_gps_meas.heading))
    velocity_estimate = fresh_imu_meas.linear_vel
    angular_velocity_estimate = fresh_imu_meas.angular_vel
    size_estimate = [0, 0, 0]
    current_segment_estimate = nothing

    state_estimate = MyLocalizationType(time_estimate, position_estimate, orientation_estimate, velocity_estimate, angular_velocity_estimate, size_estimate, current_segment_estimate)

    covariance_matrix = Diagonal([
        0.01,   # Variance of time
        1.0,    # Variance of position_x
        1.0,    # Variance of position_y
        1.0,    # Variance of position_z
        0.1,   # Variance of orientation_1
        0.1,   # Variance of orientation_2
        0.1,   # Variance of orientation_3
        0.1,   # Variance of orientation_4
        0.001,  # Variance of velocity_x
        0.001,  # Variance of velocity_y
        0.001,  # Variance of velocity_z
        0.001, # Variance of angular_velocity_x
        0.001, # Variance of angular_velocity_y
        0.001, # Variance of angular_velocity_z
        0.0,    # Variance of size_length
        0.0,    # Variance of size_width
        0.0     # Variance of size_height
    ])

    while true
        fresh_gps_meas = []
        while isready(gps_channel)
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end
        fresh_imu_meas = []
        while isready(imu_channel)
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end

        # process measurements
        # placeholder implementation -- just sends the measurements from the sensors without doing any processing
        orientation = Quaternion{Float64}(angle_to_quaternion_z(fresh_gps_meas.heading))
        localization_state = MyLocalizationType(time(), [fresh_gps_meas.lat, fresh_gps_meas.long, 2.6455622], orientation, fresh_imu_meas.velocity, fresh_imu_meas.angular_velocity)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
end


function predict_next_state(state_estimate::MyLocalizationType, delta_time::Float64)
    """
    Given the current state information, use velocity info to predict the future state of the car
    """
    # Extract relevant information from the state estimate
    position = state_estimate.position
    orientation = state_estimate.orientation
    velocity = state_estimate.velocity
    angular_velocity = state_estimate.angular_velocity

    # TO DO: not sure if this is properly accounting for angular velocity

    # Update orientation based on angular velocity
    q_angular_velocity = Quaternion{Float64}([0.0, angular_velocity...])
    quaternion_multiply!(orientation, q_angular_velocity, orientation)
    normalize!(orientation)

    # Update position based on velocity and orientation
    R = quaternion_to_rotation_matrix(orientation)
    position += delta_time * (R * velocity)

    predicted_state = MyLocalizationType(
        state_estimate.time + delta_time,
        position,
        orientation,
        velocity,
        angular_velocity,
        state_estimate.size,
        state_estimate.current_segment
    )
    return predicted_state
end


function update_covariance_matrix(predicted_state_estimate::MyLocalizationType, gps_measurement::GPSMeasurement, imu_measurement::IMUMeasurement, covariance_matrix::Matrix{Float64})
    """
    Use the predicted state and the real measurements to update the covariance matrix for future calculations
    """
    gps_position = [gps_measurement.lat, gps_measurement.long, 0.0]
    gps_heading = gps_measurement.heading
    imu_linear_vel = imu_measurement.linear_vel
    imu_angular_vel = imu_measurement.angular_vel

    predicted_gps_position = predicted_state_estimate.position
    predicted_imu_linear_vel = predicted_state_estimate.velocity
    predicted_imu_angular_vel = predicted_state_estimate.angular_velocity

    # Measurement covariance
    # found these vals in the measurements.jl file
    gps_covariance = Diagonal([1.0, 1.0, 0.01])
    imu_covariance = Diagonal([0.000001, 0.000001, 0.000001])

    # Calculate Kalman gain
    kalman_gain_gps = covariance_matrix * inv(covariance_matrix + gps_covariance)
    kalman_gain_imu = covariance_matrix * inv(covariance_matrix + imu_covariance)

    # Update state estimate
    state_estimate.position += kalman_gain_gps * (gps_position - predicted_gps_position)
    state_estimate.orientation += kalman_gain_gps * (heading_to_quaternion(gps_heading) - predicted_state_estimate.orientation)
    state_estimate.velocity += kalman_gain_imu * (imu_linear_vel - predicted_imu_linear_vel)
    state_estimate.angular_velocity += kalman_gain_imu * (imu_angular_vel - predicted_imu_angular_vel)

    # Update covariance matrix
    updated_covariance_matrix = covariance_matrix - kalman_gain_gps * covariance_matrix - kalman_gain_imu * covariance_matrix

    return state_estimate, updated_covariance_matrix
end


function perception(cam_meas_channel, localization_state_channel, perception_state_channel, map)
    # set up stuff

    println("get into per channel1")
    # from camera to vehicle, position of camera related to vehicle
    # try
    T_body_cam1 = get_cam_transform(1)
    T_body_cam2 = get_cam_transform(2)
    # catch each
    #     println(each)
    # end
    println("get into per channel2")
    # from photo to vehicle, only rotation
    T_cam_camrot = get_rotated_camera_transform()
    # from photo to body
    T_body_camrot1 = multiply_transforms(T_body_cam1, T_cam_camrot)
    T_body_camrot2 = multiply_transforms(T_body_cam2, T_cam_camrot)

    # tracks = [track1, track2,... track]
    # track = [time, [obj1_particles, obj2_particles, ...],[obj1_best_particle, obj2_best_particle, ...]]
    tracks = []
    track = [time(), [], []]

    push!(tracks, track)

    while true
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end
 

        latest_localization_state = fetch(localization_state_channel)
        myloc = latest_localization_state
        println(myloc)

        # read location info
        ego_orientation = myloc.orientation
        ego_position = myloc.position
        vehicle_size = myloc.size
        location_points = generate_location_points_from_map(map, ego_position[3])
        quat_points = uniform_quaternion_points(0, pi, pi/4)

        # process bounding boxes / run ekf / do what you think is good

        
        perception_state = []
        best_particle_of_objects = tracks[end][3]
        for i in 1:length(best_particle_of_objects)
            time_now = fresh_cam_meas[end].time
            vehicle_id = myloc.vehicle_id
            position = best_particle_of_objects[i][1:3]
            theta = best_particle_of_objects[i][5]
            orientation = angle_to_quaternion(theta)
            v = best_particle_of_objects[i][4]
            velocity = [v*cos(theta), v*sin(theta), 0]
            steering_angle = 0
            size = best_particle_of_objects[i][6:8]
            myperception_object = MyPerceptionType(time_now, vehicle_id, position, orientation, velocity, steering_angle, size)
            push!(perception_state, myperception_object)
        end


        
        perception_state = []
        best_particle_of_objects = tracks[end][3]
        for i in 1:length(best_particle_of_objects)
            time_now = fresh_cam_meas[end].time
            vehicle_id = myloc.vehicle_id
            position = best_particle_of_objects[i][1:3]
            theta = best_particle_of_objects[i][5]
            orientation = angle_to_quaternion(theta)
            v = best_particle_of_objects[i][4]
            velocity = [v*cos(theta), v*sin(theta), 0]
            steering_angle = 0
            size = best_particle_of_objects[i][6:8]
            myperception_object = MyPerceptionType(time_now, vehicle_id, position, orientation, velocity, steering_angle, size)
            push!(perception_state, myperception_object)
        end


        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
    end
end

function decision_making(localization_state_channel, 
        perception_state_channel, 
        map, 
        target_road_segment_id, 
        socket)
    # do some setup
    while true
        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)

        # figure out what to do ... setup motion planning problem etc
        steering_angle = 0.0
        target_vel = 0.0
        cmd = (steering_angle, target_vel, true)
        serialize(socket, cmd)
    end
end
catch each
    println(each)
end

end
end

# function decision_making(localization_state_channel, 
#     perception_state_channel, 
#     map, 
#     target_segment_channel, 
#     socket)
# # do some setup
# flag = 0
# println("motion start")
# # println(map)
# route_flag = 1
# segments = []   


# while true
#     # sleep(0.2)

#     latest_localization_state = take!(localization_state_channel)
#     # while latest_localization_state.vehicle_id!=vehicle_id
#     #     latest_localization_state = take!(gt_channel)
#     # end
#     println("v id")
#     println(latest_localization_state.vehicle_id)

#     # println("gt")
#     # println(latest_localization_state)

#     latest_perception_state = take!(perception_state_channel)
#     println(latest_perception_state)


#     # latest_perception_state = []
#     # println("target1")
#     target_segment = fetch(target_segment_channel)
#     # println(target_segment)
#     current_segment = []
#     v1 = latest_localization_state
#     println(v1)
#     front_position = calculate_front_position(latest_localization_state.position[1], latest_localization_state.position[2], quaternion_to_angle_z(latest_localization_state.orientation), latest_localization_state.size[1])
#     car_position = [front_position[1], front_position[2], 0]
#     for map_segment in map
#         if is_inside_segment(car_position, map_segment[2])
#             # println(map_segment[1])
#             push!(current_segment, map_segment[2])
#         end

#     end
#     if length(current_segment) == 0 
#         println("Error: car not inside a segment")
#     end

#     target_segment_id = target_segment.id
#     println("target_segment")
#     println(target_segment_id)
#     if has_passed_halfway(v1.position, target_segment.lane_boundaries[2].pt_a, target_segment.lane_boundaries[3].pt_b)
#         println("reached")
#         cmd = (0.0, 0.0, true)
#         serialize(socket, cmd)
#         segments= []
#         now_target = take!(target_segment_channel)
#         route_flag = 1
#         sleep(5.0) 
#     else
#         target_segment = fetch(target_segment_channel)
#         target_segment_id = target_segment.id
#         println("target_segment")
#         println(target_segment_id)
#     # try
#     if route_flag == 1
#         start_segment_id = current_segment[1].id
#         # println("start_segment")
#         # println(current_segment[1].id)
#         # println(current_segment[1])
#     now_segments = get_route(map, start_segment_id, target_segment_id)
#     # println(now_segments)
#     segments = now_segments
#     route_flag = 0
#     end
# # catch e
# #     println(e)
# # end
#     # println(segments)
#     # segments = []
#     # push!(segments, map[32])
#     # push!(segments, map[30])
#     # push!(segments, map[28])
#     # push!(segments, map[26])
#     # push!(segments, map[24])
#     # push!(segments, map[17])
#     # push!(segments, map[14])

#     # println(segments)
#     if length(latest_perception_state)==0
#         v2 = MyPerceptionType(
#             0.0,                        # time
#             0,                          # vehicle_id
#             SVector{3, Float64}(0.0, 0.0, 0.0), # position
#             SVector{4, Float64}(0.0, 0.0, 0.0, 0.0), # orientation (quaternion)
#             SVector{3, Float64}(Inf, Inf, 0.0), # velocity
#             0.0,                        # steering_angle
#             SVector{3, Float64}(0.0, 0.0, 0.0)  # size
#         )
#         v3 = MyPerceptionType(
#             0.0,                        # time
#             0,                          # vehicle_id
#             SVector{3, Float64}(0.0, 0.0, 0.0), # position
#             SVector{4, Float64}(0.0, 0.0, 0.0, 0.0), # orientation (quaternion)
#             SVector{3, Float64}(Inf, Inf, 0.0), # velocity
#             0.0,                        # steering_angle
#             SVector{3, Float64}(0.0, 0.0, 0.0)  # size
#         )
#     elseif length(latest_perception_state)==1       
#         v2 = MyPerceptionType(
#             0.0,                        # time
#             0,                          # vehicle_id
#             SVector{3, Float64}(0.0, 0.0, 0.0), # position
#             SVector{4, Float64}(0.0, 0.0, 0.0, 0.0), # orientation (quaternion)
#             SVector{3, Float64}(Inf, Inf, 0.0), # velocity
#             0.0,                        # steering_angle
#             SVector{3, Float64}(0.0, 0.0, 0.0)  # size
#         )
#         v3 = latest_perception_state[1]
#     else   
#         dists = [Inf; [norm(v.position[1:2]-latest_localization_state.position[1:2]) for v in latest_perception_state]]
#         closest = partialsortperm(dists, 1:2)
#         v2 = latest_perception_state[closest[1]]
#         v3 = latest_perception_state[closest[2]]
#     end
#     # max_vel = Inf
#     # for segment in segments
#     #     if segment.speed_limit <= max_vel
#     #         max_vel = segment.speed_limit
#     #     end
#     # end
#     max_vel = 2.5
#     # println("cnm")
#     if length(current_segment) > 0

#     stop_sign, flag = should_stop(car_position, current_segment[1], flag; speed = max_vel, timestamp = 0.4)
#     max_vel =  stop_sign==1 ? max_vel : 0
#     end

#     # println("max_vel")
#     # println(max_vel)
#     # my_segments = []
#     # try
#         my_segments = get_lane_segments(segments, 3)
#         # println("my road")
#         # println(my_segments)
#     # catch e
#     #     println("1")
#     #     println(e)
#     # end
#     # try
#     u = pure_pursuit(v1, v2, v3, my_segments; ls = 2.0, max_vel = max_vel, timestamp = 0.2)
#     # catch e   
#     #     println("2") 
#     #     println(e)
#     # end
#     # figure out what to do ... setup motion planning problem etc
#     # println(u)
#     target_vel = u[1] + sqrt(v1.velocity[1]^2 + v1.velocity[2]^2)
#     steering_angle = u[2]
#     println(u)
#     # println(steering_angle)
#     cmd = (steering_angle, target_vel, true)
#     serialize(socket, cmd)
#     if stop_sign==0
#         sleep(2.0)
#     end
# end
# end
# end


function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()
    # println(map_segments[1])
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)
    target_segment_channel = Channel{VehicleSim.RoadSegment}(1)
    vehicle_channel = Channel{Int}(1)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    errormonitor(@async while true
        # This while loop reads to the end of the socket stream (makes sure you
        # are looking at the latest messages)
        sleep(0.001)
        local measurement_msg
        received = false
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                measurement_msg = deserialize(socket)
                received = true
            else
                break
            end
        end
        !received && continue
        target_map_segment = measurement_msg.target_segment
        !isfull(target_segment_channel) &&put!(target_segment_channel, map_segments[target_map_segment])
        # println(map_segments[target_map_segment])
        ego_vehicle_id = measurement_msg.vehicle_id
        !isfull(vehicle_channel) &&put!(vehicle_channel, ego_vehicle_id)
        # println("ego id")
        # println(ego_vehicle_id)
        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            end
        end
    end)

    @async localize(gps_channel, imu_channel, localization_state_channel)
    @async perception(cam_channel, localization_state_channel, perception_state_channel)
    @async decision_making(localization_state_channel, perception_state_channel, map, socket)
end



function build_graph(map)
    nodes = Set{Int}()
    edges = Dict{Int, Vector{Int}}()
    segments = Dict{Int, VehicleSim.RoadSegment}()

    for (id, segment) in map
        # println("Processing segment ID: ", id)  # Debug statement
        push!(nodes, id)
        edges[id] = segment.children
        segments[id] = segment
    end

    println("Graph built with nodes and edges.")  # Debug statement
    (nodes=nodes, edges=edges, segments=segments)
end

function dijkstra(graph, start_segment_id, target_segment_id)
    distances = Dict{Int, Float64}()
    previous = Dict{Int, Int}()
    pq = PriorityQueue{Int, Float64}()  # Define as Min-Priority Queue

    # Initialize distances and queue
    for node_id in keys(graph.edges)
        distances[node_id] = Inf
        pq[node_id] = Inf  # Set initial distance as Infinite
    end
    distances[start_segment_id] = 0
    pq[start_segment_id] = 0  # Set distance to start node as 0

    println("Starting Dijkstra's algorithm")

    while !isempty(pq)
        current_id = dequeue_pair!(pq) |> first  # Fetch the node with the minimum distance and remove it from pq

        # println("Processing node: ", current_id)

        # Exit loop if target is reached
        if current_id == target_segment_id
            println("Target segment reached")
            break
        end

        # Explore each adjacent node
        for adjacent_id in graph.edges[current_id]
            edge_weight = calculate_edge_weight(current_id, adjacent_id, graph)
            alt = distances[current_id] + edge_weight
            if alt < distances[adjacent_id]
                distances[adjacent_id] = alt
                previous[adjacent_id] = current_id
                pq[adjacent_id] = alt  # Correctly updates the priority queue with the new distance
                # println("Updated distance for node ", adjacent_id, " to ", alt)
            end
        end
    end

    println("Dijkstra computation completed.")
    return distances, previous
end


function calculate_edge_weight(from_id, to_id, graph)
    segment = graph.segments[from_id]
    calculate_segment_length(segment) / 10.0
end

function calculate_segment_length(segment)
    pt_a = segment.lane_boundaries[1].pt_a
    pt_b = segment.lane_boundaries[end].pt_b
    sqrt((pt_b[1] - pt_a[1])^2 + (pt_b[2] - pt_a[2])^2)
end


# Reconstructs the path from start to target using the previous map
function reconstruct_path(previous, start_segment_id, target_segment_id)
    path = []
    current_id = target_segment_id
    while current_id != start_segment_id
        push!(path, current_id)
        current_id = get(previous, current_id, nothing)
        if isnothing(current_id)
            println("No path found to segment ", current_id)  # Debug statement
            return []  # Return an empty path if no path is found
        end
    end
    push!(path, start_segment_id)
    reverse(path)
end

# Main function to get the route between two segment IDs
function get_route(map, start_segment_id, target_segment_id)
    graph = build_graph(map)
    # println("Graph: ", graph)  # Debug statement
    distances, previous = dijkstra(graph, start_segment_id, target_segment_id)
    # println("Distances: ", distances)  # Debug statement
    path = reconstruct_path(previous, start_segment_id, target_segment_id)
    map_segments = [map[id] for id in path] # Maps segment IDs to actual segment data
    map_segments
end