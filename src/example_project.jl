struct MyLocalizationType
    time::Float64
    position::SVector{3, Float64} # position of center of vehicle
    orientation::SVector{4, Float64} # represented as quaternion
    velocity::SVector{3, Float64}
    angular_velocity::SVector{3, Float64} # angular velocity around x,y,z axes
    size::SVector{3, Float64} # length, width, height of 3d bounding box centered at (position/orientation)
    current_segment::RoadSegment
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

using StaticArrays

function heading_to_quaternion(heading::Float64)
    # Convert heading angle to quaternion representation
    # Assume rotation around vertical (z) axis
    # Construct quaternion [cos(θ/2), 0, 0, sin(θ/2)]
    θ = deg2rad(heading)  # Convert heading angle to radians
    q = SVector(cos(θ / 2), 0.0, 0.0, sin(θ / 2))
    return q
end

function is_inside_segment(car_position::SVector{3, Float64}, segment::RoadSegment)
    within_boundaries = true
    
    # Check if the car's position is within each lane boundary
    for boundary in segment.lane_boundaries
        # Determine if the car's latitude and longitude fall within the boundary
        within_boundary = (boundary.pt_a[2] <= car_position[2] <= boundary.pt_b[2] ||
                           boundary.pt_b[2] <= car_position[2] <= boundary.pt_a[2]) &&
                          (boundary.pt_a[1] <= car_position[1] <= boundary.pt_b[1] ||
                           boundary.pt_b[1] <= car_position[1] <= boundary.pt_a[1])
        
        # If the car is not within any one boundary, it's not within the segment
        if !within_boundary
            within_boundaries = false
            break
        end
    end
    
    return within_boundaries
end

function localize(gps_channel, imu_channel, localization_state_channel, map_segments)
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
        # update the current time
        current_time = time()
        dt = current_time - previous_time

        if dt >= time_step
            # update the previous time for the next iteration
            previous_time = current_time

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

            # prediction step
            # predict the next state of the system based on the known dynamics of the vehicle. 
            predicted_state = predict_next_state(state_estimate, dt)

            # update the covariance matrix
            # fuse the gps and imu measurements with the predicted state to obtain a more accurate estimate of the current state
            state_estimate, covariance_matrix = update_covariance_matrix(predicted_state, fresh_gps_meas, fresh_imu_meas, covariance_matrix)

            # TO DO: add the current segment into the state estimate
            cur_segment = nothing
            for map_segment in map_segments
                if is_inside_segment(fresh_gps_meas.position, map_segment)
                    cur_segment = map_segment
                    break
                end
            end
            if current_segment === nothing
                print("Error: car not inside a segment")
            end
            state_estimate.current_segment = cur_segment

            # add the changes into the localization_state_channel
            localization_state = state_estimate
            if isready(localization_state_channel)
                take!(localization_state_channel)
            end
            put!(localization_state_channel, localization_state)
        end
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


function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
    # set up stuff
    while true
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end

        latest_localization_state = fetch(localization_state_channel)
        
        # process bounding boxes / run ekf / do what you think is good

        perception_state = MyPerceptionType(0,0.0)
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

function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{MyLocalizationType}(1)
    #perception_state_channel = Channel{MyPerceptionType}(1)

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
        ego_vehicle_id = measurement_msg.vehicle_id
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

    @async localize(gps_channel, imu_channel, localization_state_channel, map_segments)
    @async perception(cam_channel, localization_state_channel, perception_state_channel)
    @async decision_making(localization_state_channel, perception_state_channel, map, socket)
end

using DataStructures

const CAR_SPEED = 10.0


function calculate_segment_length(segment)
    pt_a = segment.lane_boundaries[1].pt_a
    pt_b = segment.lane_boundaries[end].pt_b
    sqrt((pt_b[1] - pt_a[1])^2 + (pt_b[2] - pt_a[2])^2)
end

function build_graph(all_segs)
    nodes = Set{Int}()
    edges = Dict{Int, Vector{Int}}()
    segments = Dict{Int, RoadSegment}()
    pullout_zones = Set{Int}()

    for (id, segment) in all_segs
        push!(nodes, id) 
        edges[id] = segment.children
        segments[id] = segment
        if contains_lane_type(segment, LaneTypes.loading_zone)
            push!(pullout_zones, id)
        end
    end

    (nodes, edges, segments, pullout_zones)
end

function contains_lane_type(segment, lane_type)
    lane_type in segment.lane_types
end

function dijkstra(graph, source_id, target_id)
    distances = Dict{Int, Float64}()
    previous = Dict{Int, Int}()
    pq = PriorityQueue()

    for node_id in keys(graph.edges)
        distances[node_id] = Inf
        enqueue!(pq, node_id, Inf)
    end
    distances[source_id] = 0
    update!(pq, source_id, 0)

    while !isempty(pq)
        current_id = dequeue!(pq)
        if current_id == target_id || (current_id in graph.pullout_zones && target_id in graph.pullout_zones)
            break
        end

        for adjacent_id in graph.edges[current_id]
            edge_weight = calculate_edge_weight(current_id, adjacent_id, graph)
            alt = distances[current_id] + edge_weight
            if alt < distances[adjacent_id]
                distances[adjacent_id] = alt
                previous[adjacent_id] = current_id
                update!(pq, adjacent_id, alt)
            end
        end
    end
    return distances, previous
end

function calculate_edge_weight(from_id, to_id, graph)
    segment = graph.segments[from_id]
    calculate_segment_length(segment) / CAR_SPEED
end

function reconstruct_path(previous, source_id, target_id)
    path = []
    current_id = target_id
    while current_id != source_id
        push!(path, current_id)
        current_id = previous[current_id]
        if isnothing(current_id)
            return []  
        end
    end
    push!(path, source_id)
    reverse(path)
end
