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
    field1::Int
    field2::Float64
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
    current_segment_estimate = map_segments

    state_estimate = MyLocalizationType(time_estimate, position_estimate, orientation_estimate, velocity_estimate, angular_velocity_estimate, size_estimate, current_segment_estimate)

    covariance_matrix = Diagonal([
        0.01,   # Variance of time
        1.0,    # Variance of position_x
        1.0,    # Variance of position_y
        1.0,    # Variance of position_z
        0.01,   # Variance of orientation_1
        0.01,   # Variance of orientation_2
        0.01,   # Variance of orientation_3
        0.01,   # Variance of orientation_4
        0.001,  # Variance of velocity_x
        0.001,  # Variance of velocity_y
        0.001,  # Variance of velocity_z
        0.0001, # Variance of angular_velocity_x
        0.0001, # Variance of angular_velocity_y
        0.0001, # Variance of angular_velocity_z
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
            current_segment = nothing
            for map_segment in map_segments
                if is_inside_segment(fresh_gps_meas.position, map_segment)
                    current_segment = map_segment
                    break
            if current_segment === nothing
                print("Error: car not inside a segment")

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


function update_covariance_matrix(state_estimate::MyLocalizationType, gps_measurement::GPSMeasurement, imu_measurement::IMUMeasurement, covariance_matrix::Matrix{Float64})
    """
    Use the predicted state and the real measurements in order to update the covariance matrix for future calculations
    """
    gps_position = [gps_measurement.lat, gps_measurement.long, 0.0]
    gps_heading = gps_measurement.heading
    imu_linear_vel = imu_measurement.linear_vel
    imu_angular_vel = imu_measurement.angular_vel

    # update the estimated vals with the measurements
    state_estimate.position = (gps_position + state_estimate.position) / 2
    state_estimate.orientation = (heading_to_quaternion(gps_heading) + state_estimate.orientation)/2
    state_estimate.velocity = (imu_linear_vel + state_estimate.velocity)/2
    state_estimate.angular_velocity = (imu_angular_vel + state_estimate.angular_velocity)/2

    # TO DO: update covariance matrix based on the state_estimate
    updated_covariance_matrix = covariance_matrix

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

    #localization_state_channel = Channel{MyLocalizationType}(1)
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