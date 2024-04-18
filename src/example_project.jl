struct MyLocalizationType
    time::Float64
    position::SVector{3, Float64} # position of center of vehicle
    orientation::SVector{4, Float64} # represented as quaternion
    velocity::SVector{3, Float64}
    angular_velocity::SVector{3, Float64} # angular velocity around x,y,z axes
    current_segment::RoadSegment
end

struct MyPerceptionType
    field1::Int
    field2::Float64
end

using StaticArrays

function quaternion_to_angle_z(q)
    # q is given as (w, x, y, z)
    w, x, y, z = q
    # Ensure it is a unit quaternion (normalize if not sure)
    norm_q = sqrt(w^2 + x^2 + y^2 + z^2)
    w, x, y, z = w / norm_q, x / norm_q, y / norm_q, z / norm_q
    # Calculate the angle from the quaternion
    theta = 2 * acos(w)
    # Ensure the angle is correctly oriented for z-axis rotation
    if z < 0
        theta = -theta
    end
    return theta
end

function angle_to_quaternion_z(theta)
    # Calculate the quaternion components
    w = cos(theta / 2)
    z = sin(theta / 2)
    # Since the rotation is about the z-axis, x and y components are zero
    return (w, 0, 0, z)
end

function h_imu(x)
    linear_vel = x[8:10] # velocity
    angular_vel = x[11:13] # angular velocity
    imu_measurement = [linear_vel; angular_vel]
    return imu_measurement
end

function Jac_h_imu(x)
    J = zeros(6, 13)
    J[1:3, 8:10] = I(3) # Derivative w.r.t linear velocity
    J[4:6, 11:13] = I(3) # Derivative w.r.t angular velocity
    return J
end

function localize(gps_channel, imu_channel, localization_state_channel, map_segments)
    # Set up algorithm / initialize variables
    current_time = time()
    previous_time = current_time
    time_step = 0.1 # 10 hertz

    # initialize the state estimate
    time_estimate = time()
    position_estimate = [fresh_gps_meas.lat, fresh_gps_meas.long, 0]
    orientation_estimate = Quaternion{Float64}(angle_to_quaternion_z(fresh_gps_meas.heading))
    velocity_estimate = fresh_imu_meas.linear_vel
    angular_velocity_estimate = fresh_imu_meas.angular_vel
    size_estimate = [0, 0, 0]
    current_segment_estimate = nothing
    state_estimate = MyLocalizationType(time_estimate, position_estimate, orientation_estimate, velocity_estimate, angular_velocity_estimate, size_estimate, current_segment_estimate)
    
    # Initialize the covariance matrix P
    P = Diagonal([1.0 for _ in 1:13]) # Assuming a 13-dimensional state vector

    # Define the process noise covariance matrix Q
    Q = Diagonal([0.01 for _ in 1:13]) # Example values; adjust based on your system's noise characteristics

    # Define the measurement noise covariance matrices for GPS and IMU
    R_gps = Diagonal([1.0^2, 1.0^2, 0.1^2])
    R_imu = Diagonal([0.001^2, 0.001^2, 0.001^2, 0.001^2, 0.001^2, 0.001^2])

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
            predicted_state, P = predict_next_state(state_estimate, dt, P, Q)

            # update the covariance matrix
            # fuse the gps and imu measurements with the predicted state to obtain a more accurate estimate of the current state
            state_estimate, P = update_covariance_matrix(predicted_state, fresh_gps_meas, fresh_imu_meas, P, R_gps, R_imu)

            cur_segment = nothing
            for map_segment in map_segments
                if is_inside_segment(fresh_gps_meas.position, map_segment)
                    cur_segment = map_segment
                    break
                end
            end
            if cur_segment === nothing
                print("Error: car not inside a segment")
            end
            state_estimate.current_segment = cur_segment

            # add the changes into the localization_state_channel
            localization_state = state_estimate
            if isready(localization_state_channel)
                latest_localization_state = fetch(localization_state_channel)
                println(latest_localization_state)
                take!(localization_state_channel)
            end
            put!(localization_state_channel, localization_state)
        end
    end 
end


function predict_next_state(state_estimate::MyLocalizationType, delta_time::Float64, P::Matrix{Float64}, Q::Matrix{Float64})
    # Remove time and current_segment from the state_estimate
    position = state_estimate.position
    quaternion = state_estimate.orientation
    velocity = state_estimate.velocity
    angular_vel = state_estimate.angular_velocity
    x_current = vcat(position, quaternion, velocity, angular_vel)

    # Compute the Jacobian of the state vector BEFORE applying dynamics
    # This assumes Jac_x_f computes the Jacobian of the dynamics function with respect to the state vector
    F = Jac_x_f(x_current, delta_time)

    # Use rigid_body_dynamics to predict the next state based on the current state
    predicted_state_vector = rigid_body_dynamics(position, quaternion, velocity, angular_vel, delta_time)

    # add time and current_segment back in
    predicted_state = MyLocalizationType(
        time(),
        predicted_state_vector[1:3],
        predicted_state_vector[4:7],
        predicted_state_vector[8:10],
        predicted_state_vector[11:13],
        state_estimate.map_segment
    )
    
    # Predict the next covariance matrix incorporating process noise
    P_predicted = F * P * F' + Q
    
    return predicted_state, P_predicted
end


function update_covariance_matrix(predicted_state_estimate::MyLocalizationType, fresh_gps_meas, fresh_imu_meas, P, R_gps, R_imu)
    # Extract relevant information from the predicted state estimate
    position = predicted_state_estimate.position
    quaternion = predicted_state_estimate.orientation
    velocity = predicted_state_estimate.velocity
    angular_vel = predicted_state_estimate.angular_velocity

    # Measurement covariance matrices
    gps_covariance = R_gps
    imu_covariance = R_imu

    # Identity matrix
    I = Matrix{Float64}(I, 13, 13)

    # Compute the measurement model h for GPS and IMU measurements
    h_gps = h_gps([position; quaternion])
    h_imu = h_imu([velocity; angular_vel])

    # Compute the Jacobian matrices for GPS and IMU measurements
    H_gps = Jac_h_gps([position; quaternion])
    H_imu = Jac_h_imu([velocity; angular_vel])

    # Calculate Kalman Gain for GPS and IMU measurements
    K_gps = P * H_gps' / (H_gps * P * H_gps' + gps_covariance)
    K_imu = P * H_imu' / (H_imu * P * H_imu' + imu_covariance)

    # Update state estimate based on GPS and IMU measurements
    state_estimate.position += K_gps * (fresh_gps_meas - h_gps)
    state_estimate.orientation += K_gps * (fresh_gps_heading - h_gps_heading)
    state_estimate.velocity += K_imu * (fresh_imu_linear_vel - h_imu_linear_vel)
    state_estimate.angular_velocity += K_imu * (fresh_imu_angular_vel - h_imu_angular_vel)

    # Update covariance matrix
    updated_covariance_matrix = (I - K_gps * H_gps) * P * (I - K_gps * H_gps)' + K_gps * gps_covariance * K_gps' + K_imu * imu_covariance * K_imu'

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