struct MyLocalizationType
    time::Float64
    position::SVector{3, Float64} # position of center of vehicle
    orientation::SVector{4, Float64} # represented as quaternion
    velocity::SVector{3, Float64}
    angular_velocity::SVector{3, Float64} # angular velocity around x,y,z axes
    size::SVector{3, Float64} # length, width, height of 3d bounding box centered at (position/orientation)
    map_segment::RoadSegment
end

struct MyPerceptionType
    field1::Int
    field2::Float64
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

function angular_velocity_to_quaternion(angular_velocity, dt)
    half_dt = 0.5 * dt
    axis_angle = half_dt * angular_velocity
    norm_axis_angle = norm(axis_angle)
    if norm_axis_angle < 1e-12
        return [1.0, 0.0, 0.0, 0.0] 
    else
        unit_axis = axis_angle / norm_axis_angle
        quat_increment = [cos(norm_axis_angle), 
                          sin(norm_axis_angle) * unit_axis[1],
                          sin(norm_axis_angle) * unit_axis[2],
                          sin(norm_axis_angle) * unit_axis[3]]
        return quat_increment
    end
end


function localize(gps_channel, imu_channel, localization_state_channel, map_segments)
    gps_meas = GPSMeasurement(0, 0, 0, 0)
    imu_meas = IMUMeasurement(0, zeroes(3), zeroes(3))
    
    while true
        fresh_gps_meas = []
        while isready(gps_channel)
            meas = take!(gps_channel)
            gps_meas = meas
            push!(fresh_gps_meas, meas)
        end
        
        fresh_imu_meas = []
        while isready(imu_channel)
            meas = take!(imu_channel)
            imu_meas = meas
            push!(fresh_imu_meas, meas)
        end

        #time
        # TO DO: time currently gives the current time, it should give the time that has passed since the beginning
        time = time()
        
        # orientation
        dt = 0.1
        quat_increment = angular_velocity_to_quaternion(imu_meas.angular_vel, dt)
        orientation_updated = quaternion_multiply(localization_state.orientation, quat_increment)

        # TO DO: size?

        # Find which map segment the car is located in
        current_segment = nothing
        for map_segment in values(map_segments)
            if is_inside_segment(gps_meas.position, map_segment)
                current_segment = map_segment
                break  # Found the segment, no need to continue searching
            end
        end
        if current_segment === nothing
            println("Car is not within any map segment.")
            continue
        end

        # new vals
        localization_state = MyLocalizationType(time, [gps_meas.lat, gps_meas.long, 0.0], orientation_updated, imu_meas.linear_vel, imu_meas.angular_vel, zeros(3), current_segment)

        #putting the new vals in the channel
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
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
