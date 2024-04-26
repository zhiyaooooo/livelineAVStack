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




# add a parameter map
function perceptionnnnnnnnnnn(cam_meas_channel, localization_state_channel, perception_state_channel, map)
    # set up stuff
    
    # from camera to vehicle, position of camera related to vehicle
    T_body_cam1 = get_cam_transform(1)
    T_body_cam2 = get_cam_transform(2)
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

        # read location info
        ego_orientation = myloc.orientation
        ego_position = myloc.position
        vehicle_size = myloc.size
        location_points = generate_location_points_from_map(map, ego_position[3])
        quat_points = uniform_quaternion_points(0, pi, pi/4)

        # process bounding boxes / run ekf / do what you think is good
        image_width = fresh_cam_meas[end].image_width
        image_height = fresh_cam_meas[end].image_height
        pixel_len = fresh_cam_meas[end].pixel_length
        focal_len = fresh_cam_meas[end].focal_length
        time_now = fresh_cam_meas[end].time
        bboxes_of_objects = get_objects(fresh_cam_meas, image_width, image_height)
        update_tracks(tracks, bboxes_of_objects, time_now, ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, 
                        vehicle_size, image_width, image_height, pixel_len, focal_len, quat_points, location_points)

        
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

function get_particle_from_gt(gt)
    xyz = gt.position
    orient = gt.orientation
    vel = norm(gt.velocity)
    theta = quaternion_to_angle(orient)
    particle = [xyz[1], xyz[2], xyz[3], vel, theta]
    return particle
end

function eval_perception(cam_latest, gt_latest, map, vehicle_id, T_body_camrot1, T_body_camrot2, tracks)

    #print("\n", cam_latest)
    #print("\n", gt_latest)
    #print("\n", map)
    #print("\n", vehicle_id)
    myloc = []
    for gt in gt_latest
        if gt.vehicle_id == vehicle_id
            myloc = gt
        end
    end
    
    #print("\n", myloc)
    fresh_cam_meas = cam_latest

    #print("\ncheck point 1")

    # read location info
    ego_orientation = myloc.orientation
    #print("\ncheck point 2")
    
    ego_position = myloc.position
    vehicle_size = myloc.size
    #print("\ncheck point 3")
    
    location_points = generate_location_points_from_map(map, ego_position[3])
    #print(ego_position[3])
    #print(location_points[1])
    #print("\ncheck point 4")
    
    quat_points = uniform_quaternion_points(0, pi, pi/4)

    #print("\ncheck point 5")

    # process bounding boxes / run ekf / do what you think is good
    image_width = fresh_cam_meas[end].image_width
    image_height = fresh_cam_meas[end].image_height
    pixel_len = fresh_cam_meas[end].pixel_length
    focal_len = fresh_cam_meas[end].focal_length
    time_now = fresh_cam_meas[end].time
    bboxes_of_objects = get_objects(fresh_cam_meas, image_width, image_height)
    
    #print("\ncheck point 6")

    update_tracks(tracks, bboxes_of_objects, time_now, ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, 
                    vehicle_size, image_width, image_height, pixel_len, focal_len, quat_points, location_points)

    
    1
    best_particle_of_objects = []
    for particle in tracks[end][3]
        push!(best_particle_of_objects, particle[1:5])
    end
    true_particle_of_objects = []
    
    #print("\ncheck point 7")

    for gt in gt_latest
        if gt.vehicle_id != vehicle_id
            push!(true_particle_of_objects, get_particle_from_gt(gt))
        end
    end

    
    print("\ntime: ", time_now)
    print(" true particles: ", true_particle_of_objects)
    print(" best particles: ", best_particle_of_objects, "\n")

end










function keyboard_client_eval_perception(host::IPAddr=IPv4(0), port=4444; v_step = 1.0, s_step = π/10)
    socket = Sockets.connect(host, port)
    (peer_host, peer_port) = getpeername(socket)
    msg = deserialize(socket) # Visualization info
    @info msg


    # from camera to vehicle, position of camera related to vehicle
    T_body_cam1 = get_cam_transform(1)
    T_body_cam2 = get_cam_transform(2)
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

    map = VehicleSim.city_map()
    gt_latest = []
    cam_latest = []
    @async while isopen(socket)
        sleep(0.001)
        state_msg = deserialize(socket)
        vehicle_id = state_msg.vehicle_id
        measurements = state_msg.measurements
        #print("\nmeasurement start #\n")
        #print(measurements)
        #print("\nmeasurement end #\n")
        num_cam = 0
        num_imu = 0
        num_gps = 0
        num_gt = 0
        for meas in measurements
            if meas isa GroundTruthMeasurement
                num_gt += 1
                push!(gt_latest, meas)
            elseif meas isa CameraMeasurement
                num_cam += 1
                push!(cam_latest, meas)
            elseif meas isa IMUMeasurement
                num_imu += 1
            elseif meas isa GPSMeasurement
                num_gps += 1
            end
        end
        if length(gt_latest) > 0
            gt_list = []
            t = gt_latest[end].time
            for gt in gt_latest
                if gt.time == t
                    push!(gt_list, gt)
                end
            end
            gt_latest = gt_list
        end
        if length(cam_latest) > 2
            cam_latest = cam_latest[end-1:end]    
        end
        if length(gt_latest) > 0 && length(cam_latest) > 0
            eval_perception(cam_latest, gt_latest, map, vehicle_id, T_body_camrot1, T_body_camrot2, tracks)
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

function keyboard_client(host::IPAddr=IPv4(0), port=4444; v_step = 1.0, s_step = π/10)
    socket = Sockets.connect(host, port)
    (peer_host, peer_port) = getpeername(socket)
    msg = deserialize(socket) # Visualization info
    @info msg

    @async while isopen(socket)
        sleep(0.001)
        state_msg = deserialize(socket)
        measurements = state_msg.measurements
        #print("\nmeasurement start #\n")
        #print(measurements)
        #print("\nmeasurement end #\n")
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
