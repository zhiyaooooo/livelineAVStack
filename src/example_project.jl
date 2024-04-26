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

function get_tangent(pt_a, pt_b)

    tangent = pt_b - pt_a
    res =  tangent ./ norm(tangent)
    # println("get tangent")
    return res
end

function get_normal(tangent)
    return perp(tangent)
end

function approximate_curv_boundary_with_polyline(pt_a, pt_b, curvature, num_points)
    # 计算半径R
    # println("approximate start")
    R = 1 / abs(curvature)
    # 确定圆心方向（左侧或右侧）
    direction = curvature > 0 ? 1 : -1
    # 计算圆心相对于pt_a和pt_b中点的位置
    mid_point = (pt_a + pt_b) / 2
    chord_length = norm(pt_b - pt_a)
    sagitta = R - sqrt(R^2 - (chord_length / 2)^2)  # 弓高公式
    normal = normalize(perp(pt_b - pt_a))  # 垂直于弦的单位向量
    circle_center = mid_point + direction * normal * sagitta

    # 计算起始和结束角度
    start_angle = atan(pt_a[2] - circle_center[2], pt_a[1] - circle_center[1])
    end_angle = atan(pt_b[2] - circle_center[2], pt_b[1] - circle_center[1])

    # 确保顺时针或逆时针方向正确
    if direction < 0 && start_angle < end_angle
        start_angle += 2 * π
    elseif direction > 0 && start_angle > end_angle
        end_angle += 2 * π
    end

    # 生成折线的顶点
    angles = range(start_angle, end_angle, length=num_points)
    polyline_points = [circle_center + R * [cos(θ), sin(θ)] for θ in angles]
    segments = [MySegment(polyline_points[i], polyline_points[i+1],
                          get_tangent(polyline_points[i], polyline_points[i+1]),
                          get_normal(get_tangent(polyline_points[i], polyline_points[i+1])))
                for i in 1:length(polyline_points)-1]
    # println("approximate start")

    return segments
end

# 计算圆弧的圆心和角度范围
function calculate_arc_properties(arc)
    radius = abs(1 / arc.curvature)
    orientation = sign(arc.curvature)  # 曲率的符号决定弧的方向（顺时针或逆时针）
    
    # 计算向量从 pt_a 到 pt_b
    chord_vector = arc.pt_b - arc.pt_a
    chord_length = norm(chord_vector)
    chord_midpoint = (arc.pt_a + arc.pt_b) / 2
    
    # 勾股定理求圆心到弦中点的距离
    half_chord = chord_length / 2
    perpendicular = sqrt(radius^2 - half_chord^2)
    
    # 计算圆心位置
    normal_vector = [-chord_vector[2], chord_vector[1]] * orientation
    normal_vector = normal_vector / norm(normal_vector)
    center = chord_midpoint + perpendicular * normal_vector
    
    # 计算起始和终止角度
    start_angle = atan((arc.pt_a[2] - center[2]), (arc.pt_a[1] - center[1]))
    end_angle = atan((arc.pt_b[2] - center[2]), (arc.pt_b[1] - center[1]))
    
    return center, radius, start_angle, end_angle
end

# 根据圆弧属性生成多段线点
function generate_arc_points(center, radius, start_angle, end_angle, num_points)
    angles = range(start_angle, end_angle, length=num_points)
    points = [center + radius * [cos(θ), sin(θ)] for θ in angles]
    return points
end

# 处理两个圆弧边界，创建中间多段线
function generate_mid_polyline(arc1, arc2, n_points)
    center1, radius1, start_angle1, end_angle1 = calculate_arc_properties(arc1)
    center2, radius2, start_angle2, end_angle2 = calculate_arc_properties(arc2)
    
    # 生成每个圆弧的点
    points1 = generate_arc_points(center1, radius1, start_angle1, end_angle1, n_points)
    points2 = generate_arc_points(center2, radius2, start_angle2, end_angle2, n_points)
    
    # 计算中间点
    mid_points = [(p1 + p2) / 2 for (p1, p2) in zip(points1, points2)]
    # println(mid_points)
    segments = []
    for i in 1:length(mid_points)-1
        pt1 = mid_points[i]
        pt2 = mid_points[i+1]
        tangent = get_tangent(pt1, pt2)
        temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
        push!(segments, temp)
    end
    return segments
end


function get_lane_segments(segments, flag)
    # println("get lane start")
    route = []
    num_points = 5
    if flag == 1
        for seg in segments
            if length(seg.lane_types)>1 && seg.lane_types[2] == VehicleSim.loading_zone
                if seg.lane_boundaries[2].pt_a == seg.lane_boundaries[3].pt_a
                    #loadingzone入口
                        pt1 = seg.lane_boundaries[1].pt_a
                        pt2 = seg.lane_boundaries[2].pt_b
                        tangent = get_tangent(pt1, pt2)
                        temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                        push!(route, temp)
                elseif seg.lane_boundaries[2].pt_b == seg.lane_boundaries[3].pt_b
                    #loading_zone出口
                    pt1 = seg.lane_boundaries[2].pt_a
                    pt2 = seg.lane_boundaries[1].pt_b
                    tangent = get_tangent(pt1, pt2)
                    temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                    push!(route, temp)
                else
                    pt1 = seg.lane_boundaries[2].pt_a
                    pt2 = seg.lane_boundaries[2].pt_b
                    tangent = get_tangent(pt1, pt2)
                    temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                    push!(route, temp)
                end

            elseif seg.lane_boundaries[1].curvature == 0.0
                pt1 = seg.lane_boundaries[1].pt_a
                pt2 = seg.lane_boundaries[1].pt_b
                tangent = get_tangent(pt1, pt2)
                temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                push!(route, temp)
            else
                pt1 = seg.lane_boundaries[1].pt_a
                pt2 = seg.lane_boundaries[1].pt_b
                temps = approximate_curv_boundary_with_polyline(pt1, pt2, (seg.lane_boundaries[1].curvature + seg.lane_boundaries[2].curvature) / 2, num_points)
                append!(route, temps)
            end
        end
    elseif flag == 2
        for seg in segments
            if length(seg.lane_types)>1 && seg.lane_types[2] == VehicleSim.loading_zone
                pt1 = seg.lane_boundaries[3].pt_a
                pt2 = seg.lane_boundaries[3].pt_b
                tangent = get_tangent(pt1, pt2)
                temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                push!(route, temp)
            elseif seg.lane_boundaries[1].curvature == 0.0
                pt1 = seg.lane_boundaries[2].pt_a
                pt2 = seg.lane_boundaries[2].pt_b
                tangent = get_tangent(pt1, pt2)
                temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                push!(route, temp)
            else
                pt1 = seg.lane_boundaries[2].pt_a
                pt2 = seg.lane_boundaries[2].pt_b
                temps = approximate_curv_boundary_with_polyline(pt1, pt2, seg.lane_boundaries[1].curvature, num_points)
                append!(route, temps)
            end
        end
    else
        for (index, seg) in enumerate(segments)
            # println(seg)
            if length(seg.lane_types)>1 && seg.lane_types[2] == VehicleSim.loading_zone
                if seg.lane_boundaries[2].pt_a == seg.lane_boundaries[3].pt_a
                        #我就在这
                        if index == length(segments)-1
                        pt1 = (seg.lane_boundaries[1].pt_a + seg.lane_boundaries[2].pt_a) / 2
                        pt2 = (seg.lane_boundaries[2].pt_b + seg.lane_boundaries[3].pt_b) / 2
                        tangent = get_tangent(pt1, pt2)
                        temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                        push!(route, temp)
                        else
                            pt1 = (seg.lane_boundaries[1].pt_a + seg.lane_boundaries[2].pt_a) / 2
                            pt2 = (seg.lane_boundaries[1].pt_b + seg.lane_boundaries[2].pt_b) / 2
                            tangent = get_tangent(pt1, pt2)
                            temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                            push!(route, temp)     
                        end
                elseif seg.lane_boundaries[2].pt_b == seg.lane_boundaries[3].pt_b
                    #loading_zone出口
                    if index == 2
                    pt1 = (seg.lane_boundaries[2].pt_a + seg.lane_boundaries[3].pt_a) / 2
                    pt2 = (seg.lane_boundaries[1].pt_b + seg.lane_boundaries[2].pt_b) / 2
                    tangent = get_tangent(pt1, pt2)
                    temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                    push!(route, temp)
                    else 
                        pt1 = (seg.lane_boundaries[1].pt_a + seg.lane_boundaries[2].pt_a) / 2
                        pt2 = (seg.lane_boundaries[1].pt_b + seg.lane_boundaries[2].pt_b) / 2
                        tangent = get_tangent(pt1, pt2)
                        temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                        push!(route, temp)
                    end
                else
                    if index == length(segments)
                    pt1 = (seg.lane_boundaries[2].pt_a + seg.lane_boundaries[3].pt_a) / 2
                    pt2 = (seg.lane_boundaries[2].pt_b + seg.lane_boundaries[3].pt_b) / 2
                    tangent = get_tangent(pt1, pt2)
                    temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                    push!(route, temp)
                    else
                        pt1 = (seg.lane_boundaries[1].pt_a + seg.lane_boundaries[2].pt_a) / 2
                        pt2 = (seg.lane_boundaries[1].pt_b + seg.lane_boundaries[2].pt_b) / 2
                        tangent = get_tangent(pt1, pt2)
                        temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                        push!(route, temp)
                    end
                        
                end

            elseif seg.lane_boundaries[1].curvature == 0.0

                pt1 = (seg.lane_boundaries[1].pt_a + seg.lane_boundaries[2].pt_a) / 2
                pt2 = (seg.lane_boundaries[1].pt_b + seg.lane_boundaries[2].pt_b) / 2
                tangent = get_tangent(pt1, pt2)
                # println("直道")
                # println(seg.id)
                temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                push!(route, temp)
            else

                # println("弯道")
                # println(seg.lane_boundaries)

                temps = generate_mid_polyline(seg.lane_boundaries[1], seg.lane_boundaries[2], 5)
                append!(route, temps)
            end
        end
    end
    println("lane segments finish")
    return route
end

function signed_distance(segments, point)
    println("sd init")
    return signed_distance_index(segments, point)[1]
end

function signed_distance_index(segments, point)
    println("sd start")
    num_segments = length(segments)
    sd = zeros(num_segments)

    # distance for normal segments
    for i in 1:num_segments
        p1_p_vector = point - segments[i].pt1
        p2_p_vector = point - segments[i].pt2
        perp_dis = segments[i].normal'*p1_p_vector
        if perp_dis == 0
            if segments[i].tangent' * p1_p_vector <= 0 
                sd[i] = norm(p1_p_vector)
            elseif segments[i].tangent' * p2_p_vector >= 0 
                sd[i] = norm(p2_p_vector)
            else
                sd[i] = 0
            end
        else
            sign = perp_dis/abs(perp_dis)
            if segments[i].tangent' * p1_p_vector <= 0
                sd[i] = norm(p1_p_vector) * sign
            elseif segments[i].tangent' * p2_p_vector >= 0
                sd[i] = norm(p2_p_vector) * sign
            else
                sd[i] = perp_dis
            end
        end
    end

    min_abs_index = argmin(abs.(sd))
    println(min_abs_index)
    return sd[min_abs_index], min_abs_index
end

function collision_constraint(X1, X2, size1, size2)
    # println("collision start")
    # 安全缓冲距离
    buffer = 5
    
    # 计算两个长方体的对角线的一半作为碰撞检测的半径
    radius1 = sqrt((size1[1]/2)^2 + (size1[2]/2)^2)
    radius2 = sqrt((size2[1]/2)^2 + (size2[2]/2)^2)
    
    # 计算两个车辆中心点之间的距离的平方
    distance_squared = (X1[1:2] - X2[1:2])' * (X1[1:2] - X2[1:2])
    
    # 计算碰撞约束条件，确保两车之间至少保持足够的间距
    collision_constraint_value = distance_squared - (radius1 + radius2 + buffer)^2
    # println("collision finish")
    return collision_constraint_value
end


function should_stop(latest_position, segment, flag; speed = 2.5, timestamp = 0.4)
    # 获取用于导航判断的车道边界
    boundary = segment.lane_boundaries[1]  # 假定第一个边界用于导航判断

    # 判断车道边界是否垂直（基于x坐标是否相同）
    is_vertical = boundary.pt_a[1] == boundary.pt_b[1]
    is_left = is_vertical ? (boundary.pt_b[2] - boundary.pt_a[2]) : (boundary.pt_b[1] - boundary.pt_a[1])
    get_target_speed = is_left/abs(is_left)*timestamp*speed
    # 计算目标位置，假定你要在边界的2/3处进行停车判断
    target_pos = is_vertical ? (boundary.pt_b[2] - get_target_speed) : (boundary.pt_b[1] - get_target_speed)
    current_pos = is_vertical ? latest_position[2] : latest_position[1]
    # println("im here")
    # 检查当前位置是否已经达到或超过了目标位置
    # 检查当前的 lane_types 是否包含停车标志，并且边界的曲率为0
    # try
    println(segment.id)
    println(segment.lane_types)
    # catch e
    #     println(e)
    # end
    if segment.lane_types[1] == VehicleSim.stop_sign
        # println("stop sign")
        # println(current_pos)
        # println(target_pos)
        judgement = (current_pos - target_pos) * is_left/abs(is_left)
        # println(judgement)
    if judgement > 0
        if flag == 0
            println("stop")
            return 0, 1  # 返回0表示需要停车，1表示更改为停车状态
        elseif flag == 1
        # 如果已经停车，检查是否可以开始行驶（例如，可以设置一个条件，如停车后等待一定时间）
        # 这里需要你定义何时应该从停车状态恢复行驶
            println("start")
            return 1, 0  # 返回1表示恢复行驶，0表示更改为非停车状态
        end
    end
end
    return 1, flag  # -1 表示不更改速度，保持当前状态
end

function constant_velocity_prediction(x, timestep, u)
    # println("predict start")
    X = x
    X = evolve_state(x, u, timestep)
    # println("predict finish")
    X
end

"""
The physics model used for motion planning purposes.
Returns X[k] when inputs are X[k-1] and U[k]. 
Uses a slightly different vehicle model than presented in class for technical reasons.
"""
function evolve_state(X, U, Δ)
    # println("evolve start")
    V = X[3] + Δ * U[1] 
    θ = X[4] + Δ * U[2]
    X + Δ * [V*cos(θ), V*sin(θ), U[1], U[2]]
end

function calculate_front_position(x_c, y_c, theta, L)
    x_front = x_c + (L / 2) * cos(theta)
    y_front = y_c + (L / 2) * sin(theta)
    return (x_front, y_front)
end
function is_ahead(car1_position, car1_orientation, car2_position, car2_orientation)
    # 计算两车的位置向量
    direction_to_car2 = car2_position - car1_position

    # 将朝向角度转换为向量
    car1_heading_vector = [cos(car1_orientation), sin(car1_orientation)]
    car2_heading_vector = [cos(car2_orientation), sin(car2_orientation)]

    # 计算朝向向量的点积
    dot_product = dot(car1_heading_vector, car2_heading_vector)

    # 判断朝向是否相反
    if dot_product < -0.9 # 容忍小的误差
        return 1
    elseif dot_product > 0.9 # 朝向大致相同
        # 计算车1到车2的方向与车1的朝向的点积
        if dot(direction_to_car2, car1_heading_vector) > 0
            return 1
        end
    end

    return 0
end

function pure_pursuit(v1, v2, v3, segments; ls = 2.0, max_vel = 10.0, timestamp = 0.2) #ls = lookahead time

    # setup variables if you need
    # while true
        # sleep(0.1)
        # fetch(stop_ch) && return # check if this task should end (since it runs on a different thread than the main loop)
        # x = fetch(state_ch) # get latest simulation state
        # p1, p2, vx, vy, v3, θ = x # unpack
        u = zeros(2) # ax, ay, az, w
        p1 = calculate_front_position(v1.position[1], v1.position[2], quaternion_to_angle_z(v1.orientation), v1.size[1])
        p2 = calculate_front_position(v2.position[1], v2.position[2], quaternion_to_angle_z(v2.orientation), v2.size[1])
        p3 = calculate_front_position(v3.position[1], v3.position[2], quaternion_to_angle_z(v3.orientation), v3.size[1])
        println("p1")
        println(p1)
        x1 = [p1[1], p1[2], sqrt(v1.velocity[1]^2 + v1.velocity[2]^2), quaternion_to_angle_z(v1.orientation)]
        x2 = [p2[1], p2[2], sqrt(v2.velocity[1]^2 + v2.velocity[2]^2), quaternion_to_angle_z(v2.orientation)]
        x3 = [p3[1], p3[2], sqrt(v3.velocity[1]^2 + v3.velocity[2]^2), quaternion_to_angle_z(v3.orientation)]
        # stanley controller
        # println("car position")
        # println(x1)
        sd, index = signed_distance_index(segments, [v1.position[1], v1.position[2]])
        # println("get sd success")
        println(sd)

        v = sqrt(v1.velocity[1]^2 + v1.velocity[2]^2)
        u[1] = 1.0
        u[2] = 0.0
        u2 = zeros(2)
        u3 = zeros(2)
        u2[2] = v2.steering_angle
        u3[2] = v3.steering_angle
        center_x1 = [v1.position[1], v1.position[2], sqrt(v1.velocity[1]^2 + v1.velocity[2]^2), quaternion_to_angle_z(v1.orientation)]
        center_x2 = [v2.position[1], v2.position[2], sqrt(v2.velocity[1]^2 + v2.velocity[2]^2), quaternion_to_angle_z(v2.orientation)]
        center_x3 = [v3.position[1], v3.position[2], sqrt(v3.velocity[1]^2 + v3.velocity[2]^2), quaternion_to_angle_z(v3.orientation)]
        predict_x1 = constant_velocity_prediction(center_x1, timestamp, u)
        predict_x2 = constant_velocity_prediction(center_x2, timestamp, u2)
        predict_x3 = constant_velocity_prediction(center_x3, timestamp, u3)
        alpha = 0.5
        while collision_constraint(predict_x1, predict_x2, v1.size, v2.size)<0 && u[1]+v>0.0 && is_ahead(v1.position[1:2], x1[4],v2.position[1:2], x2[4])==1
            println("is not ahead")
            println("reduce")
            u[1] -= alpha
            predict_x1 = constant_velocity_prediction(center_x1, timestamp, u)
        end
        while collision_constraint(predict_x1, predict_x3, v1.size, v3.size)<0 && u[1]+v>0.0 && is_ahead(v1.position[1:2], x1[4],v3.position[1:2], x3[4])==1
            u[1] -= alpha
            predict_x1 = constant_velocity_prediction(center_x1, timestamp, u)
        end
        if u[1] + v > max_vel
            u[1] = max_vel - v
        end
        path_tangent = segments[index].tangent
        path_normal = segments[index].normal
        # println("road heading")
        # println(path_tangent)
        θ = quaternion_to_angle_z(v1.orientation)
        # println("car heading")
        # println(θ)
        vehicle_direction = [cos(θ), sin(θ)]
        # println("direction")
        # println(vehicle_direction)
        theta_error_value = acos(path_tangent'*vehicle_direction/norm(path_tangent)/norm(vehicle_direction))
        if path_normal'*vehicle_direction >= 0
            theta_error_sign = 1
        else
            theta_error_sign = -1
        end
        theta_error = theta_error_sign*theta_error_value
        u[2] = -theta_error - atan(1*sd/(v+u[1]))
        if u[2]>=0.9 
            u[2]=0.9
        end
        if u[2]<=-0.9
            u[2]=-0.9
        end
        # println("前轮转向")
        # println(u[2])
        # for speed, u[1] = target_vel
        """
        if v > 3.1
            u[2] = -0.5
        else
            u[2] = 0
        end
        """

        # println(u)
        # take!(control_ch) # clear old control
        # put!(control_ch, u) # put new control on the control channel
        return u
    # end
end

function is_point_in_rectangle(pt1, pt2, pt)
    (x1, y1) = pt1
    (x2, y2) = pt2
    (x, y, z) = pt

    minX = min(x1, x2)
    maxX = max(x1, x2)
    minY = min(y1, y2)
    maxY = max(y1, y2)

    return minX <= x <= maxX && minY <= y <= maxY
end

function is_point_in_arc(point, center, radius, pt_a, pt_b)
    # Check if the point is within the circle defined by radius with tolerance
    distance = norm(point - center)
    if isapprox(distance, radius; atol=1e-6)
        # Further check if the angle of the point is between angles of pt_a and pt_b
        angle_a = atan2(pt_a[2] - center[2], pt_a[1] - center[1])
        angle_b = atan2(pt_b[2] - center[2], pt_b[1] - center[1])
        angle_point = atan2(point[2] - center[2], point[1] - center[1])
        return (angle_a ≤ angle_point ≤ angle_b) || (angle_b ≤ angle_point ≤ angle_a)
    end
    return false
end

function estimate_circle_center(pt_a, pt_b, curvature)
    mid_point = (pt_a + pt_b) / 2
    direction = pt_b - pt_a
    normal = SVector(-direction[2], direction[1])  # Rotate 90 degrees
    radius = 1 / abs(curvature)
    center = mid_point + normal * (curvature > 0 ? 1 : -1) * radius / norm(normal)
    return center, radius
end

function is_inside_segment(car_position, segment)
    within_boundary = false
    if length(segment.lane_types)>1
        # println("bcnm")
        # println(segment)
        if segment.lane_types[2]==VehicleSim.loading_zone 
            # println("cnb")
        if segment.lane_boundaries[2].pt_a == segment.lane_boundaries[3].pt_a
            # println("cblnb")

            within_boundary = is_point_in_rectangle(segment.lane_boundaries[1].pt_a, segment.lane_boundaries[3].pt_b, car_position)

        elseif segment.lane_boundaries[2].pt_b == segment.lane_boundaries[3].pt_b
            # println("ji")
            within_boundary = is_point_in_rectangle(segment.lane_boundaries[3].pt_a, segment.lane_boundaries[1].pt_b, car_position)
        else
            # println(segment)
            within_boundary = is_point_in_rectangle(segment.lane_boundaries[1].pt_a, segment.lane_boundaries[3].pt_b, car_position)
        end
    end
    elseif segment.lane_boundaries[1].curvature==0.0
        # println("cnm")
        within_boundary = is_point_in_rectangle(segment.lane_boundaries[1].pt_a, segment.lane_boundaries[2].pt_b, car_position)
    else
        # println("csnm")
        point = SVector{2, Float64}(car_position[1:2])
        c1, r1 = estimate_circle_center(segment.lane_boundaries[1].pt_a, segment.lane_boundaries[1].pt_b, segment.lane_boundaries[1].curvature)
        c2, r2 = estimate_circle_center(segment.lane_boundaries[2].pt_a, segment.lane_boundaries[2].pt_b, segment.lane_boundaries[2].curvature)
        within_boundary = !is_point_in_arc(point, c1, r1, segment.lane_boundaries[1].pt_a, segment.lane_boundaries[1].pt_b)&&is_point_in_arc(point, c2, r2, segment.lane_boundaries[2].pt_a, segment.lane_boundaries[2].pt_b)
    end
    
    return within_boundary
end

function has_passed_halfway(point, diag1, diag2)
    # 计算矩形的中点坐标
    passed_halfway = false
    midpoint = ((diag1[1] + diag2[1]) / 2, (diag1[2] + diag2[2]) / 2)
    
    # 确定长边方向
    width = abs(diag1[1] - diag2[1])
    height = abs(diag1[2] - diag2[2])
    if is_point_in_rectangle(diag1, diag2, point)
    # 根据长边方向比较点的位置
    if width >= height
        # 长边沿 x 轴
        longer_axis_value = point[1] - midpoint[1]
        direction = diag2[1] - diag1[1]
    else
        # 长边沿 y 轴
        longer_axis_value = point[2] - midpoint[2]
        direction = diag2[2] - diag1[2]
    end
    passed_halfway = (longer_axis_value * direction) > 0
    end
    
    # 根据方向判断是否过半


    return passed_halfway
end


function decision_making(vehicle_channel, 
    gt_channel, 
    perception_state_channel, 
    map, 
    target_segment_channel, 
    socket)
# do some setup
flag = 0
println("motion start")
# println(map)
route_flag = 1
segments = []   
vehicle_id = 0
get_vehicle = 0

while true

    # sleep(0.2)
try
    if get_vehicle == 0
        vehicle_id = fetch(vehicle_channel)
        get_vehicle =1
    end
    latest_localization_state = take!(gt_channel)
    latest_perception_state = []
    while latest_localization_state.vehicle_id!=vehicle_id
        temp = MyPerceptionType(
            latest_localization_state.time,                        # time
            latest_localization_state.vehicle_id,                          # vehicle_id
            latest_localization_state.position, # position
            latest_localization_state.orientation, # orientation (quaternion)
            latest_localization_state.velocity, # velocity
            calculate_steering_angle(latest_localization_state.angular_velocity[3], norm(latest_localization_state.velocity[1],latest_localization_state.velocity[2]), latest_localization_state.size[1]),                        # steering_angle
            latest_localization_state.size  # size
        )
        push!(latest_perception_state, temp)
        println("csnm")
        latest_localization_state = take!(gt_channel)
    end
    println("v id")
    println(latest_localization_state.vehicle_id)


    # println("gt")
    # println(latest_localization_state)
    println("perception")
    # println(latest_perception_state)

    # println("target1")
    target_segment = fetch(target_segment_channel)
    # println(target_segment)
    current_segment = []
    v1 = latest_localization_state
    # println(v1)
    front_position = calculate_front_position(latest_localization_state.position[1], latest_localization_state.position[2], quaternion_to_angle_z(latest_localization_state.orientation), latest_localization_state.size[1])
    car_position = [front_position[1], front_position[2], 0]
    for map_segment in map
        if is_inside_segment(car_position, map_segment[2])
            # println(map_segment[1])
            push!(current_segment, map_segment[2])
        end

    end
    if length(current_segment) == 0 
        println("Error: car not inside a segment")
    end

    target_segment_id = target_segment.id
    println("target_segment")
    println(target_segment_id)

    if has_passed_halfway(v1.position, target_segment.lane_boundaries[2].pt_a, target_segment.lane_boundaries[3].pt_b)
        println("reached")
        cmd = (0.0, 0.0, true)
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