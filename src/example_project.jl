@enum LaneTypes begin
    standard
    loading_zone
    intersection
    stop_sign
end

@enum Direction begin
    north
    east
    south
    west
end

function opposite(direction::Direction)
    direction == north && return south
    direction == east && return west
    direction == south && return north
    direction == west && return east
end

struct LaneBoundary
    pt_a::SVector{2,Float64}
    pt_b::SVector{2,Float64}
    curvature::Float64
    hard_boundary::Bool
    visualized::Bool
end

function lane_boundary(pt_a, pt_b, hard, vis, left=true)
    dx = abs(pt_a[1] - pt_b[1])
    dy = abs(pt_a[2] - pt_b[2])
    if isapprox(dx - dy, 0; atol=1e-6)
        sign = left ? 1.0 : -1.0
        LaneBoundary(pt_a, pt_b, sign / dx, hard, vis)
    else
        LaneBoundary(pt_a, pt_b, 0.0, hard, vis)
    end
end

"""
lane_boundaries are in order from left to right
"""
# mutable struct RoadSegment
#     id::Int
#     lane_boundaries::Vector{LaneBoundary}
#     lane_types::Vector{LaneTypes}
#     speed_limit::Float64
#     children::Vector{Int}
# end

struct MyLocalizationType
    time::Float64
    vehicle_id::Int
    position::SVector{3, Float64} # position of center of vehicle
    orientation::SVector{4, Float64} # represented as quaternion
    velocity::SVector{3, Float64}
    angular_velocity::SVector{3, Float64} # angular velocity around x,y,z axes
    size::SVector{3, Float64} # length, width, height of 3d bounding box centered at (position/orientation)
    map_segment::VehicleSim.RoadSegment
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

struct MySegment
    pt_a::SVector{2, Float64}
    pt_b::SVector{2, Float64}
    tangent::SVector{2, Float64}
    normal::SVector{2, Float64}
end

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

function wheel_angle_to_angular_velocity(V, theta, L)
    # 计算xy平面上的速度大小
    V_magnitude = sqrt(V[1]^2 + V[2]^2)
    
    # 根据转向角计算角速度
    if theta == 0 || V_magnitude == 0
        return 0.0  # No turning or no movement, no angular velocity
    end
    R = L / tan(theta)  # Turn radius
    omega = V_magnitude / R  # Angular velocity around z-axis
    
    return omega
end

function calculate_steering_angle(ω, v, L) 
    # 提取三维速度向量中的x和y分量
    v_x, v_y = v[1], v[2]
    
    # 计算地面平面上的速度大小
    v = norm([v_x, v_y])
    
    if v == 0
        # 当车辆地面速度为0时，转向角度未定义或可能非常大
        return Inf  # 或者可以定义其他处理方式
    else
        # 计算转向角度
        δ = atan(ω * L / v)
        return δ  # 返回转向角，单位为弧度
    end
end

function heading_to_quaternion(heading::Float64)
    # Convert heading angle to quaternion representation
    # Assume rotation around vertical (z) axis
    # Construct quaternion [cos(θ/2), 0, 0, sin(θ/2)]
    θ = deg2rad(heading)  # Convert heading angle to radians
    q = SVector(cos(θ / 2), 0.0, 0.0, sin(θ / 2))
    return q
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

function perp(x)
    [-x[2], x[1]]
end

function get_tangent(pt_a, pt_b)
    tangent = pt_b - pt_a
    tangent ./= norm(tangent)
    return tangent
end

function get_normal(tangent)
    return perp(tangent)
end

function approximate_curv_boundary_with_polyline(pt_a, pt_b, curvature, num_points)
    # 计算半径R
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
    start_angle = atan2(pt_a[2] - circle_center[2], pt_a[1] - circle_center[1])
    end_angle = atan2(pt_b[2] - circle_center[2], pt_b[1] - circle_center[1])

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

    return segments
end

function get_lane_segments(segments, flag)
    route = Vector{MySegment}()
    num_points = 5
    if flag == 1
        for (index, seg) in enumerate(segments)
            if seg.lane_types[2] == loading_zone
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
                temps = approximate_curv_boundary_with_polyline(pt1, pt2, seg.lane_boundaries[1].curvature, num_points)
                append!(route, temps)
            end
        end
    elseif flag == 2
        for (index, seg) in enumerate(segments)
            if seg.lane_types[2] == loading_zone
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
            if seg.lane_types[2] == loading_zone
                if seg.lane_boundaries[2].pt_a == seg.lane_boundaries[3].pt_a
                        #我就在这
                        pt1 = (seg.lane_boundaries[1].pt_a + seg.lane_boundaries[2].pt_a) / 2
                        pt2 = (seg.lane_boundaries[2].pt_b + seg.lane_boundaries[3].pt_b) / 2
                        tangent = get_tangent(pt1, pt2)
                        temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                        push!(route, temp)
                elseif seg.lane_boundaries[2].pt_b == seg.lane_boundaries[3].pt_b
                    #loading_zone出口
                    pt1 = (seg.lane_boundaries[2].pt_a + seg.lane_boundaries[3].pt_a) / 2
                    pt2 = (seg.lane_boundaries[1].pt_b + seg.lane_boundaries[2].pt_b) / 2
                    tangent = get_tangent(pt1, pt2)
                    temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                    push!(route, temp)
                else
                    pt1 = (seg.lane_boundaries[2].pt_a + seg.lane_boundaries[3].pt_a) / 2
                    pt2 = (seg.lane_boundaries[2].pt_b + seg.lane_boundaries[3].pt_b) / 2
                    tangent = get_tangent(pt1, pt2)
                    temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                    push!(route, temp)
                end

            elseif seg.lane_boundaries[1].curvature == 0.0
                pt1 = (seg.lane_boundaries[1].pt_a + seg.lane_boundaries[2].pt_a) / 2
                pt2 = (seg.lane_boundaries[1].pt_b + seg.lane_boundaries[2].pt_b) / 2
                tangent = get_tangent(pt1, pt2)
                temp = MySegment(pt1, pt2, tangent, get_normal(tangent))
                push!(route, temp)
            else
                pt1 = (seg.lane_boundaries[1].pt_a + seg.lane_boundaries[2].pt_a) / 2
                pt2 = (seg.lane_boundaries[1].pt_b + seg.lane_boundaries[2].pt_b) / 2
                temps = approximate_curv_boundary_with_polyline(pt1, pt2, seg.lane_boundaries[1].curvature, num_points)
                append!(route, temps)
            end
        end
    end
    return route
end


function wrap(X, lane_length)
    X_wrapped = copy(X)
    # if X_wrapped[1] > lane_length
    #     X_wrapped[1] -= lane_length
    # end
    X_wrapped
end

function generate_trajectory(latest_localization_state, v2, v3, segments, callbacks, trajectory_length, timestep)
    println("start generate")
    v1 = latest_localization_state
    X1 = [v1.position[1]; v1.position[2]; v1.velocity[1]; v1.velocity[2]; v1.velocity[3]; quaternion_to_angle_z(v1.orientation)]
    X2 = [v2.position[1]; v2.position[2]; v2.velocity[1]; v2.velocity[2]; v2.velocity[3]; quaternion_to_angle_z(v2.orientation)]
    X3 = [v3.position[1]; v3.position[2]; v3.velocity[1]; v3.velocity[2]; v3.velocity[3]; quaternion_to_angle_z(v3.orientation)]
    pt_la = Vector{Float64}()
    pt_lb = Vector{Float64}()
    pt_ra = Vector{Float64}()
    pt_rb = Vector{Float64}()
    l_segments = get_lane_segments(segments, 1)
    r_segments = get_lane_segments(segments, 2)
    for segment in l_segments
        push!(pt_la, l_segments.pt1[1])
        push!(pt_lb, l_segments.pt1[2])
        if length(pt_la)==10
            break
        end
    end
    for segment in r_segments
        push!(pt_ra, r_segments.pt1[1])
        push!(pt_rb, r_segments.pt1[2])
        if length(pt_ra)==10
            break
        end
    end
    if length(pt_la) < 10
        append!(pt_la, fill(0.0, 10 - length(pt_la)))
    end
    if length(pt_lb) < 10
        append!(pt_lb, fill(0.0, 10 - length(pt_lb)))
    end
    if length(pt_ra) < 10
        append!(pt_ra, fill(0.0, 10 - length(pt_ra)))
    end
    if length(pt_la) < 10
        append!(pt_rb, fill(0.0, 10 - length(pt_rb)))
    end
    # refine callbacks with current values of parameters / problem inputs
    wrapper_f = function(z)
        callbacks.full_cost_fn(z, X1, X2, X3, v1.size, v2.size, v3.size, pt_la, pt_lb, pt_ra, pt_rb)
    end
    wrapper_grad_f = function(z, grad)
        callbacks.full_cost_grad_fn(grad, z, X1, X2, X3, v1.size, v2.size, v3.size, pt_la, pt_lb, pt_ra, pt_rb)
    end
    wrapper_con = function(z, con)
        callbacks.full_constraint_fn(con, z, X1, X2, X3, v1.size, v2.size, v3.size, pt_la, pt_lb, pt_ra, pt_rb)
    end
    wrapper_con_jac = function(z, rows, cols, vals)
        if isnothing(vals)
            rows .= callbacks.full_constraint_jac_triplet.jac_rows
            cols .= callbacks.full_constraint_jac_triplet.jac_cols
        else
            callbacks.full_constraint_jac_triplet.full_constraint_jac_vals_fn(vals, z, X1, X2, X3, v1.size, v2.size, v3.size, pt_la, pt_lb, pt_ra, pt_rb)
        end
        nothing
    end
    wrapper_lag_hess = function(z, rows, cols, cost_scaling, λ, vals)
        if isnothing(vals)
            rows .= callbacks.full_lag_hess_triplet.hess_rows
            cols .= callbacks.full_lag_hess_triplet.hess_cols
        else
            callbacks.full_lag_hess_triplet.full_hess_vals_fn(vals, z, X1, X2, X3, v1.size, v2.size, v3.size, pt_la, pt_lb, pt_ra, pt_rb, λ, cost_scaling)
        end
        nothing
    end

    n = trajectory_length*6
    m = length(callbacks.constraints_lb)
    prob = Ipopt.CreateIpoptProblem(
        n,
        fill(-Inf, n),
        fill(Inf, n),
        length(callbacks.constraints_lb),
        callbacks.constraints_lb,
        callbacks.constraints_ub,
        length(callbacks.full_constraint_jac_triplet.jac_rows),
        length(callbacks.full_lag_hess_triplet.hess_rows),
        wrapper_f,
        wrapper_con,
        wrapper_grad_f,
        wrapper_con_jac,
        wrapper_lag_hess
    )

    controls = repeat([zeros(4),], trajectory_length)
    #states = constant_velocity_prediction(X1, trajectory_length, timestep)
    states = repeat([X1,], trajectory_length)
    zinit = compose_trajectory(states, controls)
    prob.x = zinit

    Ipopt.AddIpoptIntOption(prob, "print_level", 0)
    status = Ipopt.IpoptSolve(prob)

    if status != 0
        @warn "Problem not cleanly solved. IPOPT status is $(status)."
    end
    states, controls = decompose_trajectory(prob.x)
    (; states, controls, status)
end

# function get_route(
#     latest_localization_state, 
#     latest_perception_state, 
#     map)
#     # 假设这里是一些逻辑来计算路线
#     result = Vector{RoadSegment}()
#     return result  # 创建并返回一个RoadSegment类型的空向量
# end


function should_stop(latest_position, segment, flag)
    # 获取用于导航判断的车道边界
    boundary = segment.lane_boundaries[1]  # 假定第一个边界用于导航判断

    # 判断车道边界是否垂直（基于x坐标是否相同）
    is_vertical = boundary.pt_a[1] == boundary.pt_b[1]

    # 计算目标位置，假定你要在边界的2/3处进行停车判断
    target_pos = is_vertical ? (2/3 * (boundary.pt_a[2] + boundary.pt_b[2])) : (2/3 * (boundary.pt_a[1] + boundary.pt_b[1]))
    current_pos = is_vertical ? latest_position[2] : latest_position[1]

    # 检查当前位置是否已经达到或超过了目标位置
    # 检查当前的 lane_types 是否包含停车标志，并且边界的曲率为0
    if current_pos >= target_pos && "stop_sign" in segment.lane_types && boundary.curvature == 0
        if flag == 0
            return 0, 1  # 返回0表示需要停车，1表示更改为停车状态
        end
    elseif flag == 1
        # 如果已经停车，检查是否可以开始行驶（例如，可以设置一个条件，如停车后等待一定时间）
        # 这里需要你定义何时应该从停车状态恢复行驶
        return 1, 0  # 返回1表示恢复行驶，0表示更改为非停车状态
    end
    return 1, flag  # -1 表示不更改速度，保持当前状态
end



# function decision_making(localization_state_state, 
#         perception_state_channel, 
#         map, 
#         target_segment_channel,
#         socket)
#     # do some setup
#     flag = 0

#     while true
#         latest_localization_state = fetch(localization_state_channel)
#         latest_perception_state = fetch(perception_state_channel)
#         target_segment_id = fetch(target_segment_channel)
#         segments = get_route(latest_localization_state, latest_perception_state, map)
#         dists = [Inf; [norm(v.position[1:2]-latest_localization_state.position[1:2]) for v in latest_perception_state]]
#         closest = partialsortperm(dists, 1:2)
#         v2 = latest_perception_state[closest[1]]
#         v3 = latest_perception_state[closest[2]]
#         max_vel = Inf
#         for segment in segments
#             if segment.speed_limit <= max_vel
#                 max_vel = segment.speed_limit
#             end
#         end
#         now_segment = latest_localization_state.map_segment
#         max_vel, flag = should_stop(latest_localization_state.position, now_segment, flag)
#         max_vel != -1 && (max_vel = 0)

#         callbacks = create_callback_generator(trajectory_length=10, timestep=0.2, R = Diagonal([0.1, 0.1, 0.1, 0.5]), max_vel= max_vel, angles = [v2.steering_angle, v3.steering_angle])
#         trajectory = generate_trajectory(latest_localization_state, v2, v3, segments, callbacks, trajectory_length=10, timestep=0.2)
#         # figure out what to do ... setup motion planning problem etc
#         target_vel = [trajectory.states[1][3], trajectory.states[1][4], trajectory.states[1][5]]
#         steering_angle = calculate_steering_angle(trajectory.controls[1][4], target_vel, latest_localization_state.size[1])
#         cmd = VehicleCommand(steering_angle, target_vel, true)
#         serialize(socket, cmd)
#     end
# end

function is_point_in_rectangle(pt1::SVector{2, Float64}, pt2::SVector{2, Float64}, pt::SVector{3, Float64})
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
    within_boundaries = false
    # println(segment)
    
    # Check if the car's position is within each lane boundary
    # for boundary in segment.lane_boundaries
    #     # Determine if the car's latitude and longitude fall within the boundary
    #     within_boundary = (boundary.pt_a[2] <= car_position[2] <= boundary.pt_b[2] ||
    #                        boundary.pt_b[2] <= car_position[2] <= boundary.pt_a[2]) &&
    #                       (boundary.pt_a[1] <= car_position[1] <= boundary.pt_b[1] ||
    #                        boundary.pt_b[1] <= car_position[1] <= boundary.pt_a[1])
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
        within_boundaries = is_point_in_rectangle(segment.lane_boundaries[1].pt_a, segment.lane_boundaries[2].pt_b, car_position)
    else
        # println("csnm")
        point = SVector{2, Float64}(car_position[1:2])
        c1, r1 = estimate_circle_center(segment.lane_boundaries[1].pt_a, segment.lane_boundaries[1].pt_b, segment.lane_boundaries[1].curvature)
        c2, r2 = estimate_circle_center(segment.lane_boundaries[2].pt_a, segment.lane_boundaries[2].pt_b, segment.lane_boundaries[2].curvature)
        within_boundaries = !is_point_in_arc(point, c1, r1, segment.lane_boundaries[1].pt_a, segment.lane_boundaries[1].pt_b)&&is_point_in_arc(point, c2, r2, segment.lane_boundaries[2].pt_a, segment.lane_boundaries[2].pt_b)
    end
    
    return within_boundaries
end

function decision_making(gt_channel, 
    perception_state_channel, 
    map, 
    target_segment_channel, 
    socket)
# do some setup
flag = 0
println("motion start")
# println(map)

while true
    sleep(0.2)
    latest_localization_state = fetch(gt_channel)
    # println("gt")
    # println(latest_localization_state)
    # latest_perception_state = fetch(perception_state_channel)
    latest_perception_state = []
    # println("target1")
    target_segment = fetch(target_segment_channel)
    # println(target_segment)
    current_segment = []
    for map_segment in map
        if is_inside_segment(latest_localization_state.position, map_segment[2])
            println(map_segment[1])
            push!(current_segment, map_segment[2])
        end
    end
    if length(current_segment) == 0 
        println("Error: car not inside a segment")
    end
    # start_segment_id = current_segment.id
    println("start_segment")
    println(current_segment[1].id)
    target_segment_id = target_segment.id
    println("target_segment")
    println(target_segment_id)
    # segments = get_route(map, start_segment_id, target_segment_id)
    # println(segments)
    segments = []
    push!(segments, map[32])
    push!(segments, map[30])
    push!(segments, map[28])
    # # println(segments)
    # if length(latest_perception_state)==0
    #     v2 = MyPerceptionType(
    #         0.0,                        # time
    #         0,                          # vehicle_id
    #         SVector{3, Float64}(0.0, 0.0, 0.0), # position
    #         SVector{4, Float64}(0.0, 0.0, 0.0, 0.0), # orientation (quaternion)
    #         SVector{3, Float64}(Inf, Inf, 0.0), # velocity
    #         0.0,                        # steering_angle
    #         SVector{3, Float64}(0.0, 0.0, 0.0)  # size
    #     )
    #     v3 = MyPerceptionType(
    #         0.0,                        # time
    #         0,                          # vehicle_id
    #         SVector{3, Float64}(0.0, 0.0, 0.0), # position
    #         SVector{4, Float64}(0.0, 0.0, 0.0, 0.0), # orientation (quaternion)
    #         SVector{3, Float64}(Inf, Inf, 0.0), # velocity
    #         0.0,                        # steering_angle
    #         SVector{3, Float64}(0.0, 0.0, 0.0)  # size
    #     )
    # elseif length(latest_perception_state)==0       
    #     v2 = MyPerceptionType(
    #         0.0,                        # time
    #         0,                          # vehicle_id
    #         SVector{3, Float64}(0.0, 0.0, 0.0), # position
    #         SVector{4, Float64}(0.0, 0.0, 0.0, 0.0), # orientation (quaternion)
    #         SVector{3, Float64}(Inf, Inf, 0.0), # velocity
    #         0.0,                        # steering_angle
    #         SVector{3, Float64}(0.0, 0.0, 0.0)  # size
    #     )
    #     v3 = latest_perception_state[1]
    # else   
    #     dists = [Inf; [norm(v.position[1:2]-latest_localization_state.position[1:2]) for v in latest_perception_state]]
    #     closest = partialsortperm(dists, 1:2)
    #     v2 = latest_perception_state[closest[1]]
    #     v3 = latest_perception_state[closest[2]]
    # end
    # max_vel = Inf
    # for segment in segments
    #     if segment.speed_limit <= max_vel
    #         max_vel = segment.speed_limit
    #     end
    # end
    # now_segment = segments[1]
    # println("cnm")
    # stop_sign, flag = should_stop(latest_localization_state.position, now_segment, flag)
    # max_vel =  stop_sign==1 ? max_vel : 0
    # println(max_vel)
    # try
    # callbacks = create_callback_generator(trajectory_length=10, timestep=0.2, R = Diagonal([0.1, 0.1, 0.1, 0.5]), max_vel= max_vel, angles = [v2.steering_angle, v3.steering_angle])
    # catch e
    #     println(e)
    # end
    # trajectory = generate_trajectory(latest_localization_state, v2, v3, segments, callbacks, trajectory_length=10, timestep=0.2)
    # # figure out what to do ... setup motion planning problem etc
    # println(trajectory)
    # target_vel = [trajectory.states[2][3], trajectory.states[2][4], trajectory.states[2][5]]
    # steering_angle = calculate_steering_angle(trajectory.controls[1][4], target_vel, latest_localization_state.size[1])
    cmd = VehicleCommand(steering_angle, target_vel, true)
    serialize(socket, cmd)
end
end

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
    target_segment_channel = Channel{VehicleSim.RoadSegment}(32)

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

    # @async localize(gps_channel, imu_channel, localization_state_channel)
    # @async perception(cam_channel, localization_state_channel, perception_state_channel)
    # @async decision_making(localization_state_channel, perception_state_channel, map, target_segment_channel. socket)
    @async decision_making(gt_channel, perception_state_channel, map_segments, target_segment_channel, socket)
end



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

function dijkstra(graph, start_segment_id, target_segment_id)
    distances = Dict{Int, Float64}()
    previous = Dict{Int, Int}()
    pq = PriorityQueue()

    for node_id in keys(graph.edges)
        distances[node_id] = Inf
        enqueue!(pq, node_id, Inf)
    end
    distances[start_segment_id] = 0
    update!(pq, start_segment_id, 0)

    while !isempty(pq)
        current_id = dequeue!(pq)
        if current_id == target_segment_id || (current_id in graph.pullout_zones && target_segment_idin, graph.pullout_zones)
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
    calculate_segment_length(segment) / 10.0
end

function reconstruct_path(previous, start_segment_id, target_segment_id, map)
    path = []
    segments = []
    current_id = target_segment_id
    while current_id != start_segment_id
        push!(path, current_id)
        current_id = previous[current_id]
        if isnothing(current_id)
            return []  
        end
    end
    push!(path, start_segment_id)
    reverse(path)
    println("path")
    println(path)
    for id in path
        push!(segments, map[id])
    end
    segments
end

function get_route(map, start_segment_id, target_segment_id)
    graph = build_graph(map)
    distances, previous = dijkstra(graph, start_segment_id, target_segment_id)
    segments = reconstruct_path(previous, start_segment_id, target_segment_id, map)
    segments
end

