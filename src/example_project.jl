
function perp_symbolic(x)
    # @variables x[1:2]  # 如果x未定义为符号变量，则在此定义
    return [-x[2], x[1]]  # 返回垂直向量
end

function get_tangent_symbolic(pt_a, pt_b)
    # @variables pt_a[1:2] pt_b[1:2]  # 如果pt_a, pt_b未定义为符号变量，则在此定义
    tangent = pt_b - pt_a
    norm_tangent = sqrt(tangent[1]^2 + tangent[2]^2)  # 符号计算向量的范数
    normalized_tangent = tangent ./ norm_tangent  # 归一化向量
    return normalized_tangent
end

function get_normal_symbolic(tangent)
    return perp_symbolic(tangent)
end


# function signed_distance(segments, point)
#     println("sd init")
#     return signed_distance_index(segments, point)[1]
# end

# function signed_distance_index(segments, point)
#     println("sd start")
#     num_segments = length(segments)
#     sd = zeros(num_segments)

#     # distance for starting ray

#     p_p_vector = point- segments[1].pt1
#     perp_dis = segments[1].normal'*p_p_vector
#     if perp_dis == 0
#         if segments[1].tangent' * p_p_vector >= 0 
#             sd[1] = norm(p_p_vector)
#         else
#             sd[1] = 0
#         end
#     else
#         sign = perp_dis/abs(perp_dis)
#         if segments[1].tangent' * p_p_vector >= 0 
#             sd[1] = norm(p_p_vector) * sign
#         else
#             sd[1] = perp_dis
#         end
#     end

#     # distance for terminal ray
#     p_p_vector = point - segments[num_segments].pt2
#     perp_dis = segments[num_segments].normal'*p_p_vector
#     if perp_dis == 0
#         if segments[num_segments].tangent' * p_p_vector >= 0 
#             sd[num_segments] = 0
#         else
#             sd[num_segments] = norm(p_p_vector)
#         end
#     else
#         sign = perp_dis/abs(perp_dis)
#         if segments[num_segments].tangent' * p_p_vector >= 0 
#             sd[num_segments] = perp_dis
#         else
#             sd[num_segments] = norm(p_p_vector) * sign
#         end
#     end

#     # distance for normal segments
#     for i in 2:num_segments-1
#         p1_p_vector = point - segments[i].pt1
#         p2_p_vector = point - segments[i].pt2
#         perp_dis = segments[i].normal'*p1_p_vector
#         if perp_dis == 0
#             if segments[i].tangent' * p1_p_vector <= 0 
#                 sd[i] = norm(p1_p_vector)
#             elseif segments[i].tangent' * p2_p_vector >= 0 
#                 sd[i] = norm(p2_p_vector)
#             else
#                 sd[i] = 0
#             end
#         else
#             sign = perp_dis/abs(perp_dis)
#             if segments[i].tangent' * p1_p_vector <= 0
#                 sd[i] = norm(p1_p_vector) * sign
#             elseif segments[i].tangent' * p2_p_vector >= 0
#                 sd[i] = norm(p2_p_vector) * sign
#             else
#                 sd[i] = perp_dis
#             end
#         end
#     end

#     min_abs_index = argmin(abs.(sd))
#     return sd[min_abs_index], min_abs_index
# end

function signed_distance(pts, point)
    println("sd start")
    num_pts = length(pts) - 1
    @variables sd[1:num_pts]  # 定义符号数组

    for i in 1:num_pts
        pt1 = pts[i]
        pt2 = pts[i + 1]

        # 计算切线
        tangent = pt2 - pt1
        tangent ./= sqrt(tangent' * tangent)  # 归一化

        # 计算法线
        normal = [-tangent[2], tangent[1]]  # 2D情况的垂直向量

        # 计算点到线段的距离
        p_p_vector = point - pt1
        perp_dis = normal' * p_p_vector
        tangent_dot = tangent' * p_p_vector
        norm_p_p_vector = sqrt(p_p_vector' * p_p_vector)  # 向量范数

        sd[i] = ifelse(perp_dis == 0,
                       ifelse(tangent_dot >= 0, norm_p_p_vector, 0),
                       ifelse(tangent_dot >= 0, norm_p_p_vector * sign(perp_dis), perp_dis))
    end

    # 找到最小绝对距离索引
    pringln("sd finish")
    min_abs_index = argmin(abs.(sd))

    return sd[min_abs_index], min_abs_index
end


function create_callback_generator(;trajectory_length=10, timestep=0.2, R = Diagonal([0.1, 0.1, 0.1, 0.5]), max_vel=10.0, angles=[0.0, 0.0])
    println("callback start")
    X¹, X², X³, size¹, size², size³, pt_la, pt_lb, pt_ra, pt_rb, Z = let
        @variables(X¹[1:6], X²[1:6], X³[1:6], size¹[1:3], size²[1:3], size³[1:3], pt_la[1:10], pt_lb[1:10], pt_ra[1:10], pt_rb[1:10], Z[1:10*trajectory_length]) .|> Symbolics.scalarize
    end
    states, controls = decompose_trajectory(Z)
    all_states = [[X¹,]; states]

    cost_val = sum(stage_cost(x, u, R, pt_la, pt_lb, pt_ra, pt_rb) for (x,u) in zip(states, controls))
    cost_grad = Symbolics.gradient(cost_val, Z)
    constraints_val = Symbolics.Num[]
    constraints_lb = Float64[]
    constraints_ub = Float64[]
    for k in 1:trajectory_length
        # println(k)
        vehicle_2_prediction = constant_velocity_prediction(X², size²[1], angles[1], trajectory_length, timestep)
        vehicle_3_prediction = constant_velocity_prediction(X³, size²[1], angles[2], trajectory_length, timestep)
        append!(constraints_val, all_states[k+1] .- evolve_state(all_states[k], controls[k], timestep))
        append!(constraints_lb, zeros(6))
        append!(constraints_ub, zeros(6))
        append!(constraints_val, lane_constraint(states[k], size¹, pt_la, pt_lb, 0))
        append!(constraints_val, lane_constraint(states[k], size¹, pt_ra, pt_rb, 1))
        append!(constraints_lb, zeros(2))
        append!(constraints_ub, fill(Inf, 2))
        append!(constraints_val, collision_constraint(states[k], vehicle_2_prediction, size¹, size²))
        append!(constraints_val, collision_constraint(states[k], vehicle_3_prediction, size¹, size³))
        append!(constraints_lb, zeros(2))
        append!(constraints_ub, fill(Inf, 2))
        append!(constraints_val, sqrt(states[k][3]^2 + states[k][4]^2))
        append!(constraints_lb, 0.0)
        append!(constraints_ub, max_vel)
        append!(constraints_val, states[k][6])
        append!(constraints_lb, -pi/4)
        append!(constraints_ub, pi/4)
    end
    println("constraints add")

    constraints_jac = Symbolics.sparsejacobian(constraints_val, Z)
    (jac_rows, jac_cols, jac_vals) = findnz(constraints_jac)
    num_constraints = length(constraints_val)
    println("before expression1")
    λ, cost_scaling = let
        @variables(λ[1:num_constraints], cost_scaling) .|> Symbolics.scalarize
    end
    lag = (cost_scaling * cost_val + λ' * constraints_val)
    lag_grad = Symbolics.gradient(lag, Z)
    try 
    lag_hess = Symbolics.sparsejacobian(lag_grad, Z)
    catch e
        println(e)
    end
    println("before expression2")
    (hess_rows, hess_cols, hess_vals) = findnz(lag_hess)
    println("before expression3")
    
    expression = Val{false}

    full_cost_fn = let
        cost_fn = Symbolics.build_function(cost_val, [Z; X¹; X²; X³; size¹; size²; size³; pt_la; pt_lb; pt_ra; pt_rb]; expression)
        (Z, X¹, X², X³, size¹, size², size³, pt_la, pt_lb, pt_ra, pt_rb) -> cost_fn([Z; X¹; X²; X³; size¹; size²; size³; pt_la; pt_lb; pt_ra; pt_rb])
    end

    full_cost_grad_fn = let
        cost_grad_fn! = Symbolics.build_function(cost_grad, [Z; X¹; X²; X³; size¹; size²; size³; pt_la; pt_lb; pt_ra; pt_rb]; expression)[2]
        (grad, Z, X¹, X², X³, size¹, size², size³, pt_la, pt_lb, pt_ra, pt_rb) -> cost_grad_fn!(grad, [Z; X¹; X²; X³; size¹; size²; size³; pt_la; pt_lb; pt_ra; pt_rb])
    end

    full_constraint_fn = let
        constraint_fn! = Symbolics.build_function(constraints_val, [Z; X¹; X²; X³; size¹; size²; size³; pt_la; pt_lb; pt_ra; pt_rb]; expression)[2]
        (cons, Z, X¹, X², X³, size¹, size², size³, pt_la, pt_lb, pt_ra, pt_rb) -> constraint_fn!(cons, [Z; X¹; X²; X³; size¹; size²; size³; pt_la; pt_lb; pt_ra; pt_rb])
    end

    full_constraint_jac_vals_fn = let
        constraint_jac_vals_fn! = Symbolics.build_function(jac_vals, [Z; X¹; X²; X³; size¹; size²; size³; pt_la; pt_lb; pt_ra; pt_rb]; expression)[2]
        (vals, Z, X¹, X², X³, size¹, size², size³, pt_la, pt_lb, pt_ra, pt_rb) -> constraint_jac_vals_fn!(vals, [Z; X¹; X²; X³; size¹; size²; size³; pt_la; pt_lb; pt_ra; pt_rb])
    end
    
    full_hess_vals_fn = let
        hess_vals_fn! = Symbolics.build_function(hess_vals, [Z; X¹; X²; X³; size¹; size²; size³; pt_la; pt_lb; pt_ra; pt_rb; λ; cost_scaling]; expression)[2]
        (vals, Z, X¹, X², X³, size¹, size², size³, pt_la, pt_lb, pt_ra, pt_rb, λ, cost_scaling) -> hess_vals_fn!(vals, [Z; X¹; X²; X³; size¹; size²; size³; pt_la; pt_lb; pt_ra; pt_rb; λ; cost_scaling])
    end

    full_constraint_jac_triplet = (; jac_rows, jac_cols, full_constraint_jac_vals_fn)
    full_lag_hess_triplet = (; hess_rows, hess_cols, full_hess_vals_fn)
    println("call_back finish")

    return (; full_cost_fn, 
            full_cost_grad_fn, 
            full_constraint_fn, 
            full_constraint_jac_triplet, 
            full_lag_hess_triplet,
            constraints_lb,
            constraints_ub)
end

"""
Predict a dummy trajectory for other vehicles.
"""
function constant_velocity_prediction(X0, L, steering_angle, trajectory_length, timestep)
    # println("predict start")
    X = X0
    X = evolve_state(X, [0; 0; 0; wheel_angle_to_angular_velocity(X[3:5], steering_angle, L)], timestep)
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
    V1 = X[3] + Δ * U[1] 
    V2 = X[4] + Δ * U[2] 
    V3 = X[5] + Δ * U[3] 
    θ = X[6] + Δ * U[4]
    X1 = X[1] + Δ * V1
    X2 = X[2] + Δ * V2
    # println("evolve finish")
    return [X1; X2; V1; V2; V3; θ]
end

# function lane_constraint(X, size, pt_a, pt_b, flag)
#     println("lane start")
#     # 计算车辆的四个角的位置
#     corners = get_vehicle_corners(X, size)
    
#     # 最小距离初始化为一个较大的正值，我们将寻找最小的超过边界的距离
#     max_overbound_dist = 0

#     # 遍历所有线段
#     for i in 1:length(pt_a)
#         # 获取线段的切线和法线
#         tangent = get_tangent_symbolic(pt_a[i], pt_b[i])
#         normal = get_normal_symbolic(tangent)

#         # 根据 flag 确定使用哪边的法线
#         if flag == 1  # 右车道
#             normal = -normal  # 反转法线方向
#         end

#         # 检查每个角是否越界
#         for corner in corners
#             # 计算点到直线（定义为通过 pt_a[i] 和具有方向 normal 的线）的距离
#             distance = dot(corner - pt_a[i], normal)
            
#             # 如果距离大于0，说明角越过了边界
#             if distance > 0
#                 max_overbound_dist = max(max_overbound_dist, distance)
#             end
#         end
#     end
#     println("lane finish")
#     # 如果没有越界，返回 0，否则返回最大越界距离
#     return -max_overbound_dist
# end

# function get_vehicle_corners(X, size)
#     # X 是车辆中心的位置
#     # size 是车辆的长宽高，格式为 [length, width, height]
#     println("corner start")
#     half_length = size[1] / 2
#     half_width = size[2] / 2

#     # 假设车辆局部坐标系中，车辆朝向为 X 的第三个分量
#     theta = X[6]  # 假设 X[3] 是车辆朝向
#     cos_theta = cos(theta)
#     sin_theta = sin(theta)

#     # 计算四个角的全局坐标
#     corners = [
#         X[1:2] + [ cos_theta * half_length - sin_theta * half_width, sin_theta * half_length + cos_theta * half_width],
#         X[1:2] + [ cos_theta * half_length + sin_theta * half_width, sin_theta * half_length - cos_theta * half_width],
#         X[1:2] + [-cos_theta * half_length - sin_theta * half_width, -sin_theta * half_length + cos_theta * half_width],
#         X[1:2] + [-cos_theta * half_length + sin_theta * half_width, -sin_theta * half_length - cos_theta * half_width]
#     ]
#     println("corner finish")
#     return corners
# end


# 假设 X 是符号变量的位置和方向数组，size 是车辆尺寸的符号变量数组
function get_vehicle_corners(X, size)
    # println("corner start")
    corners = []
    half_length = size[1] / 2
    half_width = size[2] / 2
    theta = X[6]
    cos_theta = cos(theta)
    sin_theta = sin(theta)
    # println("corner start")

    # 计算四个角的全局坐标，确保使用 @SVector 正确格式
    # corners = [
    #     @SVector [X[1] + cos_theta * half_length - sin_theta * half_width, X[2] + sin_theta * half_length + cos_theta * half_width],
    #     @SVector [X[1] + cos_theta * half_length + sin_theta * half_width, X[2] + sin_theta * half_length - cos_theta * half_width],
    #     @SVector [X[1] - cos_theta * half_length - sin_theta * half_width, X[2] - sin_theta * half_length + cos_theta * half_width],
    #     @SVector [X[1] - cos_theta * half_length + sin_theta * half_width, X[2] - sin_theta * half_length - cos_theta * half_width]
    # ]
    push!(corners, [X[1] + cos_theta * half_length - sin_theta * half_width, X[2] + sin_theta * half_length + cos_theta * half_width])
    push!(corners, [X[1] + cos_theta * half_length + sin_theta * half_width, X[2] + sin_theta * half_length - cos_theta * half_width])
    push!(corners, [X[1] - cos_theta * half_length - sin_theta * half_width, X[2] - sin_theta * half_length + cos_theta * half_width])
    push!(corners, [X[1] - cos_theta * half_length + sin_theta * half_width, X[2] - sin_theta * half_length - cos_theta * half_width])
    # println("corner finish")
    return corners
end


# 计算点到直线的距离，直线由两点定义，法线指向右边
function point_to_line_distance(point, a, b)
    # 计算线段的法线方向
    tangent = get_tangent_symbolic(a, b)
    normal = get_normal_symbolic(tangent)  # 归一化
    # println("distance start")
    # 计算点到直线的距离
    distance = dot(point - a, normal)
    # println("distance finish")
    return distance
end

# 判断车辆是否在车道内
function lane_constraint(X, size, pt_a, pt_b, flag)
    # println("lane start")
    corners = get_vehicle_corners(X, size)
    num_pts = length(pt_a)
    min_distance = 0

    for i in 1:num_pts-1
        a1 = pt_a[i]
        b1 = pt_b[i]
        a2 = pt_a[i+1]
        b2 = pt_b[i+1]
        pt1 = [a1, b1]
        pt2 = [a2, b2]
        # println("before dist")
        # 检查每个角
        for corner in corners
            dist = point_to_line_distance(corner, pt1, pt2)
            # 如果是右车道边界，法线反向
            dist *= (flag == 1 ? -1 : 1)
            # if dist > 0  # 如果有任何一个角越界
            #     return -abs(dist)  # 返回负的最大越界距离
            # end
            min_distance = min(min_distance, -dist)
        end
    end
    # println("lane finish")
    return min_distance  # 如果所有角都在车道内
end

function collision_constraint(X1, X2, size1, size2)
    # println("collision start")
    # 安全缓冲距离
    buffer = 0.2
    
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


"""
Cost at each stage of the plan
"""
# function stage_cost(X, U, R, pt_la, pt_lb, pt_ra, pt_rb)
#     println("cost start")
#     # 计算速度的模长，鼓励高速可以使用负的系数，这里假设为负以鼓励高速
#     speed_cost = -0.1 * norm(X[4:6])
#     # println("speed finish")
    
#     # 控制成本，反映控制力度
#     control_cost = U' * R * U
#     # println("control finish")

#     # # 路线偏离成本，距离越远成本越高
#     # segments = Vector{MySegment}(undef, 10)  # 预分配长度为10的向量

#     # 遍历每一对端点，计算中心线段
#     mid_points = Vector{SizedVector{2, Real}}(undef, 10)

#     for i in 1:10
#         # 计算中点
#         mid_pt_a = (pt_la[i] + pt_ra[i]) / 2
#         mid_pt_b = (pt_lb[i] + pt_rb[i]) / 2
#         # println("haha")
    
#         # 将计算得到的中点添加到数组中
#         mid_points[i] = (mid_pt_a, mid_pt_b)
#         # println("Mid points stored")
#     end
#     sd, index = signed_distance(mid_points, [X[1], X[2]])
#     println("get success")
#     deviation_cost = 0.5 * sd^2
    
#     # 总成本为各部分的和
#     cost = speed_cost + control_cost + deviation_cost
#     println("cost finish")
#     return cost
# end


function stage_cost(X, U, R, pt_la, pt_ra, pt_lb, pt_rb)
    # println("cost start")
    
    # 计算速度的模长，使用负的系数鼓励高速
    speed_cost = -0.1 * sqrt(X[4]^2 + X[5]^2 + X[6]^2)
    # println("speed finish")
    
    # 控制成本，反映控制力度
    control_cost = U' * R * U
    # println("control finish")

    # 使用向量存储中点
    # mid_points = Vector{SVector{2, Real}}(undef, 10)

    # for i in 1:10
    #     # 计算中点，这里使用符号向量
    #     mid_pt_a = @SVector [(pt_la[i] + pt_ra[i]) / 2, (pt_lb[i] + pt_rb[i]) / 2]
    #     mid_points[i] = mid_pt_a
    #     # println("Mid points stored")
    # end

    # # 使用新的距离计算方式
    # sd, index = signed_distance_symbolic(mid_points, @SVector [X[1], X[2]])
    # println("get success")
    # deviation_cost = 0.5 * sd^2
    
    # 总成本为各部分的和
    cost = speed_cost + control_cost 
    # println("cost finish")
    return cost
end

# function signed_distance_symbolic(mid_points, point)
#     num_pts = length(mid_points)
#     sd = @variables sd[1:num_pts][1]  # 定义符号数组
    
#     for i in 1:num_pts
#         # 简单的欧氏距离计算，适用于符号向量
#         distance = sqrt((mid_points[i][1] - point[1])^2 + (mid_points[i][2] - point[2])^2)
#         sd[i] = distance
#     end

#     # 找到最小绝对距离索引
#     min_abs_index = argmin([@eval abs(sd[$i]) for i in 1:num_pts])
#     return sd[min_abs_index], min_abs_index
# end


"""
Assume z = [U[1];...;U[K];X[1];...;X[K]]
Return states = [X[1], X[2],..., X[K]], controls = [U[1],...,U[K]]
where K = trajectory_length
"""
function decompose_trajectory(z)
    println("decompose start")
    K = Int(length(z) / 10)  # 因为每组包含4个控制元素和6个状态元素，总共是10个元素
    controls = [@view(z[(k-1)*4+1:k*4]) for k = 1:K]  # 每个控制向量有4个元素
    states = [@view(z[4K+(k-1)*6+1:4K+k*6]) for k = 1:K]  # 每个状态向量有6个元素
    println("decompose finish")
    return states, controls
end

function compose_trajectory(states, controls)
    K = length(states)
    z = [reduce(vcat, controls); reduce(vcat, states)]
end



