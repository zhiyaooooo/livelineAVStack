struct MyLocalizationType
    time::Float64
    vehicle_id::Int
    position::SVector{3, Float64} # position of center of vehicle
    orientation::SVector{4, Float64} # represented as quaternion
    velocity::SVector{3, Float64}
    angular_velocity::SVector{3, Float64} # angular velocity around x,y,z axes
    size::SVector{3, Float64} # length, width, height of 3d bounding box centered at (position/orientation)
    map_segment::Int
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

function Rot_from_quat(q)
    qw = q[1]
    qx = q[2]
    qy = q[3]
    qz = q[4]

    R = [qw^2+qx^2-qy^2-qz^2 2(qx*qy-qw*qz) 2(qw*qy+qx*qz);
         2(qx*qy+qw*qz) qw^2-qx^2+qy^2-qz^2 2(qy*qz-qw*qx);
         2(qx*qz-qw*qy) 2(qw*qx+qy*qz) qw^2-qx^2-qy^2+qz^2]
end

# first apply T2, then T1
function multiply_transforms(T1, T2)
    T1f = [T1; [0 0 0 1.]] 
    T2f = [T2; [0 0 0 1.]]

    T = T1f * T2f
    T = T[1:3, :]
end

# from photo to vehicle
function get_rotated_camera_transform()
    R = [0 0 1.;
         -1 0 0;
         0 -1 0]
    t = zeros(3)
    [R t]
end

# from camera to vehicle
function get_cam_transform(camera_id)
    # TODO load this from URDF
    R_cam_to_body = RotY(0.02)
    t_cam_to_body = [1.35, 1.7, 2.4]
    if camera_id == 2
        t_cam_to_body[2] = -1.7
    end

    T = [R_cam_to_body t_cam_to_body]
end

# from body to global
function get_body_transform(quat, loc)
    R = Rot_from_quat(quat)
    [R loc]
end

function invert_transform(T)
    R = T[1:3,1:3]
    t = T[1:3,end]
    R\[I(3) -t]
end

function get_3d_bbox_corners(orientation, position, box_size)
    quat = orientation # quatnerion
    xyz = position # position
    T = get_body_transform(quat, xyz)
    corners = []
    for dx in [-box_size[1]/2, box_size[1]/2]
        for dy in [-box_size[2]/2, box_size[2]/2]
            for dz in [-box_size[3]/2, box_size[3]/2]
                push!(corners, T*[dx, dy, dz, 1])
            end
        end
    end
    corners
end

function convert_to_pixel(num_pixels, pixel_len, px)
    min_val = -pixel_len*num_pixels/2
    pix_id = cld(px - min_val, pixel_len)+1 |> Int
    return pix_id
end

# from world to two cameras
function get_camera_from_world(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2)
    # from body to world
    T_world_body = get_body_transform(ego_orientation, ego_position)
    # from camera to world
    T_world_camrot1 = multiply_transforms(T_world_body, T_body_camrot1)
    T_world_camrot2 = multiply_transforms(T_world_body, T_body_camrot2)
    # from world to camera
    T_camrot1_world = invert_transform(T_world_camrot1)
    T_camrot2_world = invert_transform(T_world_camrot2)
    return T_camrot1_world, T_camrot2_world
end

# get 2d bboxes pixels from object state and ego state
function get_2d_bboxes(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, obj_orientation, obj_position, vehicle_size, image_width, image_height, pixel_len, focal_len)
    # from world to two cameras
    T_camrot1_world, T_camrot2_world = get_camera_from_world(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2)
    # 8 corners in world coordinate
    corners_body = get_3d_bbox_corners(obj_orientation, obj_position, vehicle_size) 

    # project the object to camera1 to get bboxes
    left = image_width +1
    right = 0
    top = image_height + 1
    bot = 0
    other_vehicle_corners_cam1 = [T_camrot1_world * [pt;1] for pt in corners_body]

    for corner in other_vehicle_corners_cam1
        if corner[3] < focal_len
            break
        end
        px = focal_len*corner[1]/corner[3]
        py = focal_len*corner[2]/corner[3]
        px = convert_to_pixel(image_height, pixel_len, px)
        py = convert_to_pixel(image_height, pixel_len, py)
        left = min(left, px)
        right = max(right, px)
        top = min(top, py)
        bot = max(bot, py)
    end
    bboxes_cam1 = []

    if !(left>image_width || top>image_height || right < 1 || bot < 1) # if obj inside the frame
        top = max(top, 1)
        left = max(left, 1)
        bot = min(bot, image_height)
        right = min(right, image_width)
        push!(bboxes_cam1, SVector(top, left, bot, right))
    end


    # project the object to camera2 to get bboxes
    left = image_width +1
    right = 0
    top = image_height + 1
    bot = 0
    other_vehicle_corners_cam2 = [T_camrot2_world * [pt;1] for pt in corners_body]

    for corner in other_vehicle_corners_cam2
        if corner[3] < focal_len
            break
        end
        px = focal_len*corner[1]/corner[3]
        py = focal_len*corner[2]/corner[3]
        px = convert_to_pixel(image_height, pixel_len, px)
        py = convert_to_pixel(image_height, pixel_len, py)
        left = min(left, px)
        right = max(right, px)
        top = min(top, py)
        bot = max(bot, py)
    end
    bboxes_cam2 = []

    if !(left>image_width || top>image_height || right < 1 || bot < 1) # if obj inside the frame
        top = max(top, 1)
        left = max(left, 1)
        bot = min(bot, image_height)
        right = min(right, image_width)
        push!(bboxes_cam2, SVector(top, left, bot, right))
    end
    return bboxes_cam1, bboxes_cam2
end


# get location points in a region
function uniform_location_points(center, half_x, half_y, step)
    points = []
    push!(points, center)
    for x in center[1] - half_x:step:center[1] + half_x
        for y in center[2] - half_y:step:center[2] + half_y
            push!(points, [x, y, center[3]])
        end
    end
    return points
end


function generate_points_from_segment(segment_points, middle_segment, height; step_size=10)
    start_point, end_point = middle_segment
    direction = normalize(end_point - start_point)
    start_point = start_point + direction
    num_steps = ceil(Int, LinearAlgebra.norm(end_point - start_point) / step_size)
    
    for i in 0:num_steps
        point = start_point + i * step_size * direction
        push!(segment_points, [point[1], point[2], height])
    end
    
    return segment_points
end


function generate_location_points_from_map(map, height)
    middle_segments = []
    for k in keys(map)
        for i in 1:(length(map[k].lane_boundaries)-1)
            start_point = (map[k].lane_boundaries[i].pt_a + map[k].lane_boundaries[i+1].pt_a)/2
            end_point = (map[k].lane_boundaries[i].pt_b + map[k].lane_boundaries[i+1].pt_b)/2
            push!(middle_segments, [start_point, end_point])
        end        
    end

    segment_points = []
    for middle_segment in middle_segments
        segment_points = generate_points_from_segment(segment_points, middle_segment, height; step_size=10)
    end

    return segment_points
end

# angle from -pi to pi
function angle_to_quaternion(x)
    return [cos(x/2), 0, 0, sin(x/2)]
end

function quaternion_to_angle(quat)
    return asin(quat[4])*2
end

# input (0, pi, step) means from -pi to pi, i.e. all directions
function uniform_quaternion_points(center_angle, half_angle, step)
    vector_of_vectors = []
    for x in center_angle - half_angle:step:center_angle + half_angle - 0.01
        push!(vector_of_vectors, angle_to_quaternion(x))
    end
    return vector_of_vectors
end

# given ego quaternion and location and bboxes from 2 cameras, return estimated location region
function estimated_location_from_2_bboxes(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, 
    vehicle_size, image_width, image_height, pixel_len, focal_len,
    quat_points, location_points, true_bboxes_cam1, true_bboxes_cam2; step = 10)
    quat_loc_bboxeserror_list=[]
    min_error = 1000
    for quat_point in quat_points
        for location_point in location_points
            bboxes_cam1, bboxes_cam2 = get_2d_bboxes(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, quat_point, location_point, vehicle_size, image_width, image_height, pixel_len, focal_len)
            if length(bboxes_cam1) == 1 && length(bboxes_cam2) == 1
                #cam1_error = bboxes_cam1[1] - true_bboxes_cam1
                #cam2_error = bboxes_cam2[1] - true_bboxes_cam2
                #bboxes_error = norm(norm(cam1_error), norm(cam2_error))
                
                cam1_error = abs.(bboxes_cam1[1] - true_bboxes_cam1)
                cam2_error = abs.(bboxes_cam2[1] - true_bboxes_cam2)
                bboxes_error = sum(cam1_error) + sum(cam2_error)
                if bboxes_error <= min_error
                    min_error = bboxes_error
                    push!(quat_loc_bboxeserror_list, [quat_point, location_point, bboxes_error])
                end
            end
        end
    end

    quat_loc_minerror_list = [v for v in quat_loc_bboxeserror_list if v[3] == min_error]

    angle_loc_list = [[quaternion_to_angle(v[1]), v[2]] for v in quat_loc_minerror_list]
    min_x = minimum([v[2][1] for v in angle_loc_list])
    max_x = maximum([v[2][1] for v in angle_loc_list])
    min_y = minimum([v[2][2] for v in angle_loc_list])
    max_y = maximum([v[2][2] for v in angle_loc_list])
    min_z = minimum([v[2][3] for v in angle_loc_list])
    max_z = maximum([v[2][3] for v in angle_loc_list])

    estimated_center = [(min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2]
    half_x = (max_x-min_x)/2 + step
    half_y = (max_y-min_y)/2 + step
    return estimated_center, half_x, half_y, quat_loc_minerror_list
end

# given ego state and bboxes of one object, estimate accurate orientation and location of this object 
function initialize_location(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, 
                            vehicle_size, image_width, image_height, pixel_len, focal_len,
                            quat_points, location_points, true_bboxes_cam1, true_bboxes_cam2; step = 10)
    
    estimated_center, half_x, half_y, quat_loc_minerror_list = estimated_location_from_2_bboxes(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, 
                                                                                                vehicle_size, image_width, image_height, pixel_len, focal_len,
                                                                                                quat_points, location_points, true_bboxes_cam1, true_bboxes_cam2; step = step)

    num_of_location_points = 121
    step = 2 * sqrt(half_x * half_y / num_of_location_points)
    location_points = uniform_location_points(estimated_center, half_x, half_y, step)
    estimated_center, half_x, half_y, quat_loc_minerror_list = estimated_location_from_2_bboxes(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, 
                                                                                                vehicle_size, image_width, image_height, pixel_len, focal_len,
                                                                                                quat_points, location_points, true_bboxes_cam1, true_bboxes_cam2; step = step)

    num_of_location_points = 50
    step = 2 * sqrt(half_x * half_y / num_of_location_points)
    location_points = uniform_location_points(estimated_center, half_x, half_y, step)
    estimated_center, half_x, half_y, quat_loc_minerror_list = estimated_location_from_2_bboxes(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, 
                                                                                                vehicle_size, image_width, image_height, pixel_len, focal_len,
                                                                                                quat_points, location_points, true_bboxes_cam1, true_bboxes_cam2; step = step)
    return quat_loc_minerror_list
end

function initialize_particles(quat_loc_minerror_list, vehicle_size; var_angle = pi/12, var_location = 0.5, max_v = 7.5, step_v = 0.5, number_of_particles = 1000)
    
    (len, wid, hei) = vehicle_size
    particles = []
    covariance = Diagonal([var_location^2, var_location^2, var_angle^2])
    n = length(quat_loc_minerror_list)
    average_number_of_particles = Int(number_of_particles ÷ n ÷ (max_v ÷ step_v))

    for v in 0:step_v:max_v
        for quat_loc in quat_loc_minerror_list

            quat = quat_loc[1]
            angle = quaternion_to_angle(quat)
            x, y, z = quat_loc[2]
            push!(particles, [x, y, z, v, angle, len, wid, hei])

            for i in 1: average_number_of_particles
                multi_normal = MultivariateNormal([x, y, angle], covariance)
                xyangle = vec(rand(multi_normal, 1))
                push!(particles, [xyangle[1], xyangle[2], z, v, xyangle[3], len, wid, hei])
            end
        end
    end
    quat_loc = quat_loc_minerror_list[1]
    quat = quat_loc[1]
    angle = quaternion_to_angle(quat)
    x, y, z = quat_loc[2]
    best_particle = [x, y, z, max_v/2, angle, len, wid, hei]
    return particles, best_particle
end

function process(particle, delta_t)
    particle[1] = particle[1] + particle[4] * cos(particle[5]) * delta_t
    particle[2] = particle[2] + particle[4] * sin(particle[5]) * delta_t
    return particle
end

# calculate error between two pairs of bboxes
function bboxes_error(bbox_cam1, bbox_cam2, obj_bboxes_cam)
    err_1 = abs.(bbox_cam1 - obj_bboxes_cam[1])
    err_2 = abs.(bbox_cam2 - obj_bboxes_cam[2])
    err = sum(err_1) + sum(err_2)
    return err
end

# assign a score according booxes error
function score_from_bboxes_error(bbox_cam1, bbox_cam2, obj_bboxes_cam)
    err = bboxes_error(bbox_cam1, bbox_cam2, obj_bboxes_cam)
    return 1/(1+err)
end


# obj_bboxes_cam =[bbox_cam1, bbox_cam2] is new perception 
function update_particles(particles, delta_t, obj_bboxes_cam, ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, image_width, image_height, pixel_len, focal_len)
    #update particles using process model
    for i in 1:length(particles)
        particles[i] = process(particles[i], delta_t)
    end
    """need to change 3,3 to other values"""
    covariance = Diagonal([0.1^2, 0.1^2, 0, 0.16^2, (pi/36)^2, 0.0, 0.0, 0.0])
    scores = zeros(length(particles))

    for i in 1:length(particles)
        multi_normal = MultivariateNormal(particles[i], covariance)
        particles[i] = vec(rand(multi_normal, 1))
    
        bboxes_cam1, bboxes_cam2 = get_2d_bboxes(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, angle_to_quaternion(particles[i][5]), particles[i][1:3], particles[i][6:8], image_width, image_height, pixel_len, focal_len)
        """#if always have exactly correct bboxes
        if length(bboxes_cam1) == 1 && length(bboxes_cam2) == 1 && bboxes_cam1[1] == obj_bboxes_cam[1] && bboxes_cam2[1] == obj_bboxes_cam[2]
            scores[i] = 1
        else
            scores[i] = 0
        end
        """
        if length(bboxes_cam1) == 1 && length(bboxes_cam2) == 1
            scores[i] = score_from_bboxes_error(bboxes_cam1[1], bboxes_cam2[1], obj_bboxes_cam)
        end
    end
    scores = normalize(scores)
    scores[1:30] = [.1, .1, .05, .05, .05, .05, .025, .025, .025, .025, .025, .025, .025, .025, .0125, .0125, .0125, .0125, .0125, .0125, .0125, .0125, .0125, .0125, .0125, .0125, .0125, .0125, .0125, .0125]
    
    return particles, scores
end

function resample(particles, scores)
    samples = sample(particles[1:100], Weights(scores[1:100]), length(scores), replace=true)
    return samples
end

function sort_particles_by_scores(particles, scores)
    sorted_indices = sortperm(scores, rev=true)

    sorted_particles = particles[sorted_indices]
    sorted_scores = scores[sorted_indices]
    return sorted_particles, sorted_scores
end

function weighted_average(particles, scores)
    weighted_sum = sum(p .* s for (p, s) in zip(particles, scores))
    total_weight = sum(scores)
    return weighted_sum / total_weight
end

function distance_to_out_of_photo(bbox, image_width, image_height)
    top, left, bot, right = bbox[1], bbox[2], bbox[3], bbox[4]
    # distance to go beyond top, left, bot, right boundary
    dis = [bot, right, image_height-top, image_width-left] 
    return minimum(dis)
end

function delete_most_marginalized_bbox(true_bboxes, image_width, image_height)
    distances = [distance_to_out_of_photo(x, image_width, image_height) for x in true_bboxes]
    min_index = argmin(distances)
    deleted_bboxes = copy(true_bboxes)
    deleteat!(deleted_bboxes, min_index)
    return deleted_bboxes
end

function pair_bboxes(bboxes_cam1, bboxes_cam2)
    object = []
    for i in 1:length(bboxes_cam1)
        push!(object, [bboxes_cam1[i], bboxes_cam2[i]])
    end
    return object
end

function get_objects(fresh_cam_meas, image_width, image_height)
    # process two latest camera message
    if fresh_cam_meas[end].camera_id == 1
        true_bboxes_cam1 = fresh_cam_meas[end].bounding_boxes
        true_bboxes_cam2 = fresh_cam_meas[end-1].bounding_boxes
    else
        true_bboxes_cam2 = fresh_cam_meas[end].bounding_boxes
        true_bboxes_cam1 = fresh_cam_meas[end-1].bounding_boxes
    end

    number_of_redundent_bboxes = length(true_bboxes_cam1) - length(true_bboxes_cam2)
    objects = []
    deleted_bboxes=[]
    if number_of_redundent_bboxes == 0
        objects = pair_bboxes(true_bboxes_cam1, true_bboxes_cam2)
    elseif number_of_redundent_bboxes > 0
        deleted_bboxes = true_bboxes_cam1
        for i in 1:number_of_redundent_bboxes
            deleted_bboxes = delete_most_marginalized_bbox(deleted_bboxes, image_width, image_height)
        end
        objects = pair_bboxes(deleted_bboxes, true_bboxes_cam2)
    elseif number_of_redundent_bboxes < 0
        deleted_bboxes = true_bboxes_cam2
        for i in 1:-number_of_redundent_bboxes
            deleted_bboxes = delete_most_marginalized_bbox(deleted_bboxes, image_width, image_height)
        end
        objects = pair_bboxes(true_bboxes_cam1, deleted_bboxes)
    end
    return objects
end

function calculate_error_from_particle(bboxes_of_object, particle, ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, vehicle_size, image_width, image_height, pixel_len, focal_len)
    quat = angle_to_quaternion(particle[5])
    location = particle[1:3]
    
    bbox_cam1, bbox_cam2 = get_2d_bboxes(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, quat, location, vehicle_size, image_width, image_height, pixel_len, focal_len)

    return bboxes_error(bbox_cam1[1], bbox_cam2[1], bboxes_of_object)    
end

function update_tracks(tracks, bboxes_of_objects, time_now, ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, 
    vehicle_size, image_width, image_height, pixel_len, focal_len, quat_points, location_points)
    # if there is no tracks known but new bboxes perceived, add initialized tracks 
    if length(tracks[end][2]) == 0
        if length(bboxes_of_objects) > 0
            particles_of_objects = []
            best_particle_of_objects = []
            for bboxes_of_object in bboxes_of_objects
                quat_loc_minerror_list = initialize_location(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, 
                                vehicle_size, image_width, image_height, pixel_len, focal_len,
                                quat_points, location_points, bboxes_of_object[1], bboxes_of_object[2]; step = 10)
                # particle = [x, y, z, v, theta, length, width, height]
                particles, best_particle = initialize_particles(quat_loc_minerror_list, vehicle_size; var_angle = pi/12, var_location = 0.5, max_v = 7.5, step_v = 0.5, number_of_particles = 1000)
                push!(particles_of_objects, particles)
                push!(best_particle_of_objects, best_particle)
            end
            push!(tracks, [time_now, particles_of_objects, best_particle_of_objects])
        end

    # if there are tracks but no bboxes, add empty tracks
    elseif length(bboxes_of_objects) == 0
        push!(tracks, [time_now, [], []])

    # if there are tracks and bboxes, pair them up
    else
        # calculate error between track i and object j
        matrix = zeros(length(tracks[end][2]), length(bboxes_of_objects))
        for i in 1:length(tracks[end][2])
            for j in 1:length(bboxes_of_objects)
                particles = tracks[end][2][i]
                particle = particles[1]
                bboxes_of_object = bboxes_of_objects[j]
                matrix[i,j] = calculate_error_from_particle(bboxes_of_object, particle, ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, vehicle_size, image_width, image_height, pixel_len, focal_len)
            end
        end

        time_previous = tracks[end][1]
        delta_t = time_now - time_previous
        particles_of_objects = []
        best_particle_of_objects = []
        # every track get a object, and add new objects
        if length(tracks[end][2]) < length(bboxes_of_objects)
            # update existing particles of objects
            J = []
            for i in 1:length(tracks[end][2])
                j = argmin(matrix[i, :])
                push!(J, j)
                bboxes_of_object = bboxes_of_objects[j]
                particles = tracks[end][2][i]

                # update particles
                particles, scores = update_particles(particles, delta_t, bboxes_of_object, ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, image_width, image_height, pixel_len, focal_len)
                particles, scores = sort_particles_by_scores(particles, scores)
                particles = resample(particles, scores)
                best_particle = mean(particles)

                push!(particles_of_objects, particles)
                push!(best_particle_of_objects, best_particle)
            end

            # add new objects which don't match existing tracks
            for j in 1:length(bboxes_of_objects)
                if !(j in J)
                    bboxes_of_object = bboxes_of_objects[j]
                    quat_loc_minerror_list = initialize_location(ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, 
                                    vehicle_size, image_width, image_height, pixel_len, focal_len,
                                    quat_points, location_points, bboxes_of_object[1], bboxes_of_object[2]; step = 10)
                    # particle = [x, y, z, v, theta, length, width, height]
                    particles, best_particle = initialize_particles(quat_loc_minerror_list, vehicle_size; var_angle = pi/12, var_location = 0.5, max_v = 7.5, step_v = 0.5, number_of_particles = 1000)
                    push!(particles_of_objects, particles)
                    push!(best_particle_of_objects, best_particle)
                end
            end

        # every object get a track, and maybe delete some existing tracks
        else #length(tracks[end][2]) >= length(bboxes_of_objects)
            # update existing particles of objects
            for j in 1:length(bboxes_of_objects)
                i = argmin(matrix[:, j])
                bboxes_of_object = bboxes_of_objects[j]
                particles = tracks[end][2][i]

                # update particles
                particles, scores = update_particles(particles, delta_t, bboxes_of_object, ego_orientation, ego_position, T_body_camrot1, T_body_camrot2, image_width, image_height, pixel_len, focal_len)
                particles, scores = sort_particles_by_scores(particles, scores)
                particles = resample(particles, scores)
                best_particle = mean(particles)

                push!(particles_of_objects, particles)
                push!(best_particle_of_objects, best_particle)
            end
        end
        push!(tracks, [time_now, particles_of_objects, best_particle_of_objects])
    end
end





















function localize(gps_channel, imu_channel, localization_state_channel)
    # Set up algorithm / initialize variables
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

        localization_state = MyLocalizationType(0,0.0)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
end

# add a parameter map
function perception(cam_meas_channel, localization_state_channel, perception_state_channel, map)
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

    @async localize(gps_channel, imu_channel, localization_state_channel)
    @async perception(cam_channel, localization_state_channel, perception_state_channel, map)
    @async decision_making(localization_state_channel, perception_state_channel, map, socket)
end
