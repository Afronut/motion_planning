
function bug1_w1(w1, plt1)
    dt = 0
    start = (x = 0, y = 0)
    goal = (x = 10, y = 10)
    x_start = [0];
    y_start = [0];
    x_goal = [10];
    y_goal = [10];
    xx = [];yy = [];
    
    points = []
    jj = 0
    while (true)
        step = start.x + 0.01
        X, Y, obs = start_toward_goal(w1, start, goal, step)
        plot!(plt1, X, Y, legend=false, color=:black, linewidth=4)
        if (X[end] == goal.x && Y[end] == goal.y)
            println("Completed")
            println("Total distance is :", dt)

            return
        end
        
        path = make_path(w1, obs, X[end], Y[end])
        xx, yy, dt = follow_path(path, (x = x_goal[1], y = y_goal[1]))
        start = (x = xx, y = yy)
        
        plot!(plt1, [xx], [yy], legend=false, marker=([:dot :d]))
    
        jj += 1;
    end
    # display(plt1)
end


function get_commun(w1, obs, pointx, pointy)
    first = []
    second = []
    comon_edge = ()
    mid_way_dege = ()
    for arr in w1
        for ii = 1:size(arr, 1)
            first = arr[ii]
            if ii == size(arr, 1)
                second = arr[1]
            else
                second = arr[ii + 1]
            end
            if first.x == pointx && first.y == pointy && arr != obs
                comon_edge = first
                break;
            end
            if second.x == pointx && second.y == pointy && arr != obs
                comon_edge = second
                break;
            end
            if first.x < second.x && pointx in collect(first.x:0.01:second.x) && first.y == pointy
                mid_way_dege = (x = pointx, y = pointy)
                break;
            end
            if first.y < second.y && pointy in collect(first.y:0.01:second.y) && first.x == pointx
                mid_way_dege = (x = pointx, y = pointy)
                break;
            end

            if first.x > second.x && pointx in collect(second.x:0.01:first.x) && first.y == pointy
                mid_way_dege = (x = pointx, y = pointy)
                break;
            end
            if first.y > second.y && pointy in collect(second.y:0.01:first.y) && first.x == pointx
                mid_way_dege = (x = pointx, y = pointy)
                break;
            end
        end
        if !isempty(comon_edge) && !isempty(mid_way_dege)
            return comon_edge, mid_way_dege, arr
        end
    end
    
    return (), (), ()
end


function make_array(cords::Array)
    x = [];
    y = [];
    for tup in cords 
        push!(x, tup.x)
        push!(y, tup.y)
    end
    push!(x, getindex(cords, 1).x)
    push!(y, getindex(cords, 1).y)
    return x, y
end

function path_eqn(start_point::NamedTuple, end_point::NamedTuple)
    slope = (end_point.y - start_point.y) / (end_point.x - start_point.x);
    
    y_cept = end_point.y - slope * end_point.x;
    return slope, y_cept;
end

function check_obstacle(edges::Array, postx, posty)
    slope = 0;y_cept = 0
    edge1 = ()
    edge2 = ()
    for i = 1:size(edges, 1)
        if i == size(edges, 1)
            edge1 = edges[i]
            edge2 = edges[1]
            slope, y_cept = path_eqn(edge1, edge2)
        else    
            edge1 = edges[i]
            edge2 = edges[i + 1]
            slope, y_cept = path_eqn(edge1, edge2)
        end
        if slope == Inf 
            eq_val = posty
        end
        if y_cept == Inf || slope == 0
            eq_val = edge1.y
            
        end
        if slope != Inf || slope != 0
            eqn = Polynomial([y_cept,slope]);
            eq_val = round(eqn(postx), digits=2)
        end                 
        if validate_point(edge1, edge2, (x = postx, y = posty))
            return true
        end
    end
    return false
end

function validate_point(edge1, edge2, point)
    order = []
    if (edge1.x == edge2.x && edge2.x == point.x)
        if (edge1.y < edge2.y && point.y in collect(edge1.y:0.01:edge2.y) )
            return true
        end
        if (edge1.y > edge2.y && point.y in collect(edge2.y:0.01:edge1.y) )
            return true
        end
    end
    if (edge1.y == edge2.y && edge2.y == point.y)
        if (edge1.x < edge2.x && point.x in collect(edge1.x:0.01:edge2.x) )
            return true
        end
        if (edge1.x > edge2.x && point.x in collect(edge2.x:0.01:edge1.x) )
            return true
        end
    end

    return false
end


function start_toward_goal(w1, start, goal, step)
    obs = false
    slope, y_cep = path_eqn(start, goal)
    eqn = Polynomial([y_cep,slope])
    plot!(eqn, step, goal.x)
    X = [];
    Y = [];
    while (step <= goal.x)
        eq_val = round(eqn(step), digits=2)
        push!(X, step);
        push!(Y, eq_val)
        for arr in w1
            obs = check_obstacle(arr, step, eq_val)
            if (obs)
                return X, Y, arr;
            end
        end
        step = round(step + 0.01, digits=2)
    end
    return [start.x, goal.x], [start.y, goal.y], []
end

function make_path(w1, obs, obx, oby)
    path = []
    for i = 1:size(obs, 1) 
        comon_edge, mid_way_dege, array = get_commun(w1, obs, obs[i].x, obs[i].y)
        if (isempty(comon_edge) && isempty(mid_way_dege))
            push!(path, obs[i])
        elseif (!isempty(comon_edge) && !isempty(mid_way_dege))
            path = [(x = obx, y = oby),(x = 3, y = 4), (x = 4, y = 4), (x = 4, y = 12), 
            (x = 12, y = 12), (x = 12, y = 6),(x = 6, y = 6),
            (x = 6, y = 5),(x = 12, y = 5),(x = 13, y = 5), (x = 13, y = 13),
            (x = 12, y = 13),(x = 3, y = 13),(x = 3, y = 12),(x = obx, y = oby)]
        end

    end
    return path
end


function follow_path(path, goal)
    X = [];
    Y = [];
    dist_total = 0
    start = path[1];
    pointx = typemax(Int32)
    pointy = typemax(Int32)
    rpointx = typemax(Int32)
    rpointy = typemax(Int32)
    shortest_dis = typemax(Int32)
    old_shotest_dis = 0
    next_point = (x = typemax(Int32), y = typemax(Int32))
    Xp = []
    Yp = []
    retx = []
    rety = []
    for i = 1:size(path, 1) 
        
        if (i + 1 ) > size(path, 1) 
            i = 0
        end
        next_point = path[i + 1];
        if start.x != next_point.x
            if start.x < next_point.x
                X = collect(LinRange(start.x, next_point.x, 100))
            else
                X = collect(LinRange(next_point.x, start.x, 100))
            end
            if start.y == next_point.y
                Y = next_point.y * ones(size(X, 1))
            end
        end
        if start.y != next_point.y
            if start.y < next_point.y
                Y = collect(LinRange(start.y, next_point.y, 100))
            else
                Y = collect(LinRange(next_point.y, start.y, 100))
            end
            if start.x == next_point.x
                X = next_point.x * ones(size(Y, 1))
            end
        end
        push!(Xp, start.x)
        push!(Yp, start.y)
        push!(retx, X)
        push!(rety, Y)
            
        dis, pointx, pointy = shortest_distance(goal, (x = X, y = Y))
        if dis <= shortest_dis
            shortest_dis = dis
            rpointx = round(pointx, digits=1) 
            rpointy = round(pointy, digits=1) 
        end
        dist_total += round(sqrt((start.x - next_point.x)^2 + (start.y - next_point.y)^2))
        start = next_point   

    end
    push!(Xp, Xp[1]), push!(Yp, Yp[1])
    plot!(Xp, Yp, legend=false, color=:black, linewidth=4)
    return rpointx, rpointy, dist_total
end

# bug2()

function shortest_distance(goal, points)
    shortest_dis = typemax(Int32);
    pointx = typemax(Int32)
    pointy = typemax(Int32)
    dis = 0;
    for i = 1:size(points.x, 1)
        dis =  round(sqrt((goal.x - points.x[i])^2 + (goal.y - points.y[i])^2), digits=2)
        if dis < shortest_dis
            shortest_dis = dis;
            pointx = round(points.x[i], digits=1)
            pointy = round(points.y[i], digits=1)
            
        end
    end
    return shortest_dis, pointx, pointy;
end


function bug1_w2(w2)
    goal = (x = 35, y = 0)
    start = (x = 0, y = 0)
    path = [(x = 30, y = 0), (x = 29, y = 0), (x = 29, y = 5), (x = 20, y = 5), 
    (x = 20, y = 0), (x = 19, y = 0), (x = 19, y = 5), (x = 10, y = 5),
    (x = 10, y = 0), (x = 9, y = 0), (x = 9, y = 5), (x = -5, y = 5), 
    (x = -5, y = -5), (x = 4, y = -5), (x = 4, y = 1), (x = 5, y = 1), 
    (x = 5, y = -5), (x = 14, y = -5), (x = 14, y = 1), (x = 15, y = 1),
     (x = 15, y = -5), (x = 24, y = -5), (x = 24, y = 1), (x = 25, y = 1),
     (x = 25, y = -6),(x = -6, y = -6), (x = -6, y = 6), (x = 30, y = 6) ,
     (x = 30, y = 0)]
     
    dist = follower(path, goal)
     #  (x=30, y=0), (x=35, y=0)]
    println("Total Distance is : ", dist)

    plot!([30, goal.x], [ goal.y, goal.y], legend=false, color=:black, arrow=:arrow, linewidth=4)
    plot!([start.x, 4], [ start.y, 0], legend=false, color=:black, arrow=:arrow, linewidth=4)
end


function follower(path, goal)
    
    start = path[1];
    next_point = ()
    dist_total = 0
    for i = 1:size(path, 1) - 1
        
        if (i + 1 ) > size(path, 1) 
            i = 0
        end
        next_point = path[i + 1];
        # println(start, next_point)
        plot!([start.x,  next_point.x], [  start.y, next_point.y], legend=false, arrow=:arrow, color=:black, linewidth=4)
        dist_total += round(sqrt((start.x - next_point.x)^2 + (start.y - next_point.y)^2))
        start = next_point;
    end
    # plot!([ goal.x, path[1].x], [ goal.y, path[1].y], legend=false, color=:blue, linewidth=1)
    return dist_total
end
# bug1()

