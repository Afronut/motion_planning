
include( "bug_1.jl")

function bug2_w1()
    plt = plot!()
    goal = (x = 10, y = 10)
    path = [(x = 0, y = 0), (x = 1, y = 1), (x = 1, y = 5), 
(x = 2, y = 5), (x = 2, y = 2), (x = 4, y = 4),
 (x = 3, y = 4), (x = 3, y = 12), (x = 3, y = 13),
  (x = 12, y = 13), (x = 13, y = 13),(x = 13, y = 5),
  (x = 12, y = 5),(x = 6, y = 5), (x = 6, y = 6), goal]
    dist_total = follower(path, goal)
    println("Total Distance is : ", dist_total)
end


function bug2_w2(w2)
    goal = (x = 35, y = 0)
    start=(x=0, y=0)
    path = [(x = 0, y = 0),(x = 4, y = 0), (x = 4, y = 1), (x = 5, y = 1), 
    (x = 5, y = 0), (x = 9, y = 0),(x = 9, y = 5), (x = -5, y = 5), (x = -5, y = 0), (x = 0, y = 0)]# (x=10, y=0),(x=14,y=0), (x=14,y=1),
     
    plot!([start.x, 4], [ start.y, 0], legend=false, color=:black, arrow=:arrow, linewidth=4)
    dist = follower(path, goal)
    println("Total Distance is : ", dist)
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