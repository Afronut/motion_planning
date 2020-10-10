
function bug2_w2(w2)
    x_start = [0];
    y_start = [0];
    x_goal = [35];
    y_goal = [0];
    points = []
    plt2 = plot(x_start, y_start, legend=false, marker=([:hex :d]))
    for arr in w2
        points = make_array(arr)
        plot!(plt2, points, legend=false)
    end
    plot!(plt2, x_goal, y_goal, marker=([:hex :d]))
    display(plt2)
end
