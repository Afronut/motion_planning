import Pkg; Pkg.add("Plots"); Pkg.add("Polynomials")
using Plots
using Polynomials 
include( "bug_1.jl")
include( "bug_2.jl")



W_O = Dict("w1" => [
                        [(x = 1, y = 1), (x = 2, y = 1), (x = 2, y = 5),(x = 1, y = 5)],
                        [(x = 3, y = 4), (x = 4, y = 4), (x = 4, y = 12),(x = 3, y = 12)],
                        [(x = 3, y = 12), (x = 12, y = 12), (x = 12, y = 13),(x = 3, y = 13)],
                        [(x = 12, y = 5), (x = 13, y = 5), (x = 13, y = 13),(x = 12, y = 13)],
                        [(x = 6, y = 5), (x = 12, y = 5), (x = 12, y = 6),(x = 6, y = 6)]
                    ],
            "w2" => [
                        [(x = -6, y = -6), (x = 25, y = -6), (x = 25, y = -5),(x = -6, y = -5)],
                        [(x = -6, y = 5), (x = 30, y = 5), (x = 30, y = 6),(x = -6, y = 6)],
                        [(x = -6, y = -5), (x = -5, y = -5), (x = -5, y = 5),(x = -6, y = 5)],
                        [(x = 4, y = -5), (x = 5, y = -5), (x = 5, y = 1),(x = 4, y = 1)],
                        [(x = 9, y = 0), (x = 10, y = 0), (x = 10, y = 5),(x = 9, y = 5)],
                        [(x = 14, y = -5), (x = 15, y = -5), (x = 15, y = 1),(x = 14, y = 1)],
                        [(x = 19, y = 0), (x = 20, y = 0), (x = 20, y = 5),(x = 19, y = 5)],
                        [(x = 24, y = -5), (x = 25, y = -5), (x = 25, y = 1),(x = 24, y = 1)],
                        [(x = 29, y = 0), (x = 30, y = 0), (x = 30, y = 5),(x = 29, y = 5)],
                    ]
    );
# display(plot([1,2],[10,3]))
w1 = get(W_O, "w1", 0);

w2 = get(W_O, "w2", 0);

function main(args)

    if isempty(args)
        println("\n --*--*--*--*")
        println("No argument provided. type: julia hw_1.jl help")
        return
    end
    if args[1] == "help"
        println("Usgae : \n
    bug1 -w1: bug 1, workspace 1\n 
    bug1 -w2: bug 1, workspace 2\n
    bug2 -w1: bug 2, workspace 1\n
    bug2 -w2: bug 2, workspace 2")
    end
    if (args[1] == "bug1")
       
        if args[2] == "-w1"
            start = (x = 0, y = 0)
            goal = (x = 10, y = 10)
            x_start = [0];
            y_start = [0];
            x_goal = [10];
            y_goal = [10];
            xx = [];yy = [];
            points = []
            plt1 = plot(x_start, y_start, legend=false, marker=([:dot :d]))
            for arr in w1
                points = make_array(arr)
                plot!(plt1, points, legend=false)
            end
            plot!(plt1, x_goal, y_goal, marker=([:dot :d]))
            bug1_w1(w1, plt1)
            png("bug1_w1")
        elseif args[2] == "-w2"
            x_start = [0];
            y_start = [0];
            x_goal = [35];
            y_goal = [0];
            points = []
            goal = (x = 35, y = 0);
            start = (x = 0, y = 0)
            plt2 = plot(x_start, y_start, legend=false, marker=([:hex :d]))
            for arr in w2
                points = make_array(arr)
                plot!(plt2, points, legend=false)
            end
            plot!(plt2, x_goal, y_goal, marker=([:hex :d]))   
            bug1_w2(w2)
            png("bug1_w2")
        end

        return

    end
    if (args[1 ] == "bug2")
        if args[2] == "-w1"
            start = (x = 0, y = 0)
            goal = (x = 10, y = 10)
            x_start = [0];
            y_start = [0];
            x_goal = [10];
            y_goal = [10];
            xx = [];yy = [];
            points = []
            plt1 = plot(x_start, y_start, legend=false, marker=([:dot :d]))
            for arr in w1
                points = make_array(arr)
                plot!(plt1, points, legend=false)
            end
            plot!(plt1, x_goal, y_goal, marker=([:dot :d]))
            bug2_w1()
            png("bug2_w1")
        elseif args[2] == "-w2"
            x_start = [0];
            y_start = [0];
            x_goal = [35];
            y_goal = [0];
            points = []
            goal = (x = 35, y = 0);
            start = (x = 0, y = 0)
            plt2 = plot(x_start, y_start, legend=false, marker=([:hex :d]))
            for arr in w2
                points = make_array(arr)
                plot!(plt2, points, legend=false)
            end
            plot!(plt2, x_goal, y_goal, marker=([:hex :d]))   
            bug2_w2(w2)
            png("bug2_w2")
        end

        
        return
    end


end


main(ARGS)