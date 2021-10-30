using Printf
using Plots


Plots.pyplot();

max_num_iter = 1000;
T_init = 1000;
T_final = 0.1;
global T_current = T_init;
beta = (T_init - T_final) / max_num_iter;
alpha = 0.9;
linear_cooling = 1;
check_probability = 1;
global curr_iter = 0;
init_sol = rand(-10.0:0.001:10.0);
global curr_solution = init_sol;
global best_solution = init_sol;



function target_function(x)
    return x^2
end


# function myPlot(x)
#     print("xxxxx")
#     scatter!([x],[target_function(x)],color="red",markersize=10)
#     display(f)
# end


function main()
    x = range(-11,stop=11,length=10000); y = x.^2;
    p = plot(x,y,legend=false)
    display(p)
    while curr_iter < max_num_iter
        new_solution = curr_solution + rand(-1.0:0.001:1.0);
        while !(new_solution ≥ -10.0 && new_solution ≤ 10.0)
            new_solution = curr_solution + rand(-1.0:0.001:1.0);
        end
        cost_diff = target_function(new_solution) - target_function(curr_solution);
        probability = exp(-cost_diff/T_current);
        if(cost_diff < 0)
            curr_solution = new_solution;
        else
            if(probability > rand())
                global curr_solution = new_solution;
            end
        end
        if(target_function(curr_solution) < target_function(best_solution))
            global best_solution = curr_solution;
        end
        global T_current = T_init - beta * curr_iter;
        # print(new_solution)
        # print("\n")
        global curr_iter+=1;
    end
    @printf("Best Solution is %f" , best_solution)
end

main()