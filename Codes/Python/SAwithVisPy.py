import time
import numpy as np
from vispy.plot import Fig

max_num_iter = 1000
T_init = 1000
T_final = 1
beta = (T_init - T_final) / max_num_iter
alpha = 0.9
linear_cooling = 1
check_probability = 1
init_sol = np.random.uniform(-10,10)
fig = Fig(size=(1000,1000),show=False)
ax1 = fig[0,0]
ax2 = fig[0,1]

# def animate(i):



def target_function(x):
    return x**2



def main():
    x = np.linspace(-10,10,10000)
    y = target_function(x)
    ax1.plot(x,y)
    fig.show(run=True)

    curr_iter = 0
    curr_solution = init_sol
    best_solution = init_sol
    T_current = T_init


    while curr_iter < max_num_iter:
        new_solution = curr_solution + np.random.uniform(-1,1)
        while not(new_solution >= -10.0 and new_solution <= 10.0):
            new_solution = curr_solution + np.random.uniform(-1,1)
        cost_diff = target_function(new_solution) - target_function(curr_solution)
        probability = np.exp(-cost_diff/T_current)
        if(cost_diff < 0):
            curr_solution = new_solution
            if(probability > np.random.random()):
                curr_solution = new_solution
        if(target_function(curr_solution) < target_function(best_solution)):
            best_solution = curr_solution
        T_current -= beta
        print(curr_solution)
        # print(T_current)
        # time.sleep(0.001)
        curr_iter+=1
    print("Best Solution is" , best_solution)

main()