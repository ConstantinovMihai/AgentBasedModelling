import numpy as np
import matplotlib.pyplot as plt
import pickle

 # load the dictionary with the results
with open('code\saved_dictionary_Prioritized.pkl', 'rb') as f:
    save_data = pickle.load(f)
    # [results, min_agents, max_agents, min_map, nb_maps, nb_spawns]
    print(save_data)
    print(save_data[1])
    print(save_data[2])
  
feature_x = np.linspace(save_data[1], save_data[2], 1 ,dtype=int)
feature_y = np.linspace(save_data[3], save_data[4], 1 ,dtype=int)

print(feature_x)
print(feature_y)
  
# Creating 2-D grid of features
[X, Y] = np.meshgrid(feature_x, feature_y)

  
for spawn_type in save_data[5]:

    fig, ax = plt.subplots(1, spawn_type)
    
    # Z = X ** 2 + Y ** 2
    Z = save_data[0][f"map_{Y}-agent_{X}-spawn_{spawn_type}"]
    
    # plots filled contour plot
    ax.contourf(X, Y, Z)
    
    ax.set_title('Filled Contour Plot')
    ax.set_xlabel('feature_x')
    ax.set_ylabel('feature_y')
  
plt.show()