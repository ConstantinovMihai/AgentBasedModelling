import numpy as np
import matplotlib.pyplot as plt
import pickle

 # load the dictionary with the results
with open('saved_dictionary_Prioritized.pkl', 'rb') as f:
    save_data = pickle.load(f)
    # [results, min_agents, max_agents, min_map, nb_maps, nb_spawns]

print(save_data[0])
  
feature_x =np.arange(save_data[1], save_data[2]+1, 1 ,dtype=int).tolist()
feature_y =np.arange(save_data[3], save_data[4]+1, 1 ,dtype=int).tolist()

# Creating 2-D grid of features
[X, Y] = np.meshgrid(feature_x, feature_y)

fig, ((ax1,ax3,ax5),(ax2,ax4,ax6)) = plt.subplots(2,3)
 
Z = np.zeros((len(feature_y),len(feature_x)))

for x in range(len(feature_x)):
    for y in range(len(feature_y)):            
        Z[y,x] = save_data[0][f"map_{y+save_data[3]}-agent_{x+save_data[1]}-spawn_{0}"]['mean_time']
print(Z)
ax1.contourf(X, Y, Z)
ax1.set_title('Unidirectional agent movement prio')
ax1.set_xlabel('number of agents')
ax1.set_ylabel('map complexity')

for x in range(len(feature_x)):
    for y in range(len(feature_y)):            
        Z[y,x] = save_data[0][f"map_{y+save_data[3]}-agent_{x+save_data[1]}-spawn_{1}"]['mean_time']
print(Z)
ax2.contourf(X, Y, Z)
ax2.set_title('Omnidirectional agent movement prio')
ax2.set_xlabel('number of agents')
ax2.set_ylabel('map complexity')



with open('saved_dictionary_Distributed.pkl', 'rb') as f:
    save_data = pickle.load(f)
    # [results, min_agents, max_agents, min_map, nb_maps, nb_spawns]

print(save_data[0])

for x in range(len(feature_x)):
    for y in range(len(feature_y)):            
        Z[y,x] = save_data[0][f"map_{y+save_data[3]}-agent_{x+save_data[1]}-spawn_{0}"]['mean_time']
print(Z)
ax3.contourf(X, Y, Z)
ax3.set_title('Unidirectional agent movement dis')
ax3.set_xlabel('number of agents')
ax3.set_ylabel('map complexity')

for x in range(len(feature_x)):
    for y in range(len(feature_y)):            
        Z[y,x] = save_data[0][f"map_{y+save_data[3]}-agent_{x+save_data[1]}-spawn_{1}"]['mean_time']
print(Z)
ax4.contourf(X, Y, Z)
ax4.set_title('Omnidirectional agent movement dis')
ax4.set_xlabel('number of agents')
ax4.set_ylabel('map complexity')

with open('saved_dictionary_CBS.pkl', 'rb') as f:
    save_data = pickle.load(f)
    # [results, min_agents, max_agents, min_map, nb_maps, nb_spawns]

print(save_data[0])

for x in range(len(feature_x)):
    for y in range(len(feature_y)):            
        Z[y,x] = save_data[0][f"map_{y+save_data[3]}-agent_{x+save_data[1]}-spawn_{0}"]['mean_time']
print(Z)
ax5.contourf(X, Y, Z)
ax5.set_title('Unidirectional agent movement cbs')
ax5.set_xlabel('number of agents')
ax5.set_ylabel('map complexity')

for x in range(len(feature_x)):
    for y in range(len(feature_y)):            
        Z[y,x] = save_data[0][f"map_{y+save_data[3]}-agent_{x+save_data[1]}-spawn_{1}"]['mean_time']
print(Z)
ax6.contourf(X, Y, Z)
ax6.set_title('Omnidirectional agent movement cbs')
ax6.set_xlabel('number of agents')
ax6.set_ylabel('map complexity')


  
plt.show()