# Accident scenario in SUMO: two cars collide

A 600 m, 2-lane highway with cars and vans. At x = 300 m on the right lane, `crash_car`
stops suddenly and `hit_car` (behind it) does not brake and hits it. Both block the lane
for 300 s, then drive on.

## Files
| File | What it is |
|---|---|
| `highway.nod.xml`, `highway.edg.xml` | Plain description of the road (2 nodes, 1 edge with 2 lanes) |
| `highway.net.xml` | The SUMO network built from the two files above |
| `routes.rou.xml` | Vehicle types (car, van), background traffic, and the two crash cars |
| `accident.sumocfg` | Simulation configuration (network + routes, time settings) |
| `sumo_accident.ipynb` | Notebook: runs the simulation with TraCI, creates the collision, measures and plots |

## How to run
Open `sumo_accident.ipynb` and run the cells in order. In the Settings cell, set `FOLDER`
to the folder that contains these files (use `"."` if the notebook is in the same folder).

If SUMO reports a problem with the network file, set `REBUILD_NETWORK = True` in the notebook, or run:
```
netconvert --node-files highway.nod.xml --edge-files highway.edg.xml -o highway.net.xml
```
