# 2D Rigid Body Physics Engine

Features:
- rigid bodies (convex polygons, circles, capsules)
- fixed timestep, with variable substeps
- quadtree backed broad phase using shape bounding box
- GJK/EPA based collision detection
- single contact point is generated per collision detection
- persistent collisions build manifold over multiple steps using best contact points
- iterative velocity-based impulse solver
- coulomb friction
- islands and sleeping
 

Still lacking:
- time-of-impact/swept collision detection
- literally any other constraints
- probably lots of other things