## surf-pathfinder

this is an experimental, work-in-progress project that will eventually calculate perfect routes along surf ramps, adjusting dynamically to game conditions and player actions, for usage in TAS creation and as a guiding line for surfers.

## features

- brachistochrone curve optimization for determining the fastest path along surf ramps
- real-time path adjustment based on changing game conditions and player actions
- efficient collision detection using the Gilbert-Johnson-Keerthi (GJK) algorithm and bounding volume hierarchy (BVH)
- accurate numerical methods for solving ordinary differential equations (ODEs) and integration
- advanced optimization strategies, including conjugate gradient and BFGS methods, with line search techniques
- pathfinding algorithms like A* for efficient deviation checking and path recalibration
- utilization of SIMD instructions and GPU acceleration for high-performance mathematical computations
- caching mechanisms for frequently used data and calculations
- thread-safe synchronization using mutexes for accessing shared data structures

## surf_ramp entities  

`SurfRampOptimizer::OnEntityCreated` function is called whenever a new entity is created. it checks if the entity's classname is "surf_ramp" and performs the necessary initialization and optimization steps.  

when an entity with the classname "surf_ramp" is created:
1. a touch hook is set on the entity using `pEntity->SetTouch(OnStartTouch)`, which allows detecting when a player touches the ramp  
2. the `GetRampFromEntity` function is called to extract the ramp information from the entity, such as start point, end point, normal, length, width, and extents  
3. a new instance of `CBrachistochroneOptimizer` is created with the extracted ramp information and added to the `g_SurfRamps` vector  
4. the ramp is inserted into the collision detection system using `CollisionDetection::InsertRamp(&ramp)`  

example surf_ramp entity:  
```
classname "surf_ramp"  
startpoint "0 0 0"  
endpoint "100 0 0"  
normal "0 0 1"  
length "100"  
width "50"  
```  
  
- startpoint: starting position of the ramp 
- endpoint: the ending position of the ramp 
- normal: the surface normal of the ramp  
- length: the length of the ramp  
- width: the width of the ramp  

- minExtents: this vector represents the minimum x, y, and z coordinates that the ramp occupies in the 3D space. it defines the lower boundary or corner of the ramp's bounding volume

- maxExtents: same thing, but the opposite

extents are calculated from ramp properties and used for collision detection

TODO: implement functionality to search, view, edit ramp properties in `g_SurfRamps`    
TODO: implement functionality to easily create and destroy surf_ramp entities

## optimization process

the optimization process for a surf ramp and player follows these steps:

1. when a player touches a surf ramp entity, the `CBrachistochroneOptimizer` class is instantiated with the ramp and player information
2. the `Optimize` function is called to determine the fastest path along the ramp (brachistochrone curve optimization)
3. the optimized path is stored in the `m_OptimizedPath` member variable of the `CBrachistochroneOptimizer` instance
4. the `Update` function is called periodically to adjust the path in real-time based on changing game conditions and player actions
5. if the player deviates significantly from the optimized path, the `RecalibratePath` function of the `PlayerMovementTracking` class is called to find a new path using the A* algorithm.

TODO: implement visualization with tempents or something  
TODO: implement functionality to search, view and edit the optimized path data for a specific player and ramp

## handling ramp collisions:

1. uses the GJK algorithm and bounding volume hierarchy (BVH) for collision detection  
2. automatically handles collisions between players and surf_ramp entities  
- TODO: perform additional collision checks and handle custom collision scenarios utilizing the collision detection functions provided in the CollisionDetection namespace  

## optimization parameters

- MAX_PLAYER_DIST_TO_PATH: max distance a player can deviate from the optimized path before triggering a recalibration
- OPTIMIZATION_METHOD: optimization method to use
- GRADIENT_TOLERANCE: tolerance threshold for the gradient descent optimization
- MAX_OPTIMIZATION_ITERATIONS: maximum number of iterations for the optimization process
- COLLISION_DETECTION_RESOLUTION: resolution of the collision detection grid

TODO: dynamically adjusting optimization parameters based on server performance and different game states

## general todo

- integration with more advanced physics engines or libraries for even more accurate simulations and predictions
- utilization of machine learning techniques to adapt and improve the optimization process based on historical data and patterns
- implementation of a user-friendly interface or API for easier integration and customization of the extension
- expansion of the extension to support additional game modes, mechanics, or environments beyond surf ramps
- implementation of dynamic variables and config files (current implementation has many hardcoded magic numbers for debugging)
- memory cleanup and error handling
- better documentation and commenting
- further profiling and optimization for potentially expensive computations (currently utilizes SIMD instructions and GPU acceleration for performance and scalability)
- parallel processing to perform CPU intensive complex calculations, such as path optimization or physics simulations with multiple cores 
- implement extensive logging 

## surf_ramp_optimizer.h

### classes
- `SurfRampOptimizer`: main extension class that handles initialization, entity creation/destruction, and overall management of the optimization process.
- `CBrachistochroneOptimizer`: handles the brachistochrone curve optimization for a specific surf ramp and player.
- `PlayerMovementTracking`: tracks player movements and states, and recalibrates the path when deviations occur.
- `BVHNode`: represents a node in the bounding volume hierarchy used for efficient collision detection.
- `Matrix`: utility class for matrix operations.
- `Vector`: utility class for vector operations.
- `Ramp`: represents a surf ramp entity with properties like start point, end point, normal, length, width, and extents.

### namespaces:
- `CollisionDetection`: contains functions for collision detection, including the GJK algorithm and intersection tests.
- `NumericalMethods`: provides numerical methods for solving ODEs, integration, and gradient calculations.
- `MathUtils`: contains utility functions for mathematical operations, including SIMD-optimized vector operations.
- `Optimization`: implements optimization algorithms and strategies, such as conjugate gradient, BFGS, and line search techniques.
- `Pathfinding`: contains the A* pathfinding algorithm for efficient path recalibration.

## surf_ramp_optimizer.cpp

### SurfRampOptimizer
- `SurfRampOptimizer::SDK_OnLoad`: initializes game interfaces and configurations.
- `SurfRampOptimizer::SDK_OnUnload`: cleans up resources and removes entity listeners.
- `SurfRampOptimizer::SDK_OnAllLoaded`: initializes collision detection and logs the server tickrate.
- `SurfRampOptimizer::OnEntityCreated`: handles the creation of new surf ramp entities and initializes optimization.
- `SurfRampOptimizer::OnEntityDestroyed`: handles the destruction of surf ramp entities and cleans up resources.
- `SurfRampOptimizer::OnStartTouch`: handles the event when a player touches a surf ramp and triggers optimization.

### CBrachistochroneOptimizer
- `CBrachistochroneOptimizer::Optimize`: determines the fastest path along the ramp using brachistochrone curve optimization.
- `CBrachistochroneOptimizer::Update`: adjusts the path in real-time based on changing game conditions and player actions.
- `CBrachistochroneOptimizer::PerformFlyCollisionResolution`: resolves collisions during the optimization process.
- `CBrachistochroneOptimizer::IsPathValid`: checks if a given point is valid on the path.

### PlayerMovementTracking
- `PlayerMovementTracking::UpdatePlayerState`: updates the player's state with position, velocity, and acceleration.
- `PlayerMovementTracking::EstimatePlayerPosition`: estimates the player's position based on the current state.
- `PlayerMovementTracking::HasDeviatedFromPath`: checks if the player has deviated significantly from the optimized path.
- `PlayerMovementTracking::RecalibratePath`: recalibrates the path using the A* algorithm if the player deviates.

### BVHNode
- `BVHNode::Insert`: inserts a ramp into the bounding volume hierarchy.
- `BVHNode::Query`: queries the bounding volume hierarchy for ramps intersecting a given point.

### CollisionDetection
- `CollisionDetection::Initialize`: initializes the collision detection system.
- `CollisionDetection::Shutdown`: shuts down the collision detection system.
- `CollisionDetection::InsertRamp`: inserts a ramp into the collision detection system.
- `CollisionDetection::Intersects`: checks if a point intersects with a ramp using the GJK algorithm.
- `CollisionDetection::GJKIntersection`: performs the GJK intersection test between two ramps.
- `CollisionDetection::SupportPoint`: finds the support point in a given direction for the GJK algorithm.
- `CollisionDetection::FarthestPointInDirection`: finds the farthest point in a given direction on a ramp.
- `CollisionDetection::UpdateSimplex`: updates the simplex for the GJK algorithm.

### NumericalMethods
- `NumericalMethods::SolveBrachistochrone`: solves the brachistochrone problem using adaptive methods.
- `NumericalMethods::CalculateBrachistochronePathTime`: calculates the time required for the brachistochrone path.
- `NumericalMethods::CalculateGradient`: calculates the gradient for optimization.
- `NumericalMethods::GaussianQuadrature`: performs Gaussian quadrature for numerical integration.
- `NumericalMethods::DormandPrince54`: solves ODEs using the Dormand-Prince method.

### MathUtils
- `MathUtils::SIMDCrossProduct`: calculates the cross product using SIMD instructions.
- `MathUtils::SIMDDotProduct`: calculates the dot product using SIMD instructions.
- `MathUtils::SIMDLength`: calculates the length of a vector using SIMD instructions.
- `MathUtils::SIMDNormalized`: normalizes a vector using SIMD instructions.
- `MathUtils::ClipVelocity`: clips the velocity vector based on a collision normal.

### Optimization
- `Optimization::OptimizePath`: optimizes the path using the BFGS method.
- `Optimization::OptimizePathConjugateGradient`: optimizea the path using the conjugate gradient method.
- `Optimization::OptimizePathBFGS`: optimizes the path using the BFGS method.
- `Optimization::SatisfiesWolfeConditions`: checks if the Wolfe conditions are satisfied for line search.
- `Optimization::LineSearch`: performs line search for optimization.
- `Optimization::CalculatePathCost`: calculates the cost of a path.
- `Optimization::GetClosestPointOnRamp`: finds the closest point on a ramp.
- `Optimization::UpdateInverseHessian`: updates the inverse Hessian matrix for BFGS optimization.

### Pathfinding
- `Pathfinding::Pathfinding`: implements the A* pathfinding algorithm for efficient path recalibration.

### Misc
- `GetRampFromEntity`: extracts ramp information from a given entity.
