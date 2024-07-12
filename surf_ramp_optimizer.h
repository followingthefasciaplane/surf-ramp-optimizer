#pragma once

#include <vector>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <optional>

class CBaseEntity;
class CBrachistochroneOptimizer;
class PlayerMovementTracking;
class BVHNode;
class Matrix;
class Vector;
class Ramp;

namespace CollisionDetection {}
namespace NumericalMethods {}
namespace MathUtils {}
namespace Optimization {}
namespace Pathfinding {}

constexpr float MAX_PLAYER_DIST_TO_PATH = 50.0f;
constexpr float GRADIENT_TOLERANCE = 1e-6f;
constexpr int MAX_OPTIMIZATION_ITERATIONS = 100;

class SurfRampOptimizer : public SDKExtension
{
public:
    virtual bool SDK_OnLoad(char* error, size_t maxlength, bool late);
    virtual void SDK_OnUnload();
    virtual void SDK_OnAllLoaded();
    virtual bool QueryRunning(char* error, size_t maxlength);

    void OnEntityCreated(CBaseEntity* pEntity, const char* classname);
    void OnEntityDestroyed(CBaseEntity* pEntity);

private:
    bool SetupGameInterfaces(char* error, size_t maxlength);
    bool LoadConfigurations(char* error, size_t maxlength);
    void InitializeDataStructures();
    static void OnStartTouch(CBaseEntity* pOther);
};

class CBrachistochroneOptimizer
{
public:
    CBrachistochroneOptimizer(const Ramp& ramp, CBaseEntity* player);
    std::vector<Vector> Optimize(const Vector& startPos, const Vector& startVel, float tickInterval);
    void Update(const Vector& playerPos, const Vector& playerVel, const Vector& playerAccel, float tickInterval);
    bool IsPathValid(const Vector& point) const;
    const Ramp& GetRamp() const;
    const std::vector<Vector>& GetPath() const;
    CBaseEntity* GetPlayer() const;

private:
    void PerformFlyCollisionResolution(trace_t& pm, Vector& move);

    Ramp m_Ramp;
    CBaseEntity* m_Player;
    std::vector<Vector> m_OptimizedPath;
    float m_AirAccelerate;
    float m_Gravity;
    PlayerMovementTracking m_PlayerTracker;
};

class PlayerMovementTracking
{
public:
    PlayerMovementTracking(float maxDeviationDistance, size_t bufferCapacity = 1000);
    void UpdatePlayerState(const Vector& position, const Vector& velocity, const Vector& acceleration, float tickInterval);
    Vector EstimatePlayerPosition(float timeStep) const;
    bool HasDeviatedFromPath(const std::vector<Vector>& path) const;
    void RecalibratePath(const std::vector<Vector>& currentPath, const std::vector<Ramp>& ramps, CBrachistochroneOptimizer& optimizer, float tickInterval);

private:
    float m_MaxDeviationDistance;
    CUtlVector<Vector> m_Positions;
    CUtlVector<Vector> m_Velocities;
    CUtlVector<Vector> m_Accelerations;
    CUtlVector<float> m_TimeIntervals;
};

class BVHNode
{
public:
    BVHNode(const Vector& min, const Vector& max);
    ~BVHNode();
    void Insert(Ramp* ramp);
    std::vector<Ramp*> Query(const Vector& point) const;

private:
    Vector minExtents;
    Vector maxExtents;
    BVHNode* left;
    BVHNode* right;
    std::vector<Ramp*> objects;
};

class Matrix
{
public:
    Matrix(int rows = 3, int cols = 3);
    Matrix(const Matrix& other);
    ~Matrix();
    static Matrix Identity(int size);
    static Matrix OuterProduct(const Vector& u, const Vector& v);
    Matrix& operator=(const Matrix& other);
    Matrix operator+(const Matrix& other) const;
    Matrix operator-(const Matrix& other) const;
    Matrix operator*(const Matrix& other) const;
    Vector operator*(const Vector& v) const;
    int Rows() const;
    int Cols() const;

private:
    int m_rows;
    int m_cols;
    float* m_data;
};

class Vector
{
public:
    Vector();
    Vector(float x, float y, float z);
    Vector(float x, float y, float z, float v, float w, float u);
    float LengthSqr() const;
    float Length() const;
    Vector Normalized() const;
    Vector Cross(const Vector& other) const;
    float Dot(const Vector& other) const;
    Vector operator+(const Vector& other) const;
    Vector operator-(const Vector& other) const;
    Vector operator*(float scalar) const;
    Vector operator/(float scalar) const;
    bool operator==(const Vector& other) const;
    bool operator!=(const Vector& other) const;

    float x, y, z;
    float v, w, u;
};

class Ramp
{
public:
    bool operator==(const Ramp& other) const;

    Vector startPoint;
    Vector endPoint;
    Vector normal;
    float length;
    float width;
    Vector minExtents;
    Vector maxExtents;
};

namespace CollisionDetection
{
    void Initialize();
    void Shutdown();
    void InsertRamp(Ramp* ramp);
    bool Intersects(const Vector& point, const Ramp& ramp);
    bool GJKIntersection(const Ramp& ramp1, const Ramp& ramp2);
    Vector SupportPoint(const Ramp& ramp1, const Ramp& ramp2, const Vector& direction);
    Vector FarthestPointInDirection(const Ramp& ramp, const Vector& direction);
    bool UpdateSimplex(Vector* simplex, Vector& direction, int index);
}

namespace NumericalMethods
{
    Vector SolveBrachistochrone(const Vector& startPos, const Vector& endPos, float time, float gravity, float airAccelerate);
    Vector SolveBrachistochroneAdaptive(const Vector& startPos, const Vector& endPos, float time, float gravity, float airAccelerate, float errorTolerance);
    float CalculateBrachistochronePathTime(const Vector& startPos, const Vector& endPos, float gravity);
    Vector CalculateGradient(const Vector& point, const Ramp& ramp, float gravity, float airAccelerate);
    float GaussianQuadrature(float a, float b, int n, float (*f)(float, void*), void* params);
    Vector DormandPrince54(float t0, float tf, const Vector& y0, float (*f)(float, Vector, void*), void* params, float& h);
}

namespace MathUtils
{
    Vector SIMDCrossProduct(const Vector& a, const Vector& b);
    float SIMDDotProduct(const Vector& a, const Vector& b);
    float SIMDLength(const Vector& v);
    Vector SIMDNormalized(const Vector& v);
    void ClipVelocity(Vector& in, Vector& normal, Vector& out, float overbounce);
}

namespace Optimization
{
    std::vector<Vector> OptimizePath(const std::vector<Vector>& path, const Ramp& ramp, float gravity, float airAccelerate);
    std::vector<Vector> OptimizePathConjugateGradient(const std::vector<Vector>& path, const Ramp& ramp, float gravity, float airAccelerate);
    std::vector<Vector> OptimizePathBFGS(const std::vector<Vector>& path, const Ramp& ramp, float gravity, float airAccelerate);
    bool SatisfiesWolfeConditions(const Vector& point, const Vector& direction, float alpha, const Ramp& ramp, float gravity, float airAccelerate, float c1, float c2);
    std::pair<float, bool> LineSearch(const Vector& point, const Vector& direction, const Ramp& ramp, float gravity, float airAccelerate);
    float CalculatePathCost(const Vector& point, const Ramp& ramp, float gravity, float airAccelerate);
    Vector GetClosestPointOnRamp(const Vector& point, const Ramp& ramp);
    void UpdateInverseHessian(Matrix& inverseHessian, const Vector& direction, const Vector& gradientDifference);
}

namespace Pathfinding
{
    std::optional<std::vector<Vector>> Pathfinding(const Vector& start, const Vector& goal, const std::vector<Ramp>& ramps);
}

Ramp GetRampFromEntity(CBaseEntity* pEntity);